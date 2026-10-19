#!/usr/bin/env python3
"""Offline prediction of the active-localization effect (T-011b).

    python3 -I tools/predict_effect.py --out /home/claude/team/predict

Plans the experiment scenario `shelves_loop_a` on a costmap built from the
world geometry (`_world_grid.py`) and compares three variants per leg:

  baseline  no layer, heading = direction of travel
  yaw-only  no layer, headings from the yaw DP (`yaw_planner.plan_yaws`, spec §4)
  active    LocalizabilityLayer (spec §3, c_max in {20, 40, 80}) + yaw DP

and writes into --out: prediction.md (tables), prediction.json (all numbers),
predict_paths.png (paths + headings over s*), predict_cmax_sweep.png (routes
over the layer cost for the c_max sweep).

Approximations (all [assumption], repeated in prediction.md):
  * costmap = rasterised world + Nav2 inflation formula (`_world_grid.py`),
    not the RTAB-Map grid; no obstacle/denoise layers;
  * planner = Dijkstra on a 16-connected grid with Theta*'s cost model, then a
    cost-aware line-of-sight shortcut pass (any-angle), instead of Nav2's
    Theta* search (which is not optimal and stops at its heuristic order);
    no smoother;
  * robot follows the planned path exactly: constant speed v, heading changes
    limited only by wz_max (omni base: translation does not depend on heading).

Theta* cost model (nav2_theta_star_planner, Humble, theta_star.hpp):
  g = sum over segments  w_euc * |segment| [cells]
      + sum over cells visited  w_trav * (26 + 0.9 c)^2 / 252^2,
  a cell is traversable iff 26 + 0.9 c <= 252 (c <= 251: INSCRIBED/LETHAL blocked).

Deterministic: no randomness; scipy Dijkstra and numpy only.
"""

import argparse
import json
import math
import os
import sys
from dataclasses import asdict, dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from scipy.sparse import csr_matrix  # noqa: E402
from scipy.sparse.csgraph import dijkstra  # noqa: E402

from _common import LAYER, LANDMARKS_REL, PKG_DIR, build_scorer  # noqa: E402
from _world_grid import (INNER_BOX, MAX_NON_OBSTACLE, SHELF_BOXES,  # noqa: E402
                         WorldGridParams, build_world_costmap, cell_to_world,
                         combine_layer, world_to_cell)
from active_localization.cost_field import (best_score_grid,  # noqa: E402
                                            layer_cost_from_score)
from active_localization.geometry import (path_headings,  # noqa: E402
                                          resample_polyline, wrap_angle)
from active_localization.yaw_planner import YawPlannerParams, plan_yaws  # noqa: E402

CONFIG_FILE = os.path.join(PKG_DIR, 'config', 'active_localization.yaml')

# Scenario shelves_loop_a (ENG, branch T-009d): start, then goals in order.
SCENARIO = {
    'name': 'shelves_loop_a',
    'start': (-2.0, -0.5, 0.0),
    'goals': [(2.6, -0.8, math.pi / 2), (-0.5, 2.45, math.pi / 2), (-2.0, -0.5, 0.0)],
}

C_MAX_MAIN = (20, 40, 80)
# c_max >= 252 makes every non-band cell untraversable for Theta* (26 + 0.9 c > 252).
C_MAX_SWEEP = (10, 20, 40, 60, 80, 120, 160, 250)
W_F_SWEEP = (0.1, 0.3, 1.0)
W_TRAV_SWEEP = (0.5, 1.0, 2.0)
# Route-lever probe (not part of the scenario): a straight leg ~2.3 m in front of
# row 1, i.e. outside the band; shows which c_max bends a route into the band.
PROBE_LEG = ((-3.0, -1.4), 0.0, (2.8, -1.4), 0.0)

# Motion model for the time estimate [assumption]: constant speed, turn rate limit.
V_LIN = 0.5      # m/s (task T-011b)
WZ_MAX = 1.0     # rad/s (MPPI wz_max, nav2_params_v2.yaml)
GOOD = 0.5       # threshold for s and n_eff


# ---------------------------------------------------------------- planner ---

@dataclass
class PlannerParams:
    w_euc_cost: float = 1.0          # nav2_params_v2.yaml GridBased
    w_traversal_cost: float = 0.5    # nav2_params_v2.yaml GridBased
    smooth: bool = True              # any-angle line-of-sight pass


def traversal_cost(cost, w_traversal_cost=0.5):
    """Theta* per-cell traversal cost w * (26 + 0.9 c)^2 / 252^2."""
    c = 26.0 + 0.9 * np.asarray(cost, float)
    return w_traversal_cost * c * c / float(MAX_NON_OBSTACLE) ** 2


def traversable(cost):
    """Theta* isSafe (known cells): 26 + 0.9 c <= 252."""
    return 26.0 + 0.9 * np.asarray(cost, float) <= MAX_NON_OBSTACLE


_OFFSETS = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1),
            (2, 1), (2, -1), (-2, 1), (-2, -1), (1, 2), (1, -2), (-1, 2), (-1, -2))


def _mid_cells(dx, dy):
    """Cells a knight move passes besides the target (both candidates)."""
    if abs(dx) == 2:
        sx = int(np.sign(dx))
        return ((sx, 0), (sx, dy))
    if abs(dy) == 2:
        sy = int(np.sign(dy))
        return ((0, sy), (dx, sy))
    return ()


class GridPlanner:
    """Theta*-cost Dijkstra (16-connected) + cost-aware line-of-sight shortcuts.

    `cost` is a (H, W) uint8 master costmap, row = y. Cells are (ix, iy).
    Knight moves pay the mean traversal cost of the two cells they cross and
    need both of them traversable [assumption: conservative vs Bresenham].
    """

    def __init__(self, cost, params: PlannerParams = PlannerParams()):
        self.cost = np.asarray(cost)
        self.p = params
        self.t = traversal_cost(self.cost, params.w_traversal_cost)
        self.safe = traversable(self.cost)
        H, W = self.cost.shape
        self.shape = (H, W)
        iy, ix = np.mgrid[0:H, 0:W]

        def inside(x, y):
            return (x >= 0) & (x < W) & (y >= 0) & (y < H)

        def at(a, x, y):
            return a[np.clip(y, 0, H - 1), np.clip(x, 0, W - 1)]

        src, dst, wts = [], [], []
        for dx, dy in _OFFSETS:
            tx, ty = ix + dx, iy + dy
            ok = self.safe & inside(tx, ty) & at(self.safe, tx, ty)
            w = params.w_euc_cost * math.hypot(dx, dy) + at(self.t, tx, ty)
            for mx, my in _mid_cells(dx, dy):
                ok &= inside(ix + mx, iy + my) & at(self.safe, ix + mx, iy + my)
                w = w + 0.5 * at(self.t, ix + mx, iy + my)
            src.append((iy * W + ix)[ok])
            dst.append((ty * W + tx)[ok])
            wts.append(w[ok])
        n = H * W
        self.graph = csr_matrix((np.concatenate(wts), (np.concatenate(src),
                                                       np.concatenate(dst))), shape=(n, n))

    def segment_cost(self, a, b):
        """Theta* line-of-sight cost a -> b (cells), None if not traversable."""
        dx, dy = int(b[0] - a[0]), int(b[1] - a[1])
        n = max(abs(dx), abs(dy))
        if n == 0:
            return 0.0
        k = np.arange(1, n + 1)
        xs = a[0] + np.rint(k * dx / n).astype(int)
        ys = a[1] + np.rint(k * dy / n).astype(int)
        if not self.safe[ys, xs].all():
            return None
        return self.p.w_euc_cost * math.hypot(dx, dy) + float(self.t[ys, xs].sum())

    def plan_cells(self, start, goal):
        """Vertex cells (K, 2) [ix, iy] and the plan cost (Theta* units)."""
        H, W = self.shape
        for c, what in ((start, 'start'), (goal, 'goal')):
            if not (0 <= c[0] < W and 0 <= c[1] < H) or not self.safe[c[1], c[0]]:
                raise ValueError(f'{what} cell {tuple(c)} is not traversable')
        s = int(start[1] * W + start[0])
        g = int(goal[1] * W + goal[0])
        dist, pred = dijkstra(self.graph, directed=True, indices=s, return_predecessors=True)
        if not np.isfinite(dist[g]):
            raise ValueError('goal not reachable')
        nodes = [g]
        while nodes[-1] != s:
            nodes.append(int(pred[nodes[-1]]))
        nodes = np.array(nodes[::-1])
        cells = np.stack([nodes % W, nodes // W], axis=1)
        cum = dist[nodes] - dist[s]
        if not self.p.smooth or len(cells) <= 2:
            return cells, float(cum[-1])
        return self._shortcut(cells, cum)

    def _shortcut(self, cells, cum):
        """Greedy any-angle pass: from each anchor take the farthest vertex whose
        straight segment is traversable and not more expensive than the grid
        sub-path; stop scanning at the first blocked segment (lost LOS)."""
        out, total, i, n = [0], 0.0, 0, len(cells)
        while i < n - 1:
            best, best_cost = i + 1, self.segment_cost(cells[i], cells[i + 1])
            if best_cost is None:          # knight edge rasterised differently
                best_cost = cum[i + 1] - cum[i]
            for j in range(i + 2, n):
                c = self.segment_cost(cells[i], cells[j])
                if c is None:
                    break
                if c <= cum[j] - cum[i] + 1e-9:
                    best, best_cost = j, c
            out.append(best)
            total += best_cost
            i = best
        return cells[out], float(total)


def plan_xy(planner: GridPlanner, grid, start_xy, goal_xy):
    """World polyline from start to goal (exact end points) and its plan cost."""
    sc = np.array(world_to_cell(grid, *start_xy), int)
    gc = np.array(world_to_cell(grid, *goal_xy), int)
    cells, cost = planner.plan_cells(sc, gc)
    x, y = cell_to_world(grid, cells[:, 0], cells[:, 1])
    xy = np.stack([x, y], axis=1)
    xy[0], xy[-1] = start_xy, goal_xy
    return xy, cost


def polyline_length(xy):
    xy = np.asarray(xy, float)
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()) if len(xy) > 1 else 0.0


# ---------------------------------------------------------------- headings ---

def baseline_yaws(xy, step, start_yaw, goal_yaw):
    """Resampled path, heading = direction of travel, start/goal pinned."""
    pts = resample_polyline(xy, step)
    yaw = path_headings(pts).astype(float)
    yaw[0] = float(wrap_angle(start_yaw))
    yaw[-1] = float(wrap_angle(goal_yaw))
    return pts, yaw


def dp_yaws(xy, scorer, yaw_params, start_yaw, goal_yaw):
    plan = plan_yaws(xy, scorer, yaw_params, start_yaw=start_yaw, goal_yaw=goal_yaw)
    return plan.xy, plan.yaw


# ---------------------------------------------------------------- metrics ---

def point_weights(ds):
    """Arc length owned by each point: half of each adjacent segment."""
    ds = np.asarray(ds, float)
    w = np.zeros(len(ds) + 1)
    w[:-1] += 0.5 * ds
    w[1:] += 0.5 * ds
    return w


def longest_run(weights, bad):
    """Largest sum of `weights` over a contiguous run where `bad` is True."""
    best = cur = 0.0
    for w, b in zip(weights, bad):
        cur = cur + w if b else 0.0
        best = max(best, cur)
    return float(best)


def pose_metrics(legs, scorer, v=V_LIN, wz=WZ_MAX, good=GOOD):
    """Metrics over a list of legs [(xy (N, 2), yaw (N,), length_m)].

    Fractions are per arc length (point-centred weights); the longest stretch
    without a marker = longest arc with n_eff < good, carried across leg ends;
    max |dyaw|/ds excludes the first and last segment of each leg (pinned start
    / goal headings); time = sum max(ds / v, |dyaw| / wz) per segment."""
    w_all, s_all, n_all, rates = [], [], [], []
    turning = time = length = 0.0
    for xy, yaw, plen in legs:
        ds = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        dyaw = np.abs(wrap_angle(np.diff(yaw)))
        comp = scorer.components(xy[:, 0], xy[:, 1], yaw)
        w_all.append(point_weights(ds))
        s_all.append(comp['s'])
        n_all.append(comp['n_eff'])
        if len(ds) > 2:
            rates.append(dyaw[1:-1] / np.maximum(ds[1:-1], 1e-9))
        turning += float(dyaw.sum())
        time += float(np.maximum(ds / v, dyaw / wz).sum())
        length += float(plen)
    w = np.concatenate(w_all)
    s = np.concatenate(s_all)
    n_eff = np.concatenate(n_all)
    tot = max(float(w.sum()), 1e-9)
    return {
        'length_m': length,
        'frac_s_ge_0.5': float(np.sum(w * (s >= good)) / tot),
        'frac_neff_ge_0.5': float(np.sum(w * (n_eff >= good)) / tot),
        'mean_s': float(np.sum(w * s) / tot),
        'longest_gap_m': longest_run(w, n_eff < good),
        'max_rate_rad_per_m': float(np.concatenate(rates).max()) if rates else 0.0,
        'turning_rad': turning,
        'time_s': time,
    }


# ---------------------------------------------------------------- scenario ---

def scenario_legs(scenario=SCENARIO):
    """[(start_xy, start_yaw, goal_xy, goal_yaw)] per leg."""
    poses = [scenario['start']] + list(scenario['goals'])
    return [((a[0], a[1]), a[2], (b[0], b[1]), b[2]) for a, b in zip(poses[:-1], poses[1:])]


def plan_routes(cost, grid, planner_params, legs):
    planner = GridPlanner(cost, planner_params)
    out = []
    for sxy, _, gxy, _ in legs:
        xy, c = plan_xy(planner, grid, sxy, gxy)
        out.append({'xy': xy, 'plan_cost': c, 'length_m': polyline_length(xy)})
    return out


def headings_for(routes, legs, mode, scorer, yaw_params):
    """[(resampled xy, yaw, polyline length)] per leg for mode baseline | dp."""
    res = []
    for r, (_, syaw, _, gyaw) in zip(routes, legs):
        if mode == 'baseline':
            pts, yaw = baseline_yaws(r['xy'], yaw_params.step, syaw, gyaw)
        else:
            pts, yaw = dp_yaws(r['xy'], scorer, yaw_params, syaw, gyaw)
        res.append((pts, yaw, r['length_m']))
    return res


def load_yaw_params(path=CONFIG_FILE):
    with open(path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}
    return YawPlannerParams.from_dict(raw.get('yaw_planner'))


def with_extra(m, base_len):
    m = dict(m)
    m['extra_length_pct'] = 100.0 * (m['length_m'] / base_len - 1.0) if base_len > 0 else 0.0
    return m


def run_prediction(world_params=WorldGridParams(), planner_params=PlannerParams(),
                   yaw_params=None, c_max_main=C_MAX_MAIN, c_max_sweep=C_MAX_SWEEP,
                   w_f_sweep=W_F_SWEEP, w_trav_sweep=W_TRAV_SWEEP, scenario=SCENARIO,
                   probe_leg=PROBE_LEG):
    """Everything the report needs; pure function of its arguments."""
    yaw_params = yaw_params or load_yaw_params()
    scorer = build_scorer()
    grid, occ, master = build_world_costmap(world_params)
    s_star = best_score_grid(scorer, grid, LAYER['yaw_bins'])
    legs = scenario_legs(scenario)

    def layer(c):
        return layer_cost_from_score(s_star, c_max=c, power=LAYER['power'])

    def routes_for(c, pp=planner_params):
        cost = master if c == 0 else combine_layer(master, layer(c))
        return plan_routes(cost, grid, pp, legs)

    def evaluate(routes, mode, yp=yaw_params, base=None):
        h = headings_for(routes, legs, mode, scorer, yp)
        per_leg = [pose_metrics([x], scorer) for x in h]
        total = pose_metrics(h, scorer)
        if base is not None:
            total = with_extra(total, base['total']['length_m'])
            per_leg = [with_extra(m, b['length_m']) for m, b in zip(per_leg, base['per_leg'])]
        return {'total': total, 'per_leg': per_leg, 'headings': h}

    base_routes = routes_for(0)
    out = {'grid': grid, 'occupied': occ, 'master': master, 's_star': s_star,
           'scorer': scorer, 'legs': legs, 'routes': {'baseline': base_routes}, 'variants': {}}
    base = evaluate(base_routes, 'baseline')
    base_ref = {'total': base['total'], 'per_leg': base['per_leg']}
    out['variants']['baseline'] = evaluate(base_routes, 'baseline', base=base_ref)
    out['variants']['yaw-only'] = evaluate(base_routes, 'dp', base=base_ref)
    routes_cache = {}
    for c in sorted(set(c_max_main) | set(c_max_sweep)):
        routes_cache[c] = routes_for(c)
        out['routes'][f'active c_max={c}'] = routes_cache[c]
    for c in c_max_main:
        out['variants'][f'active c_max={c}'] = evaluate(routes_cache[c], 'dp', base=base_ref)
    out['sweep_c_max'] = {c: evaluate(routes_cache[c], 'dp', base=base_ref)['total']
                          for c in c_max_sweep}
    out['sweep_w_f'] = {}
    for wf in w_f_sweep:
        yp = YawPlannerParams(**{**asdict(yaw_params), 'w_forward': wf})
        out['sweep_w_f'][wf] = {
            'yaw-only': evaluate(base_routes, 'dp', yp, base_ref)['total'],
            'active c_max=40': evaluate(routes_cache.get(40) or routes_for(40), 'dp', yp,
                                        base_ref)['total']}
    out['sweep_w_trav'] = {}
    for wt in w_trav_sweep:
        pp = PlannerParams(**{**asdict(planner_params), 'w_traversal_cost': wt})
        b = plan_routes(master, grid, pp, legs)
        a = routes_for(40, pp)
        out['sweep_w_trav'][wt] = {
            'baseline_length_m': sum(r['length_m'] for r in b),
            'active_c40_length_m': sum(r['length_m'] for r in a),
            'active_c40': evaluate(a, 'dp', base={'total': {'length_m': sum(
                r['length_m'] for r in b)}, 'per_leg': [{'length_m': r['length_m']}
                                                        for r in b]})['total']}
    out['probe'] = {}
    if probe_leg is not None:
        pl = [probe_leg]
        pb = plan_routes(master, grid, planner_params, pl)
        pref = {'total': {'length_m': pb[0]['length_m']}, 'per_leg': [{'length_m': 0.0}]}
        for c in (0,) + tuple(c_max_sweep):
            r = pb if c == 0 else plan_routes(combine_layer(master, layer(c)), grid,
                                              planner_params, pl)
            h = headings_for(r, pl, 'dp', scorer, yaw_params)
            m = with_extra(pose_metrics(h, scorer), pref['total']['length_m'])
            m['max_offset_m'] = float(np.max(np.abs(r[0]['xy'][:, 1] - probe_leg[0][1])))
            out['probe'][c] = m
    out['params'] = {'world': asdict(world_params), 'planner': asdict(planner_params),
                     'yaw_planner': asdict(yaw_params), 'layer': dict(LAYER),
                     'v_lin': V_LIN, 'wz_max': WZ_MAX, 'good': GOOD,
                     'grid': {'origin_x': grid.origin_x, 'origin_y': grid.origin_y,
                              'resolution': grid.resolution, 'width': grid.width,
                              'height': grid.height},
                     'landmarks_file': LANDMARKS_REL, 'scenario': scenario,
                     'probe_leg': probe_leg}
    return out


# ---------------------------------------------------------------- output ---

COLS = (('length_m', 'L [m]', '{:.2f}'), ('extra_length_pct', 'ΔL vs base [%]', '{:+.1f}'),
        ('frac_s_ge_0.5', 's≥0.5 [frac L]', '{:.3f}'),
        ('frac_neff_ge_0.5', 'n_eff≥0.5 [frac L]', '{:.3f}'),
        ('mean_s', 'mean s', '{:.3f}'),
        ('longest_gap_m', 'longest gap [m]', '{:.2f}'),
        ('max_rate_rad_per_m', 'max dψ/ds [rad/m]', '{:.2f}'),
        ('turning_rad', 'turning [rad]', '{:.2f}'), ('time_s', 'time [s]', '{:.1f}'))


def _row(label, m):
    return '| ' + label + ' | ' + ' | '.join(f.format(m[k]) for k, _, f in COLS) + ' |'


def _head(first):
    return ('| ' + first + ' | ' + ' | '.join(h for _, h, _ in COLS) + ' |\n|'
            + '---|' * (len(COLS) + 1))


def layer_breakeven(c_max, w_trav=0.5, w_euc=1.0):
    """Extra traversal cost per cell of a cell at c_max vs a free cell, relative
    to w_euc: a route through the band is preferred only if its detour is below
    this fraction of the distance it spends in the band (straight runs)."""
    return float((traversal_cost(c_max, w_trav) - traversal_cost(0, w_trav)) / w_euc)


def assumptions_md(res):
    p = res['params']
    w = p['world']
    return '\n'.join([
        '- [assumption] Static map = world geometry (FACTS 18.10) rasterised at '
        f'{w["resolution"]} m, map ≈ world; the real global costmap comes from the RTAB-Map '
        'grid and has obstacle/denoise layers (ignored here).',
        f'- [assumption] Inflation = Nav2 InflationLayer formula with inscribed radius '
        f'{w["inscribed_radius"]} m ("robot radius"; Nav2 itself derives 0.325 m from the '
        f'footprint), inflation_radius {w["inflation_radius"]} m, cost_scaling_factor '
        f'{w["cost_scaling_factor"]}.',
        '- [assumption] Planner = Dijkstra on a 16-connected grid with the Theta* cost model '
        '(w_euc·length + Σ_cells w_trav·(26+0.9c)²/252², c ≤ 251 traversable) + cost-aware '
        'line-of-sight shortcuts; Nav2 Theta* is a heuristic any-angle search and can return a '
        'slightly different (costlier) route. No path smoother.',
        '- [assumption] Layer = spec §3 on s* of the oracle scorer (rule-based shelf-row '
        'occlusion), combined as max(master, layer) on cells < 253.',
        f'- [assumption] Robot tracks the planned path exactly at constant v = {p["v_lin"]} m/s; '
        f'heading changes limited only by wz = {p["wz_max"]} rad/s (omni base); segment time = '
        'max(ds/v, |Δψ|/wz). Baseline heading = direction of travel (MPPI with path-angle / '
        'prefer-forward critics is approximated by this).',
        '- [assumption] Start/goal headings pinned in all variants (baseline turns to the goal '
        'yaw on its last 0.2 m segment); max dψ/ds (= max |Δψ|/ds over 0.2 m segments) '
        'excludes those pinned segments.',
        '- "gap" = longest arc length with n_eff < 0.5 (carried across leg ends).',
    ])


def write_markdown(res, path, figs):
    p = res['params']
    yp = p['yaw_planner']
    pp = p['planner']
    g = p['grid']
    sc = p['scenario']
    lines = ['# T-011b · Offline prediction of the active-localization effect', '',
             f'Scenario `{sc["name"]}`: start {fmt_pose(sc["start"])} → '
             + ' → '.join(fmt_pose(q) for q in sc['goals']) + '.',
             f'Grid {g["width"]}×{g["height"]} at {g["resolution"]} m, origin '
             f'({g["origin_x"]:.2f}, {g["origin_y"]:.2f}); landmarks `{p["landmarks_file"]}` '
             '(ids 1–5); scorer S2* (spec §2 defaults); yaw DP spec §4 (config: step '
             f'{yp["step"]}, w_s {yp["w_score"]}, w_f {yp["w_forward"]}, w_sm {yp["w_smooth"]}, '
             f'k {yp["max_dyaw_per_m"]} rad/m, w_lim {yp["w_limit"]}); Theta* w_euc '
             f'{pp["w_euc_cost"]}, w_trav {pp["w_traversal_cost"]}.',
             'Generated by `navigation/src/active_localization/tools/predict_effect.py` '
             '(deterministic).', '',
             '## Approximations', '', assumptions_md(res), '',
             '## Prediction (whole scenario, 3 legs)', '', _head('variant')]
    for name, v in res['variants'].items():
        lines.append(_row(name, v['total']))
    lines += ['', 'Yaw-only uses the baseline route; active = layer c_max + yaw DP.', '',
              '## Per leg', '']
    for i, leg in enumerate(res['legs']):
        lines += [f'### Leg {i + 1}: {fmt_pose((*leg[0], leg[1]))} → '
                  f'{fmt_pose((*leg[2], leg[3]))}', '', _head('variant')]
        for name, v in res['variants'].items():
            lines.append(_row(name, v['per_leg'][i]))
        lines.append('')
    lines += ['## Sensitivity', '', '### c_max sweep (layer + yaw DP, w_f from config)', '',
              _head('c_max')]
    for c, m in res['sweep_c_max'].items():
        lines.append(_row(str(c), m))
    lines += ['', '### w_f sweep (yaw DP forward prior)', '', _head('w_f / variant')]
    for wf, d in res['sweep_w_f'].items():
        for name, m in d.items():
            lines.append(_row(f'{wf} / {name}', m))
    lines += ['', '### Theta* w_traversal_cost (baseline route re-planned with the same w)', '',
              _head('w_trav / active c_max=40')]
    for wt, d in res['sweep_w_trav'].items():
        lines.append(_row(f'{wt} (baseline L {d["baseline_length_m"]:.2f} m)', d['active_c40']))
    if res['probe']:
        q = res['params']['probe_leg']
        lines += ['', f'### Route-lever probe: straight leg {fmt_pose((*q[0], q[1]))} → '
                  f'{fmt_pose((*q[2], q[3]))} (not in the scenario, outside the band)', '',
                  'c_max = 0 is the route without the layer; yaw DP on every route; '
                  '"offset" = max |y − y_start| of the route.', '',
                  _head('c_max').replace(' |\n|', ' | offset [m] |\n|---|', 1)]
        for c, m in res['probe'].items():
            lines.append(_row(str(c), m) + f' {m["max_offset_m"]:.2f} |')
    lines += ['', '### Layer strength vs path length (analytic)', '',
              'Extra Theta* cost per cell of a cell at c_max over a free cell, in units of one '
              'cell of length (w_euc = 1). A route through the band wins only if its detour is '
              'below roughly this fraction of the length it would otherwise spend outside the '
              'band.', '', '| c_max | w_trav 0.5 | w_trav 1.0 | w_trav 2.0 |', '|---|---|---|---|']
    for c in C_MAX_SWEEP:
        cells = [f'{100 * layer_breakeven(c, wt):.1f} %' for wt in W_TRAV_SWEEP]
        lines.append(f'| {c} | ' + ' | '.join(cells) + ' |')
    lines += ['', '## Figures', ''] + [f'- `{os.path.basename(f)}`' for f in figs] + ['']
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))


def fmt_pose(q):
    return f'({q[0]:.2f}, {q[1]:.2f}, {math.degrees(q[2]):.0f}°)'


def to_json(res):
    def clean(v):
        if isinstance(v, dict):
            return {str(k): clean(x) for k, x in v.items() if k != 'headings'}
        if isinstance(v, (list, tuple)):
            return [clean(x) for x in v]
        if isinstance(v, np.ndarray):
            return v.tolist()
        if isinstance(v, (np.floating, np.integer)):
            return v.item()
        return v
    routes = {k: [{'xy': r['xy'].round(4).tolist(), 'length_m': r['length_m'],
                   'plan_cost': r['plan_cost']} for r in v] for k, v in res['routes'].items()}
    return clean({'params': res['params'], 'variants': res['variants'],
                  'sweep_c_max': res['sweep_c_max'], 'sweep_w_f': res['sweep_w_f'],
                  'sweep_w_trav': res['sweep_w_trav'], 'probe': res['probe'],
                  'routes': routes})


def plot(res, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.patches as mpatches
    import matplotlib.pyplot as plt
    grid, scorer = res['grid'], res['scorer']
    ext = (grid.origin_x, grid.origin_x + grid.width * grid.resolution,
           grid.origin_y, grid.origin_y + grid.height * grid.resolution)
    blocked = np.ma.masked_where(res['master'] < 253, np.ones(res['master'].shape))

    def scene(ax):
        ax.imshow(blocked, origin='lower', extent=ext, cmap='Greys', vmin=0, vmax=3,
                  alpha=0.5, zorder=2)
        for x0, x1, y0, y1 in SHELF_BOXES:
            ax.add_patch(mpatches.Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor='#bdbdbd',
                                            edgecolor='#444444', lw=0.8, zorder=3))
        xmin, xmax, ymin, ymax = INNER_BOX
        ax.add_patch(mpatches.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, fill=False,
                                        edgecolor='#222222', lw=1.5, zorder=3))
        for lid, p in zip(scorer.lm.ids, scorer.lm.xy):
            ax.plot(p[0], p[1], 's', ms=6, mfc='#111111', mec='white', zorder=6)
            ax.text(p[0] + 0.06, p[1] + 0.1, f'id {lid}', fontsize=8, zorder=6)
        for q in [res['params']['scenario']['start']] + list(res['params']['scenario']['goals']):
            ax.plot(q[0], q[1], '*', ms=11, color='#d62728', mec='white', zorder=7)
        ax.set_xlim(ext[0], ext[1])
        ax.set_ylim(ext[2], ext[3])
        ax.set_aspect('equal')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')

    def headings(ax, h, color):
        for pts, yaw, _ in h:
            ax.plot(pts[:, 0], pts[:, 1], '-', color=color, lw=1.6, zorder=4)
            comp = scorer.components(pts[:, 0], pts[:, 1], yaw)
            k = slice(0, None, 3)
            good = comp['n_eff'][k] >= GOOD
            for m, col in ((good, '#1a9850'), (~good, '#d73027')):
                ax.quiver(pts[k, 0][m], pts[k, 1][m], np.cos(yaw[k][m]), np.sin(yaw[k][m]),
                          color=col, scale=28, width=0.004, zorder=5)

    figs = []
    wanted = ('baseline', 'yaw-only', 'active c_max=40', 'active c_max=80')
    panels = [n for n in res['variants'] if n in wanted]
    fig, axes = plt.subplots(2, 2, figsize=(13, 11))
    for ax, name in zip(axes.flat, panels):
        im = ax.imshow(res['s_star'], origin='lower', extent=ext, cmap='Blues', vmin=0,
                       vmax=0.7, zorder=1)
        scene(ax)
        headings(ax, res['variants'][name]['headings'], '#ff7f0e')
        t = res['variants'][name]['total']
        ax.set_title(f'{name}: L {t["length_m"]:.1f} m, n_eff≥0.5 on '
                     f'{100 * t["frac_neff_ge_0.5"]:.0f} % of L, gap {t["longest_gap_m"]:.1f} m',
                     fontsize=10)
    fig.colorbar(im, ax=axes, shrink=0.6, label='s* (best over yaw)')
    fig.suptitle('shelves_loop_a: path (orange) and heading (arrow every 0.6 m; green = '
                 'n_eff ≥ 0.5, red = < 0.5)')
    figs.append(os.path.join(out, 'predict_paths.png'))
    fig.savefig(figs[-1], dpi=110, bbox_inches='tight')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 9))
    lay = layer_cost_from_score(res['s_star'], c_max=LAYER['c_max'])
    im = ax.imshow(lay, origin='lower', extent=ext, cmap='Oranges', vmin=0,
                   vmax=LAYER['c_max'], zorder=1)
    scene(ax)
    names = ['baseline'] + [k for k in res['routes'] if k.startswith('active')]
    cmap = plt.get_cmap('viridis')
    for i, name in enumerate(names):
        col = '#000000' if name == 'baseline' else cmap(i / max(len(names) - 1, 1))
        for j, r in enumerate(res['routes'][name]):
            ax.plot(r['xy'][:, 0], r['xy'][:, 1], color=col, lw=2.2 if j == 0 else 1.6,
                    ls='--' if name == 'baseline' else '-', zorder=4,
                    label=name if j == 0 else None)
    ax.legend(loc='upper left', fontsize=8)
    fig.colorbar(im, ax=ax, shrink=0.8, label=f'layer cost (c_max = {LAYER["c_max"]})')
    ax.set_title('Routes for the c_max sweep over the spec §3 layer cost (c_max = 40 shown)')
    figs.append(os.path.join(out, 'predict_cmax_sweep.png'))
    fig.savefig(figs[-1], dpi=110, bbox_inches='tight')
    plt.close(fig)
    return figs


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', required=True, help='output directory')
    ap.add_argument('--inscribed', type=float, default=WorldGridParams.inscribed_radius,
                    help='inscribed radius [m] of the inflation model')
    ap.add_argument('--no-plots', action='store_true')
    a = ap.parse_args(argv)
    res = run_prediction(WorldGridParams(inscribed_radius=a.inscribed))
    os.makedirs(a.out, exist_ok=True)
    figs = [] if a.no_plots else plot(res, a.out)
    md = os.path.join(a.out, 'prediction.md')
    write_markdown(res, md, figs)
    with open(os.path.join(a.out, 'prediction.json'), 'w', encoding='utf-8') as f:
        json.dump(to_json(res), f, indent=1)
    print(_head('variant'))
    for name, v in res['variants'].items():
        print(_row(name, v['total']))
    for p in figs + [md, os.path.join(a.out, 'prediction.json')]:
        print('wrote', p)
    return 0


if __name__ == '__main__':
    sys.exit(main())
