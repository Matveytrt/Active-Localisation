#!/usr/bin/env python3
"""Golden fixtures for the C++ gtests (T-009b/c), produced by the Python oracle.

    python3 -I tools/make_fixtures.py [--out test/fixtures]

Files (each has a "header" object with every parameter the C++ side needs):
  scorer_cases.json      S2* (spec §2) at ~50 poses: per-marker terms, n_eff, s
  layer_cost_cases.json  spec §3 layer cost at ~50 cells of the room grid
  yaw_dp_case.json       spec §4 Viterbi DP on one fixed polyline

Deterministic: fixed poses / fixed RNG seed; test_fixtures.py regenerates and
compares with the committed files.
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from _common import (LAYER, PKG_DIR, ROOM, build_scorer, room_grid,  # noqa: E402
                     scorer_header)
from active_localization.cost_field import (best_score_grid,  # noqa: E402
                                            layer_cost_from_score)
from active_localization.geometry import wrap_angle  # noqa: E402
from active_localization.yaw_planner import YawPlannerParams, plan_yaws  # noqa: E402

DEFAULT_OUT = os.path.join(PKG_DIR, 'test', 'fixtures')
SEED = 20261018
HALF_PI = math.pi / 2

# Spec §4 DP parameters (YawPlannerParams field names; spec names in comments).
DP_PARAMS = {'n_bins': 36, 'step': 0.20, 'w_score': 1.0,   # B, step, w_s
             'w_forward': 0.3, 'w_smooth': 0.5,              # w_f, w_sm
             'w_limit': 100.0, 'max_dyaw_per_m': 1.0,        # w_lim, k
             'pin_start': True, 'pin_goal': True}
DP_POLYLINE = [[-2.0, -0.5], [2.6, -0.5], [2.6, 4.6]]   # south corridor, then up the east side
DP_START_YAW = 0.0
DP_GOAL_YAW = HALF_PI


def _f(v):
    return float(v)


def _facing_pose(p, r, n=(0.0, -1.0), off=0.07, yaw_delta=0.0):
    """Base pose whose camera is r m out along normal n of marker p, looking at it."""
    cx, cy = p[0] + r * n[0], p[1] + r * n[1]
    yaw = math.atan2(p[1] - cy, p[0] - cx)
    return cx - off * math.cos(yaw), cy - off * math.sin(yaw), yaw + yaw_delta


def _oblique_pose(p, r, theta, off=0.07):
    """Camera r m from marker p at incidence theta (normal (0,-1)), looking at it."""
    cx, cy = p[0] + r * math.sin(theta), p[1] - r * math.cos(theta)
    yaw = math.atan2(p[1] - cy, p[0] - cx)
    return cx - off * math.cos(yaw), cy - off * math.sin(yaw), yaw


def scorer_poses(scorer):
    xy = {int(i): tuple(map(float, p)) for i, p in zip(scorer.lm.ids, scorer.lm.xy)}
    hfov2 = 0.5 * scorer.cam.hfov
    named = []
    for r in (1.0, 1.88, 0.6, 0.7, 1.2, 1.5, 2.2, 3.0):
        named.append((f'id1 straight on, camera r={r}', _facing_pose(xy[1], r)))
    named += [
        ('id1 r=1.0 turned +90 deg', _facing_pose(xy[1], 1.0, yaw_delta=HALF_PI)),
        ('id1 r=1.0 turned -90 deg', _facing_pose(xy[1], 1.0, yaw_delta=-HALF_PI)),
        ('id1 r=1.0 turned 180 deg', _facing_pose(xy[1], 1.0, yaw_delta=math.pi)),
        ('id1 r=1.0 base yaw rotated by -HFOV/2', _facing_pose(xy[1], 1.0, yaw_delta=-hfov2)),
        ('id1 r=1.0 base yaw rotated by -HFOV/2 - 0.05',
         _facing_pose(xy[1], 1.0, yaw_delta=-hfov2 - .05)),
        ('id1 incidence 60 deg r=1.6', _oblique_pose(xy[1], 1.6, math.radians(60))),
        ('id1 incidence 60 deg r=1.2', _oblique_pose(xy[1], 1.2, math.radians(60))),
        ('id1 incidence -45 deg r=1.0', _oblique_pose(xy[1], 1.0, math.radians(-45))),
        ('id1 incidence 85 deg r=1.0', _oblique_pose(xy[1], 1.0, math.radians(85))),
        ('id1 from behind (+y side), r=1.0', _facing_pose(xy[1], 1.0, n=(0.0, 1.0))),
        ('id2 straight on r=1.0', _facing_pose(xy[2], 1.0)),
        ('id3 straight on r=1.0 (between rows)', _facing_pose(xy[3], 1.0)),
        ('id4 straight on r=1.4 (between rows)', _facing_pose(xy[4], 1.4)),
        ('id5 straight on r=0.9', _facing_pose(xy[5], 0.9)),
        ('id1 and id2 both in view', (1.05, -0.9, HALF_PI)),
        ('id4 from the south corridor: occluded by row 1', _facing_pose(xy[4], 2.9)),
        ('id4 from just south of row 1: occluded', _facing_pose(xy[4], 2.6)),
        ('east of the rows, yaw = pi (markers outside FoV)', (2.9, 2.2, math.pi)),
        ('yaw = +pi exactly', (0.2, -0.2, math.pi)),
        ('yaw = -pi exactly', (0.2, -0.2, -math.pi)),
        ('yaw = 7.0 (unwrapped input)', (0.2, -0.2, 7.0)),
        ('camera exactly at marker id1 (r = 0 guard)', (xy[1][0] - 0.07, xy[1][1], 0.0)),
        ('base inside row 1 rectangle', (-1.0, 1.2, -HALF_PI)),
        ('far corner (nothing visible)', (-4.5, 5.0, 0.0)),
    ]
    rng = np.random.default_rng(SEED)
    k = 0
    while len(named) < 52:
        x = rng.uniform(-3.5, 3.1)
        y = rng.uniform(-1.8, 3.3)
        yaw = rng.uniform(-math.pi, math.pi)
        named.append((f'random #{k}', (x, y, yaw)))
        k += 1
    return named


def build_scorer_cases(scorer):
    cases = []
    for name, (x, y, yaw) in scorer_poses(scorer):
        c = scorer.components(x, y, yaw)
        cx, cy, _ = scorer.camera_pose(x, y, yaw)
        markers = []
        for j, lid in enumerate(scorer.lm.ids):
            markers.append({'id': int(lid), 'r': _f(c['r'][j]), 'beta': _f(c['beta'][j]),
                            'cos_theta': _f(c['cos_theta'][j]), 'area': _f(c['area'][j]),
                            'g_fov': _f(c['g_fov'][j]), 'g_area': _f(c['g_area'][j]),
                            'g_near': _f(c['g_near'][j]), 'occ': _f(c['occ'][j]),
                            'w': _f(c['w'][j])})
        cases.append({'name': name, 'x': _f(x), 'y': _f(y), 'yaw': _f(yaw),
                      'camera_x': _f(cx), 'camera_y': _f(cy),
                      'markers': markers, 'n_eff': _f(c['n_eff']), 's': _f(c['s'])})
    header = scorer_header(scorer)
    header.update({'what': 'S2* scorer (spec T-009 §2) at fixed base_link poses',
                   'tolerance_s': 1e-6,
                   'notes': 'r is guarded as max(r, 1e-9) in the divisions; atan2(0,0) = 0. '
                            'yaw inputs are not pre-wrapped. occ comes from the oracle '
                            'rectangle rule: compare w/occ directly or feed occ to the C++ '
                            'scorer when it uses the static-map ray cast.'})
    return {'header': header, 'cases': cases}


def build_layer_cases(scorer):
    grid = room_grid()
    X, Y = grid.cell_centers()
    best = best_score_grid(scorer, grid, LAYER['yaw_bins'])
    cost = layer_cost_from_score(best, c_max=LAYER['c_max'], power=LAYER['power'])
    rng = np.random.default_rng(SEED + 1)
    picks = []
    # stratified: 12 cells per cost class (0, 1..39, 40) + band-edge cells
    for mask in (cost == 0, (cost > 0) & (cost < LAYER['c_max']), cost == LAYER['c_max']):
        rows, cols = np.nonzero(mask)
        sel = rng.choice(len(rows), size=min(14, len(rows)), replace=False)
        picks += [(int(rows[s]), int(cols[s])) for s in sorted(sel)]
    # straight line through the band in front of id 1 (every 3rd cell from the face)
    col1 = int(np.argmin(np.abs(X[0] - scorer.lm.xy[0, 0])))
    for row in range(int((scorer.lm.xy[0, 1] - grid.origin_y) / grid.resolution) - 1, 0, -3):
        picks.append((row, col1))
    seen, cells = set(), []
    for row, col in picks:
        if (row, col) in seen:
            continue
        seen.add((row, col))
        cells.append({'col': col, 'row': row, 'x': _f(X[row, col]), 'y': _f(Y[row, col]),
                      's_star': _f(best[row, col]), 'cost': int(cost[row, col])})
    header = scorer_header(scorer)
    header.update({
        'what': 'LocalizabilityLayer cost (spec T-009 §3) at cells of the room grid',
        'formula': 'cost = round(c_max * (1 - min(s_star / s_ref, 1))^power); round = '
                   'floor(v + 0.5); s_star = max over yaw_k = -pi + 2 pi k / yaw_bins of s',
        'scorer_formula': header['formula'],
        'c_max': LAYER['c_max'], 'power': LAYER['power'], 'yaw_bins': LAYER['yaw_bins'],
        'grid': {'origin_x': grid.origin_x, 'origin_y': grid.origin_y,
                 'resolution': grid.resolution, 'width': grid.width, 'height': grid.height,
                 'cell_center': 'x = origin_x + (col + 0.5) res, y = origin_y + (row + 0.5) res',
                 'room': dict(ROOM)},
        'tolerance_cost': 1, 'tolerance_s_star': 1e-6,
        'notes': 'every cell evaluated (no distance cut-off). Oracle check on this grid: '
                 'cost < 40 up to 2.80 m and cost < 39 up to 2.48 m from the nearest marker '
                 '(base position), so a cut-off of r_max + 0.5 m (~2.39 m) writing c_max '
                 'differs by up to 2; use >= 2.5 m for +-1, >= 2.8 m (r_max + 1 m) for exact',
    })
    return {'header': header, 'cells': cells}


def build_dp_case(scorer):
    params = YawPlannerParams.from_dict(DP_PARAMS)
    plan = plan_yaws(np.array(DP_POLYLINE), scorer, params,
                     start_yaw=DP_START_YAW, goal_yaw=DP_GOAL_YAW)
    n, b = len(plan.xy), params.n_bins
    bins = -math.pi + 2.0 * math.pi * np.arange(b) / b
    s_all = scorer.score(plan.xy[:, 0:1], plan.xy[:, 1:2], bins[None, :])
    node = (-params.w_score * s_all / scorer.score_scale
            + params.w_forward * (1.0 - np.cos(bins[None, :] - plan.travel[:, None])))
    node[0] += 1e6 * (1.0 - np.cos(bins - DP_START_YAW))
    node[-1] += 1e6 * (1.0 - np.cos(bins - DP_GOAL_YAW))
    ds = np.linalg.norm(np.diff(plan.xy, axis=0), axis=1)
    idx = plan.bin_idx
    total = float(node[np.arange(n), idx].sum())
    for i in range(1, n):
        d = abs(float(wrap_angle(bins[idx[i]] - bins[idx[i - 1]])))
        total += params.w_smooth * d * d \
            + params.w_limit * max(0.0, d - params.max_dyaw_per_m * ds[i - 1]) ** 2
    header = scorer_header(scorer)
    header.update({
        'what': 'Viterbi yaw DP (spec T-009 §4) on a fixed polyline',
        'scorer_formula': header.pop('formula'),
        'dp': dict(DP_PARAMS),
        'dp_spec_names': {'n_bins': 'B', 'w_score': 'w_s', 'w_forward': 'w_f',
                          'w_smooth': 'w_sm', 'w_limit': 'w_lim', 'max_dyaw_per_m': 'k'},
        'node': 'node(j,b) = -w_score * s(x_j, y_j, psi_b) / s_ref '
                '+ w_forward * (1 - cos(psi_b - phi_j)); pins add 1e6 * (1 - cos(psi_b - '
                'yaw_pin)) at j = 0 (start_yaw) and j = N-1 (goal_yaw)',
        'edge': 'edge(j, a->b) = w_smooth * d^2 + w_limit * max(0, |d| - max_dyaw_per_m * '
                'ds_{j-1})^2, d = wrap(psi_b - psi_a)',
        'bins': 'psi_b = -pi + 2 pi b / n_bins',
        'travel': 'phi_j = atan2 of (p_{j+1} - p_j); the last point repeats phi_{N-2}',
        'resampling': 'n_seg = ceil(L / step); points at arc length linspace(0, L, n_seg + 1) '
                      '(first/last exact)',
        'ties': 'argmin takes the lowest index (numpy)',
        'output_yaw': 'bins[bin_idx], except yaw[0] = start_yaw, yaw[N-1] = goal_yaw (exact)',
        'start_yaw': DP_START_YAW, 'goal_yaw': DP_GOAL_YAW,
    })
    return {'header': header,
            'polyline': DP_POLYLINE,
            'points': plan.xy.tolist(),
            'ds': ds.tolist(),
            'travel': plan.travel.tolist(),
            'bin_idx': [int(i) for i in idx],
            'yaw': plan.yaw.tolist(),
            'score_at_chosen_bin': plan.score.tolist(),
            'total_cost': total}


def build_all():
    scorer = build_scorer()
    return {'scorer_cases.json': build_scorer_cases(scorer),
            'layer_cost_cases.json': build_layer_cases(scorer),
            'yaw_dp_case.json': build_dp_case(scorer)}


def dumps(obj):
    return json.dumps(obj, indent=1, sort_keys=False, ensure_ascii=True) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=DEFAULT_OUT)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    for name, obj in build_all().items():
        path = os.path.join(a.out, name)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(dumps(obj))
        n = len(obj.get('cases') or obj.get('cells') or obj.get('points'))
        print(f'wrote {path} ({n} entries)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
