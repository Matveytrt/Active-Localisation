"""T-011b offline prediction: world grid builder, Theta*-like planner, metrics.

Pure Python (numpy/scipy), no ROS. The tools are scripts, so their directory
is put on sys.path here.
"""

import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(os.path.dirname(HERE), 'tools')
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import _world_grid as wg  # noqa: E402
import predict_effect as pe  # noqa: E402
from active_localization.cost_field import GridSpec  # noqa: E402


# ------------------------------------------------------------ world grid ---

def _cost_at(grid, cost, x, y):
    ix, iy = wg.world_to_cell(grid, x, y)
    return int(cost[iy, ix])


def test_world_grid_covers_room_with_walls_and_shelves():
    grid, occ, cost = wg.build_world_costmap()
    assert (grid.width, grid.height) == (169, 153)
    assert grid.origin_x <= wg.INNER_BOX[0] - 0.05 and grid.origin_y <= wg.INNER_BOX[2] - 0.05
    assert occ.shape == cost.shape == (grid.height, grid.width) and cost.dtype == np.uint8
    assert _cost_at(grid, cost, 0.0, 0.0) == 0                     # open floor
    assert _cost_at(grid, cost, -4.95, 0.0) == wg.LETHAL           # beyond the west wall
    assert _cost_at(grid, cost, 3.40, 0.0) == wg.LETHAL            # beyond the east wall
    assert _cost_at(grid, cost, -2.165, 1.2) == wg.LETHAL          # shelf box centre
    # 0.38 m gap between two boxes of a row: free cell, but inside the inscribed radius
    gap = (-1.505 + -1.125) / 2
    ix, iy = wg.world_to_cell(grid, gap, 1.2)
    assert not occ[iy, ix] and cost[iy, ix] == wg.INSCRIBED
    for x, y, _ in [pe.SCENARIO['start']] + pe.SCENARIO['goals']:
        assert pe.traversable(_cost_at(grid, cost, x, y))


def test_inflation_matches_nav2_formula():
    occ = np.zeros((1, 40), bool)
    occ[0, 0] = True
    c = wg.inflate(occ, 0.05, inscribed_radius=0.40, inflation_radius=0.50,
                   cost_scaling_factor=3.0)[0]
    assert c[0] == wg.LETHAL
    assert np.all(c[1:9] == wg.INSCRIBED)                          # d = 0.05 .. 0.40 m
    assert c[9] == math.floor(252 * math.exp(-3.0 * 0.05))         # d = 0.45 m
    assert c[10] == math.floor(252 * math.exp(-3.0 * 0.10))        # d = 0.50 m
    assert np.all(c[11:] == 0)
    assert np.all(wg.inflate(np.zeros((3, 3), bool), 0.05) == 0)


def test_combine_layer_keeps_inscribed_and_lethal():
    master = np.array([0, 100, 253, 254], np.uint8)
    out = wg.combine_layer(master, np.array([40, 40, 40, 40]))
    assert out.tolist() == [40, 100, 253, 254]
    assert wg.combine_layer(np.zeros(1, np.uint8), np.array([300])).tolist() == [252]


# --------------------------------------------------------------- planner ---

def test_theta_star_cost_model():
    assert pe.traversal_cost(0, 0.5) == pytest.approx(0.5 * 26 ** 2 / 252 ** 2)
    assert pe.traversal_cost(252, 1.0) == pytest.approx((26 + 0.9 * 252) ** 2 / 252 ** 2)
    assert pe.traversable(251) and not pe.traversable(252) and not pe.traversable(253)
    assert pe.layer_breakeven(40) == pytest.approx(
        0.5 * ((26 + 36) ** 2 - 26 ** 2) / 252 ** 2)


def _toy_grid(n=60, res=0.1):
    return GridSpec(origin_x=0.0, origin_y=0.0, resolution=res, width=n, height=n)


def test_free_space_is_one_straight_segment():
    grid = _toy_grid()
    planner = pe.GridPlanner(np.zeros((grid.height, grid.width), np.uint8))
    xy, cost = pe.plan_xy(planner, grid, (0.52, 0.31), (5.07, 2.88))
    assert len(xy) == 2
    assert pe.polyline_length(xy) == pytest.approx(math.hypot(5.07 - 0.52, 2.88 - 0.31))
    assert cost > 0


def _wall_world():
    grid = _toy_grid()
    occ = np.zeros((grid.height, grid.width), bool)
    occ[:45, 30] = True                                  # wall x in [3.0, 3.1], y < 4.5
    cost = wg.inflate(occ, grid.resolution, inscribed_radius=0.2, inflation_radius=0.3)
    return grid, cost


def _assert_traversable(grid, cost, xy):
    pts = np.concatenate([np.linspace(a, b, 50) for a, b in zip(xy[:-1], xy[1:])])
    ix, iy = wg.world_to_cell(grid, pts[:, 0], pts[:, 1])
    assert np.all(pe.traversable(cost[iy, ix]))


def test_detour_around_wall_is_any_angle_and_near_shortest():
    grid, cost = _wall_world()
    planner = pe.GridPlanner(cost)
    s, g = (1.05, 1.05), (5.05, 1.05)
    xy, _ = pe.plan_xy(planner, grid, s, g)
    _assert_traversable(grid, cost, xy)
    # lower bound: must cross the wall's column above the last blocked cell
    blocked_rows = np.where(~pe.traversable(cost[:, 30]))[0]
    y_top = grid.origin_y + (blocked_rows.max() + 1) * grid.resolution
    tip = (3.05, y_top)
    lb = math.dist(s, tip) + math.dist(tip, g)
    length = pe.polyline_length(xy)
    assert lb - 1e-9 <= length <= 1.04 * lb
    assert len(xy) <= 10                       # few vertices (rounding the inflated tip)
    raw = pe.GridPlanner(cost, pe.PlannerParams(smooth=False))
    xy_raw, _ = pe.plan_xy(raw, grid, s, g)
    assert length <= pe.polyline_length(xy_raw) + 1e-9


def test_planner_is_deterministic():
    grid, cost = _wall_world()
    a = pe.plan_xy(pe.GridPlanner(cost), grid, (1.05, 1.05), (5.05, 1.05))
    b = pe.plan_xy(pe.GridPlanner(cost), grid, (1.05, 1.05), (5.05, 1.05))
    assert np.array_equal(a[0], b[0]) and a[1] == b[1]


def test_unreachable_or_blocked_raises():
    grid, cost = _wall_world()
    planner = pe.GridPlanner(cost)
    with pytest.raises(ValueError):
        pe.plan_xy(planner, grid, (3.05, 1.0), (5.0, 1.0))      # start on the wall
    closed = cost.copy()
    closed[:, 30] = wg.LETHAL
    with pytest.raises(ValueError):
        pe.plan_xy(pe.GridPlanner(closed), grid, (1.05, 1.05), (5.05, 1.05))


def test_layer_bends_route_into_cheap_band_only_when_strong_enough():
    grid = _toy_grid()
    band = np.zeros((grid.height, grid.width), bool)
    band[20:24, :] = True                               # cheap band y in [2.0, 2.4]
    s, g = (0.55, 1.25), (5.55, 1.25)                   # straight line 0.75 m below it

    def offset(c_max, w_trav=0.5):
        cost = np.where(band, 0, c_max).astype(np.uint8)
        planner = pe.GridPlanner(cost, pe.PlannerParams(w_traversal_cost=w_trav))
        xy, _ = pe.plan_xy(planner, grid, s, g)
        return float(np.max(xy[:, 1]) - s[1])

    assert offset(0) == pytest.approx(0.0)
    assert offset(40) == pytest.approx(0.0)             # 2.5 % per cell: detour not worth it
    assert offset(250, w_trav=2.0) > 0.7                # strong layer: route enters the band


# --------------------------------------------------------------- metrics ---

class _StubScorer:
    """n_eff = 1 for x >= x_good, else 0; s = 1 - exp(-n_eff)."""

    def __init__(self, x_good):
        self.x_good = x_good

    def components(self, x, y, yaw):
        n = (np.asarray(x) >= self.x_good).astype(float) + 0 * np.asarray(yaw)
        return {'n_eff': n, 's': 1.0 - np.exp(-n)}


def test_point_weights_and_longest_run():
    w = pe.point_weights([1.0, 1.0, 2.0])
    assert w.tolist() == [0.5, 1.0, 1.5, 1.0]
    assert pe.longest_run(w, [True, True, False, True]) == 1.5
    assert pe.longest_run(w, [False] * 4) == 0.0


def test_pose_metrics_fractions_gap_and_time():
    xy = np.stack([np.linspace(0, 4, 21), np.zeros(21)], axis=1)        # ds = 0.2 m
    yaw = np.zeros(21)
    m = pe.pose_metrics([(xy, yaw, 4.0)], _StubScorer(x_good=2.0 - 1e-9), v=0.5, wz=1.0)
    assert m['length_m'] == 4.0
    assert m['frac_neff_ge_0.5'] == pytest.approx(2.1 / 4.0)            # x >= 2 incl. half cell
    assert m['frac_s_ge_0.5'] == pytest.approx(2.1 / 4.0)
    assert m['longest_gap_m'] == pytest.approx(1.9)
    assert m['time_s'] == pytest.approx(8.0) and m['turning_rad'] == 0.0
    yaw2 = yaw.copy()
    yaw2[10:] = math.pi / 2                                             # 90 deg on one segment
    m2 = pe.pose_metrics([(xy, yaw2, 4.0)], _StubScorer(0.0), v=0.5, wz=1.0)
    assert m2['time_s'] == pytest.approx(8.0 - 0.4 + math.pi / 2)
    assert m2['max_rate_rad_per_m'] == pytest.approx((math.pi / 2) / 0.2)
    assert m2['turning_rad'] == pytest.approx(math.pi / 2)


def test_baseline_yaws_face_travel_with_pinned_ends():
    pts, yaw = pe.baseline_yaws(np.array([[0, 0], [2, 0], [2, 2]], float), 0.2, 1.0, -1.0)
    assert yaw[0] == 1.0 and yaw[-1] == -1.0
    assert np.allclose(yaw[1:9], 0.0) and np.allclose(yaw[12:-1], math.pi / 2)


# ------------------------------------------------------- real-world smoke ---

def test_prediction_on_real_world(real_landmarks_path):
    res = pe.run_prediction(c_max_main=(40,), c_max_sweep=(40,), w_f_sweep=(0.3,),
                            w_trav_sweep=(0.5,), probe_leg=None)
    v = res['variants']
    base, yaw_only, act = v['baseline']['total'], v['yaw-only']['total'], \
        v['active c_max=40']['total']
    assert yaw_only['extra_length_pct'] == 0.0
    assert yaw_only['frac_neff_ge_0.5'] > base['frac_neff_ge_0.5'] + 0.3
    assert act['extra_length_pct'] < 5.0
    yp = res['params']['yaw_planner']
    bin_w = 2 * math.pi / yp['n_bins']
    assert yaw_only['max_rate_rad_per_m'] <= yp['max_dyaw_per_m'] + bin_w / yp['step']
    for routes in res['routes'].values():
        for r in routes:
            _assert_traversable(res['grid'], res['master'], r['xy'])
