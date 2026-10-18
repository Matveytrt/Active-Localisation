import numpy as np
import pytest

from active_localization.cost_field import CostFieldParams, GridSpec, best_score_grid, cost_grid
from active_localization.landmarks import CameraModel, Landmark, LandmarkArrays
from active_localization.scorers import VisibleCountScorer

GRID = GridSpec(origin_x=-10.0, origin_y=-10.0, resolution=0.5, width=40, height=40)


def scorer():
    lm = LandmarkArrays.from_list([Landmark(1, 0.0, 0.0)])
    # min_range 0.5 m > distance of the nearest cell centre (0.354 m)
    return VisibleCountScorer(lm, CameraModel(min_range=0.5))


def cell(x, y):
    return int((y - GRID.origin_y) / GRID.resolution), int((x - GRID.origin_x) / GRID.resolution)


def test_shape_dtype_and_range():
    c = cost_grid(scorer(), GRID, CostFieldParams(good_score=1.0, cost_max=60))
    assert c.shape == (40, 40) and c.dtype == np.int8
    assert c.min() == 0 and c.max() == 60


def test_near_good_far_bad():
    c = cost_grid(scorer(), GRID, CostFieldParams(good_score=1.0, cost_max=60))
    assert c[cell(2.1, 0.1)] == 0          # 2 m from the marker: can always turn to see it
    assert c[cell(-9.6, -9.6)] == 60       # > 9 m away
    assert c[cell(0.1, 0.1)] == 60         # inside min_range


def test_best_over_yaw_is_rotation_free():
    b = best_score_grid(scorer(), GRID, 36)
    assert b[cell(2.1, 0.1)] == b[cell(0.1, 2.1)] == b[cell(-1.9, 0.1)] == 1.0


def test_cost_max_must_stay_below_lethal():
    with pytest.raises(ValueError):
        cost_grid(scorer(), GRID, CostFieldParams(cost_max=100))


# ---- spec T-009 §3 layer cost (C++ LocalizabilityLayer oracle) --------------

def test_layer_cost_formula():
    from active_localization.cost_field import S_REF, layer_cost_from_score
    s = np.array([0.0, S_REF, 1.0, S_REF / 2, S_REF * (1 - 0.51 / 40), S_REF * (1 - 0.49 / 40)])
    c = layer_cost_from_score(s, c_max=40)
    assert c.dtype == np.uint8
    assert c.tolist() == [40, 0, 0, 20, 1, 0]          # nearest integer
    assert layer_cost_from_score(S_REF / 2, c_max=40, power=2.0) == 10
    assert layer_cost_from_score(0.0, c_max=252) == 252
    with pytest.raises(ValueError):
        layer_cost_from_score(0.0, c_max=253)


def test_layer_cost_grid_band_in_front_of_marker():
    from active_localization.cost_field import layer_cost_grid
    from active_localization.scorers import MarkerSoftCountScorer
    lm = LandmarkArrays.from_list([Landmark(1, 0.0, 0.0)])
    s = MarkerSoftCountScorer(lm, CameraModel(offset_x=0.07), occlusion_rows=[])
    g = GridSpec(origin_x=-3.0, origin_y=-3.0, resolution=0.1, width=60, height=60)
    c = layer_cost_grid(s, g, c_max=40)
    assert c.shape == (60, 60) and c.dtype == np.uint8 and c.max() == 40

    def at(x, y):
        return c[int((y + 3.0) / 0.1), int((x + 3.0) / 0.1)]
    assert at(0.05, -1.25) <= 1                         # 1.2 m straight in front
    assert at(0.05, 1.25) == 40                         # behind the marker face
    assert at(0.05, -2.75) >= 35                        # too far (area gate)
    assert at(0.05, -0.35) >= 30                        # too near (r_min gate)
    # unchanged OccupancyGrid path still works with the new scorer
    occ = cost_grid(s, g, CostFieldParams(good_score=s.score_scale, cost_max=16))
    assert occ.dtype == np.int8 and occ.max() == 16
