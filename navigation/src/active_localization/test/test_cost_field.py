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
