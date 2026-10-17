import numpy as np

from active_localization.geometry import wrap_angle
from active_localization.landmarks import CameraModel, Landmark, LandmarkArrays
from active_localization.scorers import NullScorer, VisibleCountScorer
from active_localization.yaw_planner import YawPlannerParams, plan_yaws

STRAIGHT = np.array([[0.0, 0.0], [6.0, 0.0]])


def lms(*pts):
    return LandmarkArrays.from_list([Landmark(i + 1, x, y) for i, (x, y) in enumerate(pts)])


def test_null_scorer_faces_travel_direction():
    path = np.array([[0, 0], [3, 0], [3, 3]], float)
    plan = plan_yaws(path, NullScorer(lms((0, 0)), CameraModel()), YawPlannerParams())
    err = np.abs(wrap_angle(plan.yaw - plan.travel))
    # bins are 10 deg; corner may be smoothed over a couple of samples
    assert np.median(err) < 1e-9 and err.max() <= np.pi / 2 + 1e-9


def test_turns_towards_side_marker():
    s = VisibleCountScorer(lms((3.0, 3.0)), CameraModel(), soft=0.05)
    p = YawPlannerParams(w_forward=0.2)
    plan = plan_yaws(STRAIGHT, s, p)
    mid = len(plan.yaw) // 2
    # marker is at +90 deg from (3, 0): heading must turn left, and see it
    assert plan.yaw[mid] > 0.8 and plan.score[mid] > 0.5
    base = plan_yaws(STRAIGHT, NullScorer(lms((3, 3)), CameraModel()), p)
    assert plan.score.sum() > s.score(base.xy[:, 0], base.xy[:, 1], base.yaw).sum()


def test_pins_start_and_goal():
    s = VisibleCountScorer(lms((3.0, 3.0)), CameraModel())
    plan = plan_yaws(STRAIGHT, s, YawPlannerParams(), start_yaw=0.3, goal_yaw=-1.0)
    assert abs(plan.yaw[0] - 0.3) < 1e-12 and abs(plan.yaw[-1] + 1.0) < 1e-12


def test_rate_limit_respected():
    s = VisibleCountScorer(lms((3.0, -3.0), (3.0, 3.0)), CameraModel(), soft=0.05)
    p = YawPlannerParams(max_dyaw_per_m=0.8, w_forward=0.0, w_smooth=0.0)
    plan = plan_yaws(STRAIGHT, s, p, start_yaw=np.pi)
    ds = np.linalg.norm(np.diff(plan.xy, axis=0), axis=1)
    dyaw = np.abs(wrap_angle(np.diff(plan.yaw)))
    bin_w = 2 * np.pi / p.n_bins
    # pinned start is exact (not a bin) -> allow one bin of slack
    assert np.all(dyaw <= p.max_dyaw_per_m * ds + bin_w + 1e-9)


def test_from_dict_rejects_unknown():
    import pytest
    with pytest.raises(ValueError):
        YawPlannerParams.from_dict({'nope': 1})
    assert YawPlannerParams.from_dict({'n_bins': '72'}).n_bins == 72
