import numpy as np
import pytest

from active_localization.landmarks import CameraModel, LandmarkArrays, Landmark
from active_localization.scorers import REGISTRY, NullScorer, VisibleCountScorer, make_scorer


def lm(*pts, normals=None):
    out = []
    for i, (x, y) in enumerate(pts):
        n = None if normals is None else np.asarray(normals[i], float)
        out.append(Landmark(id=i + 1, x=x, y=y, normal_xy=n))
    return LandmarkArrays.from_list(out)


CAM = CameraModel()


def test_registry_and_factory():
    assert {'null', 'visible_count'} <= set(REGISTRY)
    assert isinstance(make_scorer('null', lm((1, 0)), CAM), NullScorer)
    with pytest.raises(ValueError):
        make_scorer('nope', lm((1, 0)), CAM)


def test_hard_visibility():
    s = VisibleCountScorer(lm((3, 0)), CAM)
    assert s.score(0, 0, 0.0) == 1.0              # straight ahead
    assert s.score(0, 0, np.pi) == 0.0            # behind
    assert s.score(0, 0, 0.6) == 0.0              # 0.6 rad > HFOV/2 = 0.545
    assert s.score(0, 0, 0.5) == 1.0
    assert s.score(-7, 0, 0.0) == 0.0             # 10 m > max_range 9 m
    assert s.score(2.9, 0, 0.0) == 0.0            # 0.1 m < min_range


def test_facing_check():
    lms = lm((3, 0), normals=[(-1, 0)])           # marker faces -x, i.e. towards origin
    s = VisibleCountScorer(lms, CAM)
    assert s.score(0, 0, 0.0) == 1.0
    assert s.score(6, 0, np.pi) == 0.0            # looking at its back


def test_soft_close_to_hard_far_from_edges():
    s = VisibleCountScorer(lm((3, 0)), CAM, soft=0.02)
    assert s.score(0, 0, 0.0) > 0.99
    assert s.score(0, 0, np.pi) < 1e-6
    edge = float(s.score(0, 0, CAM.hfov / 2))
    assert 0.3 < edge < 0.7


def test_broadcast_shape():
    s = VisibleCountScorer(lm((3, 0), (0, 3)), CAM)
    xs = np.zeros((5, 1))
    yaws = np.linspace(-np.pi, np.pi, 8, endpoint=False)[None, :]
    assert s.score(xs, xs, yaws).shape == (5, 8)
    assert NullScorer(lm((3, 0)), CAM).score(xs, xs, yaws).shape == (5, 8)


def test_weighting_and_saturation():
    lms = lm((1.5, 0), (3, 0.2), (6, -0.3))
    assert VisibleCountScorer(lms, CAM).score(0, 0, 0.0) == 3.0
    sat = VisibleCountScorer(lms, CAM, saturate=2.0)
    assert sat.score(0, 0, 0.0) == 2.0 and sat.score_scale == 2.0
    d2 = VisibleCountScorer(lm((1.5, 0)), CAM, weighting='inv_d2', ref_range=3.0)
    assert abs(float(d2.score(0, 0, 0.0)) - 4.0) < 1e-9
    with pytest.raises(ValueError):
        VisibleCountScorer(lms, CAM, weighting='bad')


def test_camera_offset():
    cam = CameraModel(offset_yaw=np.pi / 2)        # camera looks left of the base
    s = VisibleCountScorer(lm((0, 3)), cam)
    assert s.score(0, 0, 0.0) == 1.0
