"""S2* scorer (spec T-009 §2): check values and properties."""

import math

import numpy as np
import pytest

from active_localization.landmarks import CameraModel, Landmark, LandmarkArrays
from active_localization.scorers import REGISTRY, MarkerSoftCountScorer, make_scorer
from active_localization.scorers.marker_soft_count import S_REF, segments_hit_rects

CAM = CameraModel(hfov=1.089, offset_x=0.07)
ID1 = (0.19940729439258575, 0.8983477354049683)     # landmarks.yaml id 1


def lms(*pts):
    return LandmarkArrays.from_list([Landmark(i + 1, x, y) for i, (x, y) in enumerate(pts)])


def scorer(*pts, **kw):
    return MarkerSoftCountScorer(lms(*(pts or (ID1,))), CAM, **kw)


def facing(r, marker=ID1, yaw=math.pi / 2):
    """Base pose whose camera is r metres straight in front of the marker (-y side)."""
    return marker[0], marker[1] - r - CAM.offset_x, yaw


def test_registered():
    assert REGISTRY['marker_soft_count'] is MarkerSoftCountScorer
    assert isinstance(make_scorer('marker_soft_count', lms(ID1), CAM), MarkerSoftCountScorer)
    assert abs(scorer().score_scale - (1 - math.exp(-1))) < 1e-15 and S_REF == scorer().score_scale


def test_check_value_1m():
    c = scorer().components(*facing(1.0))
    assert abs(c['r'][0] - 1.0) < 1e-12 and abs(c['cos_theta'][0] - 1.0) < 1e-12
    assert c['w'][0] > 0.99
    assert abs(float(c['s']) - 0.63) < 0.01


def test_check_value_area_limit():
    c = scorer().components(*facing(1.88))
    assert abs(c['area'][0] - 500.0) < 10.0          # A = A_min at r ~ 1.88 m
    assert abs(c['w'][0] - 0.5) < 0.1


def test_check_value_near_limit():
    c = scorer().components(*facing(0.6))
    assert abs(c['w'][0] - 0.12) < 0.03


def test_turned_away():
    assert float(scorer().score(*facing(1.0, yaw=0.0))) < 0.01
    assert float(scorer().score(*facing(1.0, yaw=-math.pi / 2))) < 1e-6


def test_behind_the_marker_is_invisible():
    # camera on the +y side (behind the face), looking at the marker
    # A = 0 -> area gate = sig(-k_A) = 4.5e-5 (the soft floor of the spec formula)
    s = scorer(occlusion_rows=[])
    c = s.components(ID1[0], ID1[1] + 1.07, -math.pi / 2)
    assert c['area'][0] == 0.0
    assert abs(c['w'][0] - 1.0 / (1.0 + math.exp(10.0))) < 1e-6


def test_incidence_60deg_at_1p6m_not_visible():
    # r_max(60 deg) = f s sqrt(cos 60 / A_min) = 1.34 m < 1.6 m
    th = math.radians(60.0)
    cx, cy = ID1[0] + 1.6 * math.sin(th), ID1[1] - 1.6 * math.cos(th)
    yaw = math.atan2(ID1[1] - cy, ID1[0] - cx)
    x, y = cx - CAM.offset_x * math.cos(yaw), cy - CAM.offset_x * math.sin(yaw)
    c = scorer().components(x, y, yaw)
    assert abs(c['r'][0] - 1.6) < 1e-9 and abs(c['cos_theta'][0] - 0.5) < 1e-9
    assert c['w'][0] < 0.1
    r_max = 528.4 * 0.08 * math.sqrt(0.5 / 500.0)
    assert 1.3 < r_max < 1.36


def test_occlusion_rect_blocks():
    # marker at (0, 3) facing -y, a shelf row between it and the camera
    s = scorer((0.0, 3.0), occlusion_rows=[[-1.0, 1.0, 1.0, 1.5]])
    free = scorer((0.0, 3.0), occlusion_rows=[])
    pose = (0.0, 1.8 - CAM.offset_x, math.pi / 2)       # camera 1.2 m in front, above the row
    assert float(s.score(*pose)) > 0.5                   # row is behind the camera
    pose = (0.0, 0.5 - CAM.offset_x, math.pi / 2)        # row between camera and marker
    assert s.components(*pose)['occ'][0] == 0.0 and float(s.score(*pose)) == 0.0
    assert free.components(*pose)['occ'][0] == 1.0


def test_occlusion_ignores_last_0p2m():
    # the marker face lies on the rectangle edge: the segment must stop before it
    s = scorer((0.0, 1.0), occlusion_rows=[[-1.0, 1.0, 1.0, 1.5]])
    assert s.components(*facing(1.0, marker=(0.0, 1.0)))['occ'][0] == 1.0
    # a rectangle reaching into the last 0.2 m (but not into the rest) does not block
    s = scorer((0.0, 1.0), occlusion_rows=[[-1.0, 1.0, 0.85, 1.5]])
    assert s.components(*facing(1.0, marker=(0.0, 1.0)))['occ'][0] == 1.0
    s = scorer((0.0, 1.0), occlusion_rows=[[-1.0, 1.0, 0.75, 1.5]])
    assert s.components(*facing(1.0, marker=(0.0, 1.0)))['occ'][0] == 0.0


def test_default_rows_row1_hides_row2_from_the_south():
    s = scorer((0.16, 3.41))                             # id 4 on row 2
    c = s.components(0.16, -0.5, math.pi / 2)
    assert c['occ'][0] == 0.0
    c = s.components(0.16, 2.3, math.pi / 2)             # between the rows
    assert c['occ'][0] == 1.0 and c['w'][0] > 0.9


def test_segments_hit_rects_cases():
    rect = [[0.0, 1.0, 0.0, 1.0]]
    a = np.array([-1.0, -1.0, 0.5, -1.0, -1.0, 2.0])
    b = np.array([2.0, -0.5, 0.5, -1.0, 0.0, 2.0])
    ay = np.array([0.5, 0.5, -1.0, -1.0, 0.0, 2.0])
    by = np.array([0.5, 0.5, 2.0, 2.0, 1.0, 3.0])
    hit = segments_hit_rects(a, ay, b, by, rect)
    #   through, stops short, vertical through, vertical outside, touches corner, far
    assert hit.tolist() == [True, False, True, False, True, False]
    assert segments_hit_rects(0.0, 0.0, 1.0, 1.0, []).shape == ()


def test_broadcast_shape_and_range():
    s = scorer(ID1, (1.9, 0.89), (0.16, 3.41))
    xs = np.linspace(-1, 2, 5)[:, None, None]
    ys = np.linspace(-1, 3, 4)[None, :, None]
    yaws = np.linspace(-np.pi, np.pi, 6, endpoint=False)[None, None, :]
    v = s.score(xs, ys, yaws)
    assert v.shape == (5, 4, 6) and np.all(np.isfinite(v))
    assert np.all(v >= 0) and np.all(v < 1)
    c = s.components(xs, ys, yaws)
    assert c['w'].shape == (5, 4, 6, 3) and c['n_eff'].shape == (5, 4, 6)
    # element-wise equal to scalar evaluation
    assert abs(v[2, 1, 3] - float(s.score(xs[2, 0, 0], ys[0, 1, 0], yaws[0, 0, 3]))) < 1e-15


def test_saturation_and_two_markers():
    s = scorer((0.0, 1.0), (0.3, 1.0), occlusion_rows=[])
    c = s.components(0.15, 1.0 - 1.2 - CAM.offset_x, math.pi / 2)
    assert 1.5 < float(c['n_eff']) < 2.0
    assert abs(float(c['s']) - (1 - math.exp(-float(c['n_eff'])))) < 1e-15


def test_normals_param():
    # marker faces +x; camera 1 m along +x looking back
    s = scorer((0.0, 0.0), normals={1: [2.0, 0.0]}, occlusion_rows=[])
    assert np.allclose(s.normals, [[1.0, 0.0]])
    assert float(s.score(1.0 + CAM.offset_x, 0.0, math.pi)) > 0.6
    with pytest.raises(ValueError):
        scorer((0.0, 0.0), normals={7: [1.0, 0.0]})
    with pytest.raises(ValueError):
        scorer((0.0, 0.0), normals={1: [0.0, 0.0]})
    with pytest.raises(ValueError):
        scorer((0.0, 0.0), bogus=1)


def test_camera_offset_is_used():
    s = scorer()
    x, y, yaw = facing(0.70)                             # camera exactly at r_min
    assert abs(scorer().components(x, y, yaw)['r'][0] - 0.70) < 1e-12
    assert abs(s.components(x, y, yaw)['g_near'][0] - 0.5) < 1e-12
