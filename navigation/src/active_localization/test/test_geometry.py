import numpy as np

from active_localization.geometry import (path_headings, quaternion_from_yaw,
                                          resample_polyline, rotate_vector,
                                          wrap_angle, yaw_from_quaternion)


def test_wrap_angle_range():
    a = np.linspace(-10, 10, 101)
    w = wrap_angle(a)
    assert np.all(w >= -np.pi) and np.all(w < np.pi)
    assert np.allclose(np.cos(w), np.cos(a)) and np.allclose(np.sin(w), np.sin(a))


def test_quaternion_roundtrip():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 3.1):
        assert abs(wrap_angle(yaw_from_quaternion(*quaternion_from_yaw(yaw)) - yaw)) < 1e-12


def test_rotate_vector_yaw90():
    q = quaternion_from_yaw(np.pi / 2)
    assert np.allclose(rotate_vector(q, [1, 0, 0]), [0, 1, 0])


def test_resample_keeps_endpoints_and_spacing():
    xy = np.array([[0, 0], [1, 0], [1, 2]], float)
    r = resample_polyline(xy, 0.25)
    assert np.allclose(r[0], xy[0]) and np.allclose(r[-1], xy[-1])
    d = np.linalg.norm(np.diff(r, axis=0), axis=1)
    assert np.all(d <= 0.25 + 1e-9) and abs(d.sum() - 3.0) < 1e-9


def test_resample_degenerate():
    assert resample_polyline([[1, 1]], 0.1).shape == (1, 2)
    assert resample_polyline([[1, 1], [1, 1]], 0.1).shape == (2, 2)


def test_path_headings():
    h = path_headings([[0, 0], [1, 0], [1, 1]])
    assert np.allclose(h, [0, np.pi / 2, np.pi / 2])
