"""Small SE(2) helpers shared by scorers, the yaw planner and the ROS nodes.

Pure numpy, no ROS. All angles in radians.
"""

import math

import numpy as np


def wrap_angle(a):
    """Wrap angle(s) to [-pi, pi). Works on scalars and arrays."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def yaw_from_quaternion(qx, qy, qz, qw):
    """Yaw (rotation about z) of a quaternion, ZYX convention."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def quaternion_from_yaw(yaw):
    """(qx, qy, qz, qw) for a pure rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def rotate_vector(q, v):
    """Rotate 3-vector v by unit quaternion q = (qx, qy, qz, qw)."""
    qx, qy, qz, qw = q
    u = np.array([qx, qy, qz], dtype=float)
    v = np.asarray(v, dtype=float)
    return v + 2.0 * np.cross(u, np.cross(u, v) + qw * v)


def resample_polyline(xy, step):
    """Resample a 2D polyline at (approximately) uniform arc-length `step`.

    Keeps the first and the last point exactly. Returns (M, 2) array, M >= 2
    unless the input has a single point.
    """
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if len(xy) < 2:
        return xy.copy()
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    if total <= 1e-9:
        return xy[[0, -1]].copy()
    n = max(int(math.ceil(total / step)), 1)
    s_new = np.linspace(0.0, total, n + 1)
    x = np.interp(s_new, s, xy[:, 0])
    y = np.interp(s_new, s, xy[:, 1])
    return np.stack([x, y], axis=1)


def path_headings(xy):
    """Direction of travel at each point of a polyline (last point repeats)."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if len(xy) < 2:
        return np.zeros(len(xy))
    d = np.diff(xy, axis=0)
    h = np.arctan2(d[:, 1], d[:, 0])
    return np.concatenate([h, h[-1:]])
