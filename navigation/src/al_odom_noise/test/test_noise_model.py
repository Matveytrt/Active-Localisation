"""Unit tests for al_odom_noise.noise_model (pure Python, no ROS needed)."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from al_odom_noise.noise_model import (  # noqa: E402
    OdomNoiseModel, OdomNoiseParams, wrap_angle)

DT = 0.02  # 50 Hz, as gazebo_ros_planar_move


def zero_noise(**kw):
    p = dict(a1=0.0, a2=0.0, a3=0.0, a4=0.0,
             scale_x=0.0, scale_y=0.0, scale_yaw=0.0, yaw_drift_per_m=0.0)
    p.update(kw)
    return OdomNoiseParams(**p)


def run(model, traj):
    """traj: array (N, 3) of x, y, yaw at DT spacing -> array (N, 3) of output."""
    out = []
    for i, (x, y, th) in enumerate(traj):
        s = model.update(x, y, th, i * DT)
        out.append((s.x, s.y, s.yaw))
    return np.array(out)


def pose_err(out, traj):
    dxy = np.hypot(out[:, 0] - traj[:, 0], out[:, 1] - traj[:, 1])
    dth = np.array([abs(wrap_angle(a - b)) for a, b in zip(out[:, 2], traj[:, 2])])
    return dxy.max(), dth.max()


def random_traj(n=2000, seed=0):
    rng = np.random.default_rng(seed)
    traj = np.zeros((n, 3))
    traj[0] = (-2.0, -0.5, 0.3)
    for i in range(1, n):
        vx, vy, wz = rng.normal(0.0, [0.5, 0.3, 0.8])
        x, y, th = traj[i - 1]
        c, s = math.cos(th), math.sin(th)
        traj[i] = (x + (c * vx - s * vy) * DT, y + (s * vx + c * vy) * DT,
                   wrap_angle(th + wz * DT))
    return traj


def circle_traj(n=3000, r=2.0, w=0.5):
    """Facing-forward motion on a circle (yaw tangent), several laps through +-pi."""
    t = np.arange(n) * DT
    return np.stack([r * np.cos(w * t), r * np.sin(w * t),
                     [wrap_angle(a + math.pi / 2) for a in w * t]], axis=1)


def sideways_traj(n=1500, v=0.4, yaw=0.7):
    """Pure strafe: constant heading, motion along body +y (omni base)."""
    t = np.arange(n) * DT
    return np.stack([1.0 - v * t * math.sin(yaw), 2.0 + v * t * math.cos(yaw),
                     np.full(n, yaw)], axis=1)


def strafe_and_spin_traj(n=3000):
    """Holonomic: translate along world x while spinning in place."""
    t = np.arange(n) * DT
    return np.stack([0.3 * t, 0.1 * np.sin(t),
                     [wrap_angle(a) for a in 1.2 * t]], axis=1)


# 1 -------------------------------------------------------------------------
def test_disabled_is_identity():
    traj = random_traj()
    m = OdomNoiseModel(OdomNoiseParams(enabled=False, init_from_gt=False), seed=1)
    out = run(m, traj)
    exy, eth = pose_err(out, traj)
    assert exy < 1e-9 and eth < 1e-9


# 2 -------------------------------------------------------------------------
@pytest.mark.parametrize('traj_fn', [circle_traj, sideways_traj,
                                     strafe_and_spin_traj, random_traj])
def test_zero_noise_integrator_matches_gt(traj_fn):
    traj = traj_fn()
    m = OdomNoiseModel(zero_noise(), seed=3)
    out = run(m, traj)
    exy, eth = pose_err(out, traj)
    assert exy < 1e-6 and eth < 1e-6


def test_zero_noise_sideways_velocity_is_body_y():
    traj = sideways_traj(v=0.4)
    m = OdomNoiseModel(zero_noise(), seed=3)
    s = None
    for i, (x, y, th) in enumerate(traj[:10]):
        s = m.update(x, y, th, i * DT)
    assert s.vx == pytest.approx(0.0, abs=1e-9)
    assert s.vy == pytest.approx(0.4, abs=1e-9)
    assert s.wz == pytest.approx(0.0, abs=1e-9)


def test_init_from_gt_false_starts_at_origin():
    traj = sideways_traj()
    m = OdomNoiseModel(zero_noise(init_from_gt=False), seed=3)
    out = run(m, traj)
    assert np.allclose(out[0], 0.0)
    # Relative motion still correct: final pose = first^-1 * last.
    x0, y0, th0 = traj[0]
    dx, dy = traj[-1, 0] - x0, traj[-1, 1] - y0
    c, s = math.cos(th0), math.sin(th0)
    assert out[-1, 0] == pytest.approx(c * dx + s * dy, abs=1e-6)
    assert out[-1, 1] == pytest.approx(-s * dx + c * dy, abs=1e-6)


# 3 -------------------------------------------------------------------------
def test_determinism_same_seed():
    traj = random_traj()
    a = run(OdomNoiseModel(OdomNoiseParams(), seed=42), traj)
    b = run(OdomNoiseModel(OdomNoiseParams(), seed=42), traj)
    assert np.array_equal(a, b)


def test_different_seed_differs():
    traj = random_traj()
    a = run(OdomNoiseModel(OdomNoiseParams(), seed=42), traj)
    b = run(OdomNoiseModel(OdomNoiseParams(), seed=43), traj)
    assert np.abs(a - b).max() > 1e-3


def test_reset_repeats_run():
    traj = random_traj(500)
    m = OdomNoiseModel(OdomNoiseParams(), seed=7)
    a = run(m, traj)
    m.reset()
    b = run(m, traj)
    assert np.array_equal(a, b)


# 4 -------------------------------------------------------------------------
@pytest.mark.parametrize('heading', [0.0, 1.0, -2.5])
def test_yaw_drift_straight_10m(heading):
    n = 1001  # 1000 steps of 1 cm = 10 m
    s = np.linspace(0.0, 10.0, n)
    traj = np.stack([s * math.cos(heading), s * math.sin(heading),
                     np.full(n, heading)], axis=1)
    m = OdomNoiseModel(zero_noise(yaw_drift_per_m=0.01), seed=0)
    out = run(m, traj)
    yaw_err = wrap_angle(out[-1, 2] - traj[-1, 2])
    assert yaw_err == pytest.approx(0.1, abs=1e-6)


# 5 -------------------------------------------------------------------------
def test_standing_robot_does_not_drift():
    m = OdomNoiseModel(OdomNoiseParams(), seed=5)  # default (noisy) params
    traj = random_traj(200)
    run(m, traj)  # move first, so the estimate is already off GT
    x, y, th = traj[-1]
    s0 = m.update(x, y, th, 200 * DT)
    for k in range(100):
        s = m.update(x, y, th, (201 + k) * DT)
        assert (s.x, s.y, s.yaw) == (s0.x, s0.y, s0.yaw)
        assert s.pose_var == s0.pose_var
        assert (s.vx, s.vy, s.wz) == (0.0, 0.0, 0.0)


# 6 -------------------------------------------------------------------------
def test_angle_wrap_no_jumps():
    # Spin in place for several turns (crosses +-pi many times) with a yaw
    # scale error: the error must grow smoothly, without 2*pi jumps.
    n = 2000
    w = 1.5
    traj = np.stack([np.zeros(n), np.zeros(n),
                     [wrap_angle(w * i * DT) for i in range(n)]], axis=1)
    m = OdomNoiseModel(zero_noise(scale_yaw=0.01), seed=0)
    out = run(m, traj)
    assert np.all(out[:, 2] >= -math.pi) and np.all(out[:, 2] < math.pi)
    err = np.array([wrap_angle(a - b) for a, b in zip(out[:, 2], traj[:, 2])])
    step = 0.01 * w * DT
    assert np.abs(np.diff(err)).max() < 2 * step
    assert err[-1] == pytest.approx(0.01 * w * (n - 1) * DT, abs=1e-6)


def test_zero_noise_matches_gt_across_pi():
    # Facing-forward circle: heading passes through +-pi each lap.
    traj = circle_traj(n=4000)
    assert np.any(np.abs(traj[:, 2]) > math.pi - 0.02)
    out = run(OdomNoiseModel(zero_noise(), seed=0), traj)
    _, eth = pose_err(out, traj)
    assert eth < 1e-6


# extra ---------------------------------------------------------------------
def test_default_noise_drifts_and_covariance_grows():
    traj = circle_traj()
    m = OdomNoiseModel(OdomNoiseParams(), seed=42)
    prev = None
    for i, (x, y, th) in enumerate(traj):
        s = m.update(x, y, th, i * DT)
        if prev is not None:
            assert all(a >= b for a, b in zip(s.pose_var, prev))
        prev = s.pose_var
        assert all(v > 0.0 for v in s.twist_var)
    assert math.hypot(s.x - traj[-1, 0], s.y - traj[-1, 1]) > 1e-3
    assert prev[0] > OdomNoiseParams().cov_floor


def test_nonpositive_dt_gives_zero_velocity():
    m = OdomNoiseModel(OdomNoiseParams(), seed=0)
    m.update(0.0, 0.0, 0.0, 1.0)
    s = m.update(0.1, 0.0, 0.0, 1.0)
    assert (s.vx, s.vy, s.wz) == (0.0, 0.0, 0.0)


def test_unknown_param_rejected():
    with pytest.raises(ValueError):
        OdomNoiseParams.from_dict({'a1': 0.1, 'bogus': 1.0})
