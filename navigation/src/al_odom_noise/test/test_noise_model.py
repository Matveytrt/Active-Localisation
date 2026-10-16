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
    p = dict(k_xx=0.0, k_xr=0.0, k_yy=0.0, k_yr=0.0, k_rr=0.0, k_rt=0.0,
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
def test_default_noise_drifts_and_covariance_is_valid():
    traj = circle_traj()
    m = OdomNoiseModel(OdomNoiseParams(), seed=42)
    for i, (x, y, th) in enumerate(traj):
        s = m.update(x, y, th, i * DT)
        P = s.pose_cov
        assert P.shape == (3, 3)
        assert np.allclose(P, P.T, atol=0.0)
        assert np.linalg.eigvalsh(P).min() > 0.0
        assert all(v > 0.0 for v in s.twist_var)
    assert math.hypot(s.x - traj[-1, 0], s.y - traj[-1, 1]) > 1e-3
    assert s.pose_var[2] > 10 * OdomNoiseParams().cov_floor
    assert abs(P[0, 1]) > 0.0 and abs(P[1, 2]) > 0.0  # cross terms present


def test_nonpositive_dt_gives_zero_velocity():
    m = OdomNoiseModel(OdomNoiseParams(), seed=0)
    m.update(0.0, 0.0, 0.0, 1.0)
    s = m.update(0.1, 0.0, 0.0, 1.0)
    assert (s.vx, s.vy, s.wz) == (0.0, 0.0, 0.0)


def test_unknown_param_rejected():
    with pytest.raises(ValueError):
        OdomNoiseParams.from_dict({'k_xx': 0.1, 'bogus': 1.0})
    with pytest.raises(ValueError):
        OdomNoiseParams.from_dict({'k_rr': -1.0})


# T-004c: Monte-Carlo consistency of the reported covariance ------------------
MC_SEEDS = 500
MC_RATES = (10.0, 50.0)
MC_TOL = 0.2


def mc_straight(hz, length=10.0, v=1.0):
    n = int(round(length / v * hz)) + 1
    t = np.arange(n) / hz
    return t, np.stack([v * t, np.zeros(n), np.zeros(n)], axis=1)


def mc_circle(hz, r=2.0, w=1.0):
    n = int(round(2.0 * math.pi / w * hz)) + 1  # one lap
    t = np.arange(n) / hz
    return t, np.stack([r * np.cos(w * t), r * np.sin(w * t),
                        [wrap_angle(a + math.pi / 2) for a in w * t]], axis=1)


_MC_CACHE = {}


def monte_carlo(traj_fn, hz):
    """Empirical covariance of the final pose error and mean reported P."""
    key = (traj_fn.__name__, hz)
    if key not in _MC_CACHE:
        t, traj = traj_fn(hz)
        params = OdomNoiseParams(scale_x=0.0, scale_y=0.0, scale_yaw=0.0,
                                 yaw_drift_per_m=0.0, cov_floor=0.0)
        errs, covs = [], []
        for seed in range(MC_SEEDS):
            m = OdomNoiseModel(params, seed=seed)
            for ti, (x, y, th) in zip(t, traj):
                s = m.update(x, y, th, ti)
            errs.append((s.x - traj[-1, 0], s.y - traj[-1, 1],
                         wrap_angle(s.yaw - traj[-1, 2])))
            covs.append(s.pose_cov)
        _MC_CACHE[key] = (np.cov(np.array(errs).T), np.mean(covs, axis=0))
    return _MC_CACHE[key]


def assert_cov_close(C, P, tol=MC_TOL):
    """Diagonal: relative error <= tol. Off-diagonal: error <= tol*sigma_i*sigma_j
    (i.e. correlation coefficient error <= tol; relative error is ill-defined
    for terms that are ~0)."""
    sd = np.sqrt(np.diag(P))
    for i in range(3):
        assert abs(C[i, i] / P[i, i] - 1.0) <= tol, (i, C[i, i], P[i, i])
        for j in range(i + 1, 3):
            assert abs(C[i, j] - P[i, j]) <= tol * sd[i] * sd[j], (i, j, C[i, j], P[i, j])


@pytest.mark.parametrize('hz', MC_RATES)
@pytest.mark.parametrize('traj_fn', [mc_straight, mc_circle])
def test_mc_reported_cov_matches_empirical(traj_fn, hz):
    C, P = monte_carlo(traj_fn, hz)
    assert_cov_close(C, P)


@pytest.mark.parametrize('traj_fn', [mc_straight, mc_circle])
def test_mc_cov_independent_of_rate(traj_fn):
    C10, P10 = monte_carlo(traj_fn, MC_RATES[0])
    C50, P50 = monte_carlo(traj_fn, MC_RATES[1])
    assert_cov_close(P50, P10)   # reported
    assert_cov_close(C50, C10)   # empirical


def test_default_sigma_y_10m_straight():
    # Design target from LEAD (T-004c): random sigma_y over 10 m in 0.05..0.1 m.
    _, P = monte_carlo(mc_straight, 50.0)
    assert 0.05 <= math.sqrt(P[1, 1]) <= 0.1


def test_cov_jacobians_numeric():
    from al_odom_noise.noise_model import compose, compose_jacobians
    p, d, h = (0.3, -1.2, 2.9), (0.4, -0.25, 0.3), 1e-6
    F, G = compose_jacobians(p, d)
    for k in range(3):
        e = np.zeros(3)
        e[k] = h
        fp = (np.array(compose(tuple(np.add(p, e)), d)) - np.array(compose(p, d))) / h
        gd = (np.array(compose(p, tuple(np.add(d, e)))) - np.array(compose(p, d))) / h
        assert np.allclose(F[:, k], fp, atol=1e-5)
        assert np.allclose(G[:, k], gd, atol=1e-5)
