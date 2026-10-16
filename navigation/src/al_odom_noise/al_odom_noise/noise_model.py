"""Odometry noise model (pure Python + numpy, no ROS dependencies).

The model consumes a stream of ground-truth (GT) planar poses (x, y, yaw, t)
and produces a drifting "wheel odometry" estimate:

1. The GT increment between two consecutive GT poses is expressed in the body
   frame of the previous GT pose: (dx_b, dy_b, dyaw). Using pose increments
   (instead of the twist) removes any ambiguity about the twist frame.
2. The increment is corrupted with random-walk noise whose *variance* is
   proportional to the distance / angle travelled (so the accumulated random
   error does not depend on the input rate), plus deterministic errors:

       trans = hypot(dx_b, dy_b),  rot = |dyaw|
       var_dx   = k_xx*trans + k_xr*rot
       var_dy   = k_yy*trans + k_yr*rot
       var_dyaw = k_rr*rot   + k_rt*trans
       dx'   = dx_b * (1 + scale_x) + N(0, var_dx)
       dy'   = dy_b * (1 + scale_y) + N(0, var_dy)
       dyaw' = dyaw * (1 + scale_yaw) + yaw_drift_per_m * trans + N(0, var_dyaw)

   If trans < min_motion and rot < min_motion the increment is passed through
   unchanged (no noise, no scale, no drift): a standing robot does not drift.
3. The noisy increment is composed onto the estimated pose (SE(2)).

Covariance: full 3x3 (x, y, yaw) first-order propagation through SE(2)
composition, P <- F P F^T + G Q G^T, Q = diag(var_dx, var_dy, var_dyaw).
Deterministic errors (scale_*, yaw_drift_per_m) are NOT represented in P.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Any, Dict, Optional, Tuple

import numpy as np


def wrap_angle(a: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def relative_pose(p_from: Tuple[float, float, float],
                  p_to: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Pose of p_to expressed in the frame of p_from (p_from^-1 * p_to)."""
    x0, y0, th0 = p_from
    x1, y1, th1 = p_to
    dxw = x1 - x0
    dyw = y1 - y0
    c = math.cos(th0)
    s = math.sin(th0)
    return (c * dxw + s * dyw, -s * dxw + c * dyw, wrap_angle(th1 - th0))


def compose(p: Tuple[float, float, float],
            d: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """SE(2) composition p * d, with the result angle normalized."""
    x, y, th = p
    dx, dy, dth = d
    c = math.cos(th)
    s = math.sin(th)
    return (x + c * dx - s * dy, y + s * dx + c * dy, wrap_angle(th + dth))


def compose_jacobians(p: Tuple[float, float, float],
                      d: Tuple[float, float, float]) -> Tuple[np.ndarray, np.ndarray]:
    """Jacobians of compose(p, d): F = d/dp, G = d/dd."""
    th = p[2]
    dx, dy = d[0], d[1]
    c = math.cos(th)
    s = math.sin(th)
    F = np.array([[1.0, 0.0, -s * dx - c * dy],
                  [0.0, 1.0, c * dx - s * dy],
                  [0.0, 0.0, 1.0]])
    G = np.array([[c, -s, 0.0],
                  [s, c, 0.0],
                  [0.0, 0.0, 1.0]])
    return F, G


@dataclass
class OdomNoiseParams:
    """Model parameters. Defaults: see README "How the defaults were chosen";
    the final values are a team decision."""
    enabled: bool = True
    init_from_gt: bool = True
    k_xx: float = 2.5e-4         # var of dx per metre travelled [m^2/m]
    k_xr: float = 1.0e-5         # var of dx per radian turned [m^2/rad]
    k_yy: float = 2.5e-4         # var of dy per metre travelled [m^2/m]
    k_yr: float = 1.0e-5         # var of dy per radian turned [m^2/rad]
    k_rr: float = 5.0e-4         # var of dyaw per radian turned [rad^2/rad]
    k_rt: float = 1.0e-5         # var of dyaw per metre travelled [rad^2/m]
    scale_x: float = 0.01        # systematic scale error, body x
    scale_y: float = 0.01        # systematic scale error, body y
    scale_yaw: float = 0.0       # systematic scale error, yaw
    yaw_drift_per_m: float = 0.01  # deterministic yaw drift [rad/m]
    min_motion: float = 1e-6     # below this (m and rad) no noise is applied
    cov_floor: float = 1e-6      # added to every reported diagonal variance

    @classmethod
    def names(cls) -> Tuple[str, ...]:
        return tuple(f.name for f in fields(cls))

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "OdomNoiseParams":
        known = set(cls.names())
        unknown = set(d) - known
        if unknown:
            raise ValueError("unknown noise parameters: %s" % sorted(unknown))
        p = cls(**d)
        for name in ('k_xx', 'k_xr', 'k_yy', 'k_yr', 'k_rr', 'k_rt'):
            if getattr(p, name) < 0.0:
                raise ValueError("%s must be >= 0 (it is a variance rate)" % name)
        return p


@dataclass
class OdomSample:
    """One output sample of the model."""
    x: float
    y: float
    yaw: float
    vx: float          # body-frame linear velocity x [m/s]
    vy: float          # body-frame linear velocity y [m/s]
    wz: float          # yaw rate [rad/s]
    pose_cov: np.ndarray   # 3x3 covariance of (x, y, yaw), odom frame
    twist_var: Tuple[float, float, float]  # (var_vx, var_vy, var_wz)

    @property
    def pose_var(self) -> Tuple[float, float, float]:
        """Diagonal of pose_cov."""
        return (float(self.pose_cov[0, 0]), float(self.pose_cov[1, 1]),
                float(self.pose_cov[2, 2]))


class OdomNoiseModel:
    """SE(2) integrator turning GT poses into drifting noisy odometry."""

    def __init__(self, params: Optional[OdomNoiseParams] = None, seed: int = 42):
        self.params = params if params is not None else OdomNoiseParams()
        self.seed = int(seed)
        self.reset()

    def reset(self) -> None:
        """Forget all state (RNG is re-seeded too, so a reset run repeats)."""
        self._rng = np.random.default_rng(self.seed)
        self._prev_gt: Optional[Tuple[float, float, float]] = None
        self._prev_t: Optional[float] = None
        self._pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._P = np.zeros((3, 3))

    @property
    def initialized(self) -> bool:
        return self._prev_gt is not None

    def _reported_cov(self) -> np.ndarray:
        return self._P + self.params.cov_floor * np.eye(3)

    def _floor(self, v) -> Tuple[float, float, float]:
        f = self.params.cov_floor
        return (v[0] + f, v[1] + f, v[2] + f)

    def _sample(self, vx: float, vy: float, wz: float,
                tw_var: Tuple[float, float, float]) -> OdomSample:
        return OdomSample(self._pose[0], self._pose[1], self._pose[2], vx, vy, wz,
                          self._reported_cov(), self._floor(tw_var))

    def update(self, x: float, y: float, yaw: float, t: float) -> OdomSample:
        """Feed one GT pose at time t [s]; return the noisy odometry sample."""
        p = self.params
        gt = (float(x), float(y), wrap_angle(float(yaw)))

        # --- first pose: initialize only ---------------------------------
        if self._prev_gt is None:
            self._prev_gt = gt
            self._prev_t = float(t)
            if p.enabled and not p.init_from_gt:
                self._pose = (0.0, 0.0, 0.0)
            else:
                self._pose = gt
            return self._sample(0.0, 0.0, 0.0, (0.0, 0.0, 0.0))

        dt = float(t) - self._prev_t
        d_gt = relative_pose(self._prev_gt, gt)
        self._prev_gt = gt
        self._prev_t = float(t)

        # --- identity mode -------------------------------------------------
        if not p.enabled:
            self._pose = gt
            vx, vy, wz = self._vel(d_gt, dt)
            return self._sample(vx, vy, wz, (0.0, 0.0, 0.0))

        # --- noisy increment -----------------------------------------------
        dx_b, dy_b, dyaw = d_gt
        trans = math.hypot(dx_b, dy_b)
        rot = abs(dyaw)
        if trans < p.min_motion and rot < p.min_motion:
            d_noisy = d_gt
            q = (0.0, 0.0, 0.0)
        else:
            q = (p.k_xx * trans + p.k_xr * rot,
                 p.k_yy * trans + p.k_yr * rot,
                 p.k_rr * rot + p.k_rt * trans)
            n = self._rng.standard_normal(3)
            d_noisy = (
                dx_b * (1.0 + p.scale_x) + math.sqrt(q[0]) * n[0],
                dy_b * (1.0 + p.scale_y) + math.sqrt(q[1]) * n[1],
                dyaw * (1.0 + p.scale_yaw) + p.yaw_drift_per_m * trans
                + math.sqrt(q[2]) * n[2],
            )
            # First-order propagation, linearized at the current estimate.
            F, G = compose_jacobians(self._pose, d_noisy)
            P = F @ self._P @ F.T + G @ np.diag(q) @ G.T
            self._P = 0.5 * (P + P.T)

        self._pose = compose(self._pose, d_noisy)

        vx, vy, wz = self._vel(d_noisy, dt)
        if dt > 0.0:
            inv = 1.0 / (dt * dt)
            tw_var = (q[0] * inv, q[1] * inv, q[2] * inv)
        else:
            tw_var = (0.0, 0.0, 0.0)
        return self._sample(vx, vy, wz, tw_var)

    @staticmethod
    def _vel(d: Tuple[float, float, float], dt: float) -> Tuple[float, float, float]:
        if dt <= 0.0:
            return (0.0, 0.0, 0.0)
        return (d[0] / dt, d[1] / dt, d[2] / dt)
