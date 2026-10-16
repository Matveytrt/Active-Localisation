"""Odometry noise model (pure Python + numpy, no ROS dependencies).

The model consumes a stream of ground-truth (GT) planar poses (x, y, yaw, t)
and produces a drifting "wheel odometry" estimate:

1. The GT increment between two consecutive GT poses is expressed in the body
   frame of the previous GT pose: (dx_b, dy_b, dyaw). Using pose increments
   (instead of the twist) removes any ambiguity about the twist frame.
2. The increment is corrupted with motion-proportional errors (in the spirit
   of the probabilistic odometry motion model, Thrun et al.):

       trans = hypot(dx_b, dy_b),  rot = |dyaw|
       dx'   = dx_b * (1 + scale_x) + N(0, (a1*trans + a2*rot)^2)
       dy'   = dy_b * (1 + scale_y) + N(0, (a1*trans + a2*rot)^2)
       dyaw' = dyaw * (1 + scale_yaw) + yaw_drift_per_m * trans
               + N(0, (a3*rot + a4*trans)^2)

   If trans < min_motion and rot < min_motion the increment is passed through
   unchanged (no noise, no scale, no drift): a standing robot does not drift.
3. The noisy increment is composed onto the estimated pose (SE(2)).

Covariance (diagonal approximation, see README):
    pose_var  += (sigma_t^2, sigma_t^2, sigma_r^2) per step, reported + cov_floor
    twist_var  = (sigma_t^2, sigma_t^2, sigma_r^2) / dt^2 + cov_floor
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


@dataclass
class OdomNoiseParams:
    """Model parameters. Defaults are a plausible wheel-odometry scale only;
    the final values are a team decision."""
    enabled: bool = True
    init_from_gt: bool = True
    a1: float = 0.05             # trans noise per metre of translation [m/m]
    a2: float = 0.01             # trans noise per radian of rotation [m/rad]
    a3: float = 0.05             # rot noise per radian of rotation [rad/rad]
    a4: float = 0.01             # rot noise per metre of translation [rad/m]
    scale_x: float = 0.01        # systematic scale error, body x
    scale_y: float = 0.01        # systematic scale error, body y
    scale_yaw: float = 0.0       # systematic scale error, yaw
    yaw_drift_per_m: float = 0.01  # deterministic yaw drift [rad/m]
    min_motion: float = 1e-6     # below this (m and rad) no noise is applied
    cov_floor: float = 1e-6      # added to every reported variance

    @classmethod
    def names(cls) -> Tuple[str, ...]:
        return tuple(f.name for f in fields(cls))

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "OdomNoiseParams":
        known = set(cls.names())
        unknown = set(d) - known
        if unknown:
            raise ValueError("unknown noise parameters: %s" % sorted(unknown))
        return cls(**d)


@dataclass
class OdomSample:
    """One output sample of the model."""
    x: float
    y: float
    yaw: float
    vx: float          # body-frame linear velocity x [m/s]
    vy: float          # body-frame linear velocity y [m/s]
    wz: float          # yaw rate [rad/s]
    pose_var: Tuple[float, float, float]   # (var_x, var_y, var_yaw)
    twist_var: Tuple[float, float, float]  # (var_vx, var_vy, var_wz)


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
        self._var = [0.0, 0.0, 0.0]

    @property
    def initialized(self) -> bool:
        return self._prev_gt is not None

    def _floor(self, v) -> Tuple[float, float, float]:
        f = self.params.cov_floor
        return (v[0] + f, v[1] + f, v[2] + f)

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
            return OdomSample(self._pose[0], self._pose[1], self._pose[2],
                              0.0, 0.0, 0.0,
                              self._floor(self._var), self._floor((0.0, 0.0, 0.0)))

        dt = float(t) - self._prev_t
        d_gt = relative_pose(self._prev_gt, gt)
        self._prev_gt = gt
        self._prev_t = float(t)

        # --- identity mode -------------------------------------------------
        if not p.enabled:
            self._pose = gt
            vx, vy, wz = self._vel(d_gt, dt)
            return OdomSample(gt[0], gt[1], gt[2], vx, vy, wz,
                              self._floor(self._var), self._floor((0.0, 0.0, 0.0)))

        # --- noisy increment -----------------------------------------------
        dx_b, dy_b, dyaw = d_gt
        trans = math.hypot(dx_b, dy_b)
        rot = abs(dyaw)
        if trans < p.min_motion and rot < p.min_motion:
            d_noisy = d_gt
            step_var = (0.0, 0.0, 0.0)
        else:
            sig_t = p.a1 * trans + p.a2 * rot
            sig_r = p.a3 * rot + p.a4 * trans
            n = self._rng.standard_normal(3)
            d_noisy = (
                dx_b * (1.0 + p.scale_x) + sig_t * n[0],
                dy_b * (1.0 + p.scale_y) + sig_t * n[1],
                dyaw * (1.0 + p.scale_yaw) + p.yaw_drift_per_m * trans + sig_r * n[2],
            )
            step_var = (sig_t * sig_t, sig_t * sig_t, sig_r * sig_r)

        self._pose = compose(self._pose, d_noisy)
        # Diagonal accumulation. dx/dy noise is isotropic, so rotating it into
        # the odom frame does not change the diagonal; yaw->position coupling
        # (Jacobian propagation) is intentionally neglected.
        for i in range(3):
            self._var[i] += step_var[i]

        vx, vy, wz = self._vel(d_noisy, dt)
        if dt > 0.0:
            inv = 1.0 / (dt * dt)
            tw_var = (step_var[0] * inv, step_var[1] * inv, step_var[2] * inv)
        else:
            tw_var = (0.0, 0.0, 0.0)
        return OdomSample(self._pose[0], self._pose[1], self._pose[2], vx, vy, wz,
                          self._floor(self._var), self._floor(tw_var))

    @staticmethod
    def _vel(d: Tuple[float, float, float], dt: float) -> Tuple[float, float, float]:
        if dt <= 0.0:
            return (0.0, 0.0, 0.0)
        return (d[0] / dt, d[1] / dt, d[2] / dt)
