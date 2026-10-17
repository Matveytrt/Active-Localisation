"""Yaw lever: choose a heading for every pose of a fixed (x, y) path.

The base is omnidirectional and the camera is fixed to the torso, so the
heading along the path is a free variable that only the camera cares about
(the lidar is 360 deg). We pick it by dynamic programming (Viterbi) over a
discretised set of headings:

    min_{b_0..b_N}  sum_i  node(i, b_i)  +  sum_i  edge(i, b_i, b_{i+1})

    node(i, b) = - w_score   * score(x_i, y_i, yaw_b) / score_scale
                 + w_forward * (1 - cos(yaw_b - heading_of_travel_i))
    edge(i, a, b) = w_smooth * dyaw^2
                  + w_limit  * max(0, |dyaw| - max_dyaw_per_m * ds_i)^2

w_forward reproduces the baseline habit (face where you go) when the scorer
has nothing to say; w_limit keeps the yaw rate feasible for the controller
(wz_max / v). Start and goal headings can be pinned. Complexity O(N * B^2).

This is method-agnostic: any `Scorer` plugs in. ActLoc does per-waypoint
viewpoint selection with a smoothness term; RAVPP's MPC term
Q_f (theta - theta_f)^2 is the continuous analogue of node + edge here.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .geometry import path_headings, resample_polyline, wrap_angle


@dataclass
class YawPlannerParams:
    n_bins: int = 36
    step: float = 0.25          # resampling step along the path [m]
    w_score: float = 1.0
    w_forward: float = 0.5
    w_smooth: float = 0.5
    w_limit: float = 100.0
    max_dyaw_per_m: float = 1.0  # ~ wz_max / v_max of MPPI (1.0 / 1.0) [rad/m]
    pin_start: bool = True
    pin_goal: bool = True

    @classmethod
    def from_dict(cls, d):
        d = dict(d or {})
        out = cls()
        for k, v in d.items():
            if not hasattr(out, k):
                raise ValueError(f'unknown yaw planner param {k!r}')
            setattr(out, k, type(getattr(out, k))(v))
        return out


@dataclass
class YawPlan:
    xy: np.ndarray       # (N, 2) resampled path
    yaw: np.ndarray      # (N,)  chosen headings
    score: np.ndarray    # (N,)  scorer value at the chosen headings
    travel: np.ndarray   # (N,)  direction of travel
    bin_idx: Optional[np.ndarray] = None   # (N,) chosen bin per waypoint (before pinning)


def plan_yaws(xy, scorer, params: YawPlannerParams = YawPlannerParams(),
              start_yaw: Optional[float] = None,
              goal_yaw: Optional[float] = None) -> YawPlan:
    p = params
    pts = resample_polyline(xy, p.step) if p.step > 0 else np.asarray(xy, float)
    n = len(pts)
    travel = path_headings(pts)
    if n == 0:
        z = np.zeros(0)
        return YawPlan(pts, z, z, z, np.zeros(0, dtype=int))

    bins = -np.pi + 2.0 * np.pi * np.arange(p.n_bins) / p.n_bins       # (B,)
    scale = max(float(scorer.score_scale), 1e-9)
    s = scorer.score(pts[:, 0:1], pts[:, 1:2], bins[None, :])           # (N, B)
    node = (-p.w_score * s / scale
            + p.w_forward * (1.0 - np.cos(bins[None, :] - travel[:, None])))

    big = 1e6
    if p.pin_start and start_yaw is not None:
        node[0] += big * (1.0 - np.cos(bins - start_yaw))
    if p.pin_goal and goal_yaw is not None:
        node[-1] += big * (1.0 - np.cos(bins - goal_yaw))

    dyaw = np.abs(wrap_angle(bins[None, :] - bins[:, None]))           # (B, B) from a to b
    ds = np.linalg.norm(np.diff(pts, axis=0), axis=1) if n > 1 else np.zeros(0)

    cost = node[0].copy()
    back = np.zeros((n, p.n_bins), dtype=int)
    for i in range(1, n):
        excess = np.maximum(0.0, dyaw - p.max_dyaw_per_m * ds[i - 1])
        edge = p.w_smooth * dyaw ** 2 + p.w_limit * excess ** 2
        total = cost[:, None] + edge                                     # (B_prev, B)
        back[i] = np.argmin(total, axis=0)
        cost = total[back[i], np.arange(p.n_bins)] + node[i]

    idx = np.zeros(n, dtype=int)
    idx[-1] = int(np.argmin(cost))
    for i in range(n - 1, 0, -1):
        idx[i - 1] = back[i, idx[i]]

    yaw = bins[idx].astype(float)
    if p.pin_start and start_yaw is not None:
        yaw[0] = float(wrap_angle(start_yaw))
    if p.pin_goal and goal_yaw is not None:
        yaw[-1] = float(wrap_angle(goal_yaw))
    chosen = s[np.arange(n), idx]
    return YawPlan(xy=pts, yaw=yaw, score=chosen, travel=travel, bin_idx=idx)
