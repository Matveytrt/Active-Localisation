"""Scorer interface: the only thing the chosen method (GATE-1) has to provide.

A scorer maps a robot pose (x, y, yaw) in the map frame to a scalar
"localizability" score: higher = the pose is expected to localize better.
Everything downstream (yaw planner, cost field) only calls `score()`, so a
scorer can be swapped by a config string without touching the ROS nodes.

Contract:
- `score(x, y, yaw)` takes numpy-broadcastable arrays and returns an array of
  the broadcast shape, dtype float, finite, >= 0.
- `score_scale` is a typical "good enough" score (used to normalise weights,
  so planner weights do not need retuning when the scorer changes).
- Pure function of the pose and the static landmark map; no ROS, no state.
"""

from abc import ABC, abstractmethod

import numpy as np

from ..geometry import wrap_angle
from ..landmarks import CameraModel, LandmarkArrays, Observation


class Scorer(ABC):
    name = 'abstract'

    def __init__(self, landmarks: LandmarkArrays, camera: CameraModel, **params):
        self.lm = landmarks
        self.cam = camera
        self.params = params

    @property
    def score_scale(self) -> float:
        return 1.0

    @abstractmethod
    def score(self, x, y, yaw) -> np.ndarray:
        """Localizability of pose(s); see module docstring."""

    # ---- shared geometry -------------------------------------------------
    def camera_pose(self, x, y, yaw):
        """Camera centre and optical-axis yaw in the map frame.

        c = (x, y) + R(yaw) (offset_x, offset_y), camera yaw = yaw + offset_yaw.
        Outputs have shape broadcast(x, y, yaw).shape.
        """
        x, y, yaw = np.broadcast_arrays(np.asarray(x, float),
                                        np.asarray(y, float),
                                        np.asarray(yaw, float))
        c, s = np.cos(yaw), np.sin(yaw)
        cx = x + c * self.cam.offset_x - s * self.cam.offset_y
        cy = y + s * self.cam.offset_x + c * self.cam.offset_y
        return cx, cy, yaw + self.cam.offset_yaw

    def observe(self, x, y, yaw) -> Observation:
        """Camera->landmark geometry for every pose and every landmark.

        Output arrays have shape broadcast(x, y, yaw).shape + (L,).
        """
        cx, cy, cyaw = self.camera_pose(x, y, yaw)
        dx = self.lm.xy[:, 0] - cx[..., None]
        dy = self.lm.xy[:, 1] - cy[..., None]
        rng = np.hypot(dx, dy)
        bearing = wrap_angle(np.arctan2(dy, dx) - cyaw[..., None])
        # incidence: angle between (landmark -> camera) and marker normal
        safe = np.maximum(rng, 1e-9)
        cos_inc = (-dx * self.lm.normal[:, 0] - dy * self.lm.normal[:, 1]) / safe
        incidence = np.where(self.lm.has_normal,
                             np.arccos(np.clip(cos_inc, -1.0, 1.0)), 0.0)
        return Observation(rng=rng, bearing=bearing, incidence=incidence,
                           has_normal=self.lm.has_normal)

    def visibility(self, obs: Observation, soft: float = 0.0) -> np.ndarray:
        """Visibility in [0, 1] per landmark.

        soft == 0: hard FoV / range / incidence test.
        soft > 0: sigmoid edges (FIF eq. 26 style); `soft` is the edge width in
        radians for angles and in metres for range.
        """
        half = 0.5 * self.cam.hfov
        if soft <= 0.0:
            v = ((np.abs(obs.bearing) <= half)
                 & (obs.rng >= self.cam.min_range)
                 & (obs.rng <= self.cam.max_range)
                 & (obs.incidence <= self.cam.max_incidence))
            return v.astype(float)

        def sig(z):
            return 1.0 / (1.0 + np.exp(-z / soft))
        return (sig(half - np.abs(obs.bearing))
                * sig(obs.rng - self.cam.min_range)
                * sig(self.cam.max_range - obs.rng)
                * sig(self.cam.max_incidence - obs.incidence))
