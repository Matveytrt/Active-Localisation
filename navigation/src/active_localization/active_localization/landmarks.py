"""Landmark map + camera model.

The landmark file is the one RTAB-Map export already produces
(`rtabmap_landmarks/config/landmarks.yaml`: `frame_id`, `big_markers[]` with
`id`, `position`, `orientation`). We read it as is.

Marker facing direction: which axis of the landmark frame points out of the
marker face is NOT verified yet (T-003 / ENG). It is a parameter
(`normal_axis`, e.g. "+z", "-y"); `None` disables the facing check.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np
import yaml

from .geometry import rotate_vector

_AXES = {'x': np.array([1.0, 0.0, 0.0]),
         'y': np.array([0.0, 1.0, 0.0]),
         'z': np.array([0.0, 0.0, 1.0])}


@dataclass
class Landmark:
    id: int
    x: float
    y: float
    z: float = 0.0
    # Unit 2D vector of the marker face normal in the map frame, or None
    # when unknown (then visibility ignores the incidence angle).
    normal_xy: Optional[np.ndarray] = None


@dataclass
class CameraModel:
    """Forward-looking camera rigidly attached to the base.

    Defaults: torso_rgbd, HFOV 1.089 rad (FACTS). Range limits: 0.3 m (near)
    and 9 m (RTAB-Map landmark max range, FACTS). Offset of the camera from
    base_link in the base frame: [unverified] 0 until ENG reads it from TF.
    """
    hfov: float = 1.089
    min_range: float = 0.3
    max_range: float = 9.0
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_yaw: float = 0.0
    # Max angle between the camera->marker ray and the marker normal.
    max_incidence: float = 1.2

    @classmethod
    def from_dict(cls, d):
        d = dict(d or {})
        return cls(**{k: float(v) for k, v in d.items()})


def _normal_from_axis(quat, axis: Optional[str]):
    if not axis:
        return None
    sign = -1.0 if axis.startswith('-') else 1.0
    key = axis.lstrip('+-').lower()
    if key not in _AXES:
        raise ValueError(f'normal_axis must be one of [+-]x/y/z, got {axis!r}')
    n = sign * rotate_vector(quat, _AXES[key])
    nxy = n[:2]
    norm = np.linalg.norm(nxy)
    if norm < 1e-6:
        # Normal is vertical: facing check is meaningless for a planar robot.
        return None
    return nxy / norm


def load_landmarks(path: str, normal_axis: Optional[str] = None,
                   ids: Optional[Sequence[int]] = None,
                   expect_frame: str = 'map') -> List[Landmark]:
    """Load `landmarks.yaml` (RTAB-Map export format)."""
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return landmarks_from_dict(data, normal_axis=normal_axis, ids=ids,
                               expect_frame=expect_frame)


def landmarks_from_dict(data, normal_axis=None, ids=None, expect_frame='map'):
    frame = data.get('frame_id', expect_frame)
    if expect_frame and frame != expect_frame:
        raise ValueError(f'landmarks are in frame {frame!r}, expected {expect_frame!r}')
    raw = data.get('big_markers') or data.get('landmarks') or []
    if not raw:
        raise ValueError('no landmarks found (expected key big_markers or landmarks)')
    out = []
    for m in raw:
        lid = int(m['id'])
        if ids is not None and lid not in ids:
            continue
        p = m['position']
        o = m.get('orientation')
        normal = None
        if o is not None:
            q = (float(o['x']), float(o['y']), float(o['z']), float(o['w']))
            normal = _normal_from_axis(q, normal_axis)
        out.append(Landmark(id=lid, x=float(p['x']), y=float(p['y']),
                            z=float(p.get('z', 0.0)), normal_xy=normal))
    return out


@dataclass
class LandmarkArrays:
    """Landmarks as arrays, for vectorized scorers."""
    ids: np.ndarray
    xy: np.ndarray            # (L, 2)
    normal: np.ndarray        # (L, 2), zeros where unknown
    has_normal: np.ndarray    # (L,) bool

    @classmethod
    def from_list(cls, lms: List[Landmark]):
        ids = np.array([m.id for m in lms], dtype=int)
        xy = np.array([[m.x, m.y] for m in lms], dtype=float).reshape(-1, 2)
        normal = np.zeros_like(xy)
        has = np.zeros(len(lms), dtype=bool)
        for i, m in enumerate(lms):
            if m.normal_xy is not None:
                normal[i] = m.normal_xy
                has[i] = True
        return cls(ids=ids, xy=xy, normal=normal, has_normal=has)


@dataclass
class Observation:
    """Per-pose, per-landmark camera geometry (broadcast shape S + (L,))."""
    rng: np.ndarray        # distance camera -> landmark [m]
    bearing: np.ndarray    # angle of the landmark relative to the optical axis [rad]
    incidence: np.ndarray  # angle between ray (landmark -> camera) and marker normal; 0 if unknown
    has_normal: np.ndarray = field(default=None)
