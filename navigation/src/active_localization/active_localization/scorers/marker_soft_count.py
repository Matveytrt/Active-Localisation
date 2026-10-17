"""S2* scorer (ADR-001, spec T-009 §2): soft count of detectable markers.

Per marker i, with camera centre c = (x, y) + R(yaw) (offset_x, 0):

    delta_i = p_i - c,  r_i = |delta_i|  (horizontal only; the ~0.2 m height
                                          difference is ignored [assumption, spec §1])
    beta_i  = wrap(atan2(delta_i.y, delta_i.x) - yaw_cam)
    cos t_i = (-delta_i . n_i) / r_i
    A_i     = (f s / r_i)^2 * max(cos t_i, 0)                [px^2]

    w_i = sig(k_fov (HFOV/2 - |beta_i|)) * sig(k_A (A_i / A_min - 1))
          * sig(k_r (r_i - r_min)) * occ_i

    n_eff = sum_i w_i,   s = 1 - exp(-n_eff / M),   score_scale = 1 - exp(-1)

sig(z) = 1 / (1 + exp(-z)) (evaluated as 0.5 (1 + tanh(z / 2)), identical
up to rounding and free of overflow warnings).

Occlusion (oracle only, rule-based): occ_i = 0 if the 2D segment
c -> q_i, q_i = p_i + occ_stop * n_i (i.e. the segment stops occ_stop = 0.20 m
in front of the marker face, on the camera side), touches any axis-aligned
shelf rectangle [xmin, xmax, ymin, ymax] in `occlusion_rows`; else 1. The C++
layer ray-casts the static map instead (spec §2), so the fixtures carry occ_i.

Marker normals: the S2* spec fixes the outward face normal per marker by
parameter (default (0, -1) for all markers: every marker faces -y). The
landmark file's orientation / `landmarks.normal_axis` is NOT used by this
scorer, so Python and C++ agree without a quaternion-axis convention.
World file check: the HD17 marker visuals have roll = +pi/2, which turns the
face (+z of the marker box) to -y, consistent with the default.
"""

import math

import numpy as np

from ..geometry import wrap_angle
from .base import Scorer

S_REF = 1.0 - math.exp(-1.0)

# [assumption] Shelf rows as merged rectangles (LEAD, from the world). The
# individual collision boxes in worlds/astribot_hd15_hd17_walls.world are
# x in [-2.825, -1.505], [-1.125, 0.195], [0.575, 1.895] with 0.38 m gaps,
# y in [0.9, 1.5] (row 1) and [3.4, 4.0] (row 2); the merged rows are a
# conservative superset (gaps treated as blocked).
DEFAULT_OCCLUSION_ROWS = ((-2.9, 2.15, 0.9, 1.5),
                          (-2.9, 2.15, 3.4, 4.0))

DEFAULTS = {
    'k_fov': 40.0,        # rad^-1 (~5 deg edge), RND §3 C2
    'a_min': 500.0,       # px^2, ENG-10 fit (485-508)
    'k_a': 10.0,          # LEAD choice
    'r_min': 0.70,        # m, ENG-10
    'k_r': 20.0,          # m^-1, ENG-10
    'focal_px': 528.4,    # px
    'marker_size': 0.08,  # m
    'm_sat': 1.0,         # M, ADR-001
    'default_normal': (0.0, -1.0),
    'occ_stop': 0.20,     # m, segment ends this far in front of the marker
    'occlusion_rows': DEFAULT_OCCLUSION_ROWS,
}


def sigmoid(z):
    return 0.5 * (1.0 + np.tanh(0.5 * np.asarray(z, float)))


def segments_hit_rects(ax, ay, bx, by, rects):
    """True where segment a->b touches any closed axis-aligned rectangle.

    a*, b* broadcastable arrays of shape S; rects (K, 4) [xmin, xmax, ymin, ymax].
    Returns bool array of shape S (Liang-Barsky slab test).
    """
    rects = np.asarray(rects, float).reshape(-1, 4)
    ax, ay, bx, by = np.broadcast_arrays(*(np.asarray(v, float) for v in (ax, ay, bx, by)))
    if len(rects) == 0:
        return np.zeros(ax.shape, dtype=bool)
    a = (ax[..., None], ay[..., None])
    d = (bx[..., None] - a[0], by[..., None] - a[1])
    lo = (rects[:, 0], rects[:, 2])
    hi = (rects[:, 1], rects[:, 3])
    t_in = np.zeros(ax.shape + (len(rects),))
    t_out = np.ones_like(t_in)
    ok = np.ones(t_in.shape, dtype=bool)
    for k in range(2):
        par = np.abs(d[k]) < 1e-12
        ok &= ~par | ((a[k] >= lo[k]) & (a[k] <= hi[k]))
        dk = np.where(par, 1.0, d[k])
        t1 = (lo[k] - a[k]) / dk
        t2 = (hi[k] - a[k]) / dk
        t_in = np.where(par, t_in, np.maximum(t_in, np.minimum(t1, t2)))
        t_out = np.where(par, t_out, np.minimum(t_out, np.maximum(t1, t2)))
    return np.any(ok & (t_in <= t_out), axis=-1)


class MarkerSoftCountScorer(Scorer):
    """S2*: 1 - exp(-n_eff / M), n_eff = soft count of detectable markers.

    params (defaults = spec §2, see DEFAULTS):
      k_fov, a_min, k_a, r_min, k_r, focal_px, marker_size, m_sat
      normals:        optional {id: [nx, ny]}; markers not listed use
                      default_normal (normalised to unit length)
      occlusion_rows: list of [xmin, xmax, ymin, ymax]; [] disables occlusion
      occ_stop:       metres in front of the marker where the segment ends
    HFOV and the camera offset come from the CameraModel (offset_x = 0.07 m,
    FACTS 18.10). CameraModel min/max_range and max_incidence are not used.
    """
    name = 'marker_soft_count'

    def __init__(self, landmarks, camera, normals=None, **params):
        unknown = set(params) - set(DEFAULTS)
        if unknown:
            raise ValueError(f'unknown marker_soft_count params: {sorted(unknown)}')
        super().__init__(landmarks, camera, **params)
        p = dict(DEFAULTS)
        p.update(params)
        self.k_fov = float(p['k_fov'])
        self.a_min = float(p['a_min'])
        self.k_a = float(p['k_a'])
        self.r_min = float(p['r_min'])
        self.k_r = float(p['k_r'])
        self.focal_px = float(p['focal_px'])
        self.marker_size = float(p['marker_size'])
        self.m_sat = float(p['m_sat'])
        self.occ_stop = float(p['occ_stop'])
        if self.m_sat <= 0 or self.a_min <= 0:
            raise ValueError('m_sat and a_min must be > 0')
        self.occlusion_rows = np.asarray(p['occlusion_rows'] or [], float).reshape(-1, 4)
        normals = {int(k): v for k, v in (normals or {}).items()}
        unknown_ids = set(normals) - set(int(i) for i in landmarks.ids)
        if unknown_ids:
            raise ValueError(f'normals given for unknown marker ids {sorted(unknown_ids)}')
        n = np.array([normals.get(int(i), p['default_normal']) for i in landmarks.ids],
                     float).reshape(-1, 2)
        norm = np.linalg.norm(n, axis=1, keepdims=True)
        if np.any(norm < 1e-9):
            raise ValueError('marker normals must be non-zero')
        self.normals = n / norm                                          # (L, 2)
        self.occ_targets = self.lm.xy + self.occ_stop * self.normals     # (L, 2)

    @property
    def score_scale(self):
        return S_REF

    def components(self, x, y, yaw):
        """All intermediate terms; arrays of shape S + (L,) except n_eff, s (S)."""
        cx, cy, cyaw = self.camera_pose(x, y, yaw)
        dx = self.lm.xy[:, 0] - cx[..., None]
        dy = self.lm.xy[:, 1] - cy[..., None]
        r = np.hypot(dx, dy)
        beta = wrap_angle(np.arctan2(dy, dx) - cyaw[..., None])
        cos_t = (-dx * self.normals[:, 0] - dy * self.normals[:, 1]) / np.maximum(r, 1e-9)
        area = (self.focal_px * self.marker_size / np.maximum(r, 1e-9)) ** 2 \
            * np.maximum(cos_t, 0.0)
        g_fov = sigmoid(self.k_fov * (0.5 * self.cam.hfov - np.abs(beta)))
        g_area = sigmoid(self.k_a * (area / self.a_min - 1.0))
        g_near = sigmoid(self.k_r * (r - self.r_min))
        occ = 1.0 - segments_hit_rects(cx[..., None], cy[..., None],
                                       self.occ_targets[:, 0], self.occ_targets[:, 1],
                                       self.occlusion_rows).astype(float)
        w = g_fov * g_area * g_near * occ
        n_eff = w.sum(axis=-1)
        s = 1.0 - np.exp(-n_eff / self.m_sat)
        return {'r': r, 'beta': beta, 'cos_theta': cos_t, 'area': area,
                'g_fov': g_fov, 'g_area': g_area, 'g_near': g_near, 'occ': occ,
                'w': w, 'n_eff': n_eff, 's': s}

    def score(self, x, y, yaw):
        return self.components(x, y, yaw)['s']
