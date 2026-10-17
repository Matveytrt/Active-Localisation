"""Shared setup for the offline tools (score maps, golden fixtures).

Not a ROS node. The tools put the package directory on sys.path themselves,
so they run with `python3 -I tools/<tool>.py` from any working directory.
"""

import os
import sys

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(TOOLS_DIR)
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from active_localization.cost_field import GridSpec  # noqa: E402
from active_localization.landmarks import (CameraModel, LandmarkArrays,  # noqa: E402
                                           load_landmarks)
from active_localization.scorers.marker_soft_count import (DEFAULTS,  # noqa: E402
                                                           MarkerSoftCountScorer)

LANDMARKS_FILE = os.path.normpath(
    os.path.join(PKG_DIR, '..', 'rtabmap_landmarks', 'config', 'landmarks.yaml'))
LANDMARKS_REL = 'navigation/src/rtabmap_landmarks/config/landmarks.yaml'
MARKER_IDS = [1, 2, 3, 4, 5]

# Spec T-009 §1/§2: HFOV (FACTS camera), camera 0.07 m ahead of base_link (FACTS 18.10).
CAMERA = {'hfov': 1.089, 'offset_x': 0.07, 'offset_y': 0.0, 'offset_yaw': 0.0}

# Room box of the world (LEAD, spec task text) and map resolution.
ROOM = {'xmin': -4.9, 'xmax': 3.3, 'ymin': -2.1, 'ymax': 5.4}
RESOLUTION = 0.05

# Spec T-009 §3 layer parameters.
LAYER = {'c_max': 40, 'power': 1.0, 'yaw_bins': 36}


def build_scorer(**overrides):
    lms = load_landmarks(LANDMARKS_FILE, ids=MARKER_IDS)
    params = {k: v for k, v in DEFAULTS.items()}
    params.update(overrides)
    return MarkerSoftCountScorer(LandmarkArrays.from_list(lms), CameraModel(**CAMERA), **params)


def room_grid(resolution=RESOLUTION):
    w = int(round((ROOM['xmax'] - ROOM['xmin']) / resolution))
    h = int(round((ROOM['ymax'] - ROOM['ymin']) / resolution))
    return GridSpec(origin_x=ROOM['xmin'], origin_y=ROOM['ymin'],
                    resolution=resolution, width=w, height=h)


def scorer_header(scorer):
    """Every parameter the C++ side needs to reproduce the scorer."""
    return {
        'formula': 'w_i = sig(k_fov*(hfov/2-|beta_i|)) * sig(k_a*(A_i/a_min-1)) '
                   '* sig(k_r*(r_i-r_min)) * occ_i; A_i = (focal_px*marker_size/r_i)^2 '
                   '* max(cos_theta_i,0); n_eff = sum w_i; s = 1-exp(-n_eff/m_sat); '
                   'sig(z) = 1/(1+exp(-z)); camera c = (x,y) + R(yaw)*(offset_x,offset_y), '
                   'camera yaw = yaw + offset_yaw; beta wrapped to [-pi,pi)',
        'camera': dict(CAMERA),
        'k_fov': scorer.k_fov, 'a_min': scorer.a_min, 'k_a': scorer.k_a,
        'r_min': scorer.r_min, 'k_r': scorer.k_r, 'focal_px': scorer.focal_px,
        'marker_size': scorer.marker_size, 'm_sat': scorer.m_sat,
        's_ref': scorer.score_scale,
        'markers': [{'id': int(i), 'x': float(p[0]), 'y': float(p[1]),
                     'normal': [float(n[0]), float(n[1])]}
                    for i, p, n in zip(scorer.lm.ids, scorer.lm.xy, scorer.normals)],
        'occlusion': {
            'rule': 'occ_i = 0 if the 2D segment c -> (p_i + occ_stop*n_i) touches any '
                    'closed rectangle [xmin,xmax,ymin,ymax], else 1 (Python oracle rule; '
                    'the C++ layer ray-casts the static map instead) [assumption: rows]',
            'occ_stop': scorer.occ_stop,
            'rows': scorer.occlusion_rows.tolist(),
        },
        'landmarks_file': LANDMARKS_REL,
    }
