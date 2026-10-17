"""One YAML file configures the whole package (see config/active_localization.yaml).

Kept outside the ROS parameter system on purpose: nested dicts for scorer
params are awkward as ROS params, and the same file is used by tests and
offline scripts. ROS nodes only receive `config_file` (+ a few overrides).
"""

import os
from dataclasses import dataclass

import yaml

from .cost_field import CostFieldParams
from .landmarks import CameraModel, LandmarkArrays, load_landmarks
from .scorers import Scorer, make_scorer
from .yaw_planner import YawPlannerParams

_TOP_KEYS = {'landmarks', 'camera', 'scorer', 'yaw_planner', 'cost_field', 'navigator'}


@dataclass
class ALConfig:
    scorer: Scorer
    yaw_planner: YawPlannerParams
    cost_field: CostFieldParams
    navigator: dict
    raw: dict


def _resolve(path, base_dir):
    path = os.path.expanduser(os.path.expandvars(path))
    if not os.path.isabs(path) and base_dir:
        path = os.path.join(base_dir, path)
    return path


def build_config(raw: dict, base_dir: str = '', scorer_override: str = '') -> ALConfig:
    unknown = set(raw) - _TOP_KEYS
    if unknown:
        raise ValueError(f'unknown top-level keys: {sorted(unknown)}')
    lm_cfg = raw.get('landmarks') or {}
    if 'file' not in lm_cfg:
        raise ValueError('landmarks.file is required')
    lms = load_landmarks(_resolve(lm_cfg['file'], base_dir),
                         normal_axis=lm_cfg.get('normal_axis'),
                         ids=lm_cfg.get('ids'),
                         expect_frame=lm_cfg.get('frame_id', 'map'))
    cam = CameraModel.from_dict(raw.get('camera'))
    sc = dict(raw.get('scorer') or {})
    name = scorer_override or sc.get('name', 'null')
    scorer = make_scorer(name, LandmarkArrays.from_list(lms), cam, **(sc.get('params') or {}))
    return ALConfig(scorer=scorer,
                    yaw_planner=YawPlannerParams.from_dict(raw.get('yaw_planner')),
                    cost_field=CostFieldParams.from_dict(raw.get('cost_field')),
                    navigator=dict(raw.get('navigator') or {}),
                    raw=raw)


def load_config(path: str, scorer_override: str = '') -> ALConfig:
    with open(path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}
    return build_config(raw, base_dir=os.path.dirname(os.path.abspath(path)),
                        scorer_override=scorer_override)
