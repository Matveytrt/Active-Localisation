import os

import pytest
import yaml

from active_localization.config import build_config, load_config

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(PKG, 'config', 'active_localization.yaml')


def _raw(real_path):
    with open(CFG) as f:
        raw = yaml.safe_load(f)
    raw['landmarks']['file'] = real_path
    return raw


def test_shipped_config_builds(real_landmarks_path):
    cfg = build_config(_raw(real_landmarks_path))
    assert cfg.scorer.name == 'visible_count'
    assert len(cfg.scorer.lm.ids) == 5
    assert cfg.navigator['action_name'] == 'al/navigate_to_pose'


def test_scorer_override(real_landmarks_path):
    assert build_config(_raw(real_landmarks_path), scorer_override='null').scorer.name == 'null'


def test_unknown_key_rejected(real_landmarks_path):
    raw = _raw(real_landmarks_path)
    raw['typo'] = 1
    with pytest.raises(ValueError):
        build_config(raw)


def test_load_config_relative_path(tmp_path, real_landmarks_path):
    raw = _raw(os.path.relpath(real_landmarks_path, tmp_path))
    p = tmp_path / 'c.yaml'
    p.write_text(yaml.safe_dump(raw))
    assert load_config(str(p)).scorer.name == 'visible_count'
