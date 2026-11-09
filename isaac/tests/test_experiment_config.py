"""Protect the original control experiment and the accepted execution profile."""
from pathlib import Path
import sys
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from experiment_config import nav_parameters, validate_seed

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture
def landmarks(tmp_path):
    path = tmp_path/'landmarks.yaml'
    path.write_text(yaml.safe_dump({'frame_id': 'map', 'source': 'optimized_graph',
                                  'big_markers': [{'id': n} for n in range(1, 6)]}))
    return path


def differences(a, b, path=''):
    if isinstance(a, dict) and isinstance(b, dict):
        return sum((differences(a[k], b[k], path+'.'+k) for k in a.keys() | b.keys()), [])
    return [] if a == b else [path]


def test_control_only_removes_localization_objective(landmarks):
    active, _ = nav_parameters(REPO, 'active', landmarks)
    control, _ = nav_parameters(REPO, 'control', landmarks)
    assert set(differences(active, control)) == {
        '.planner_server.ros__parameters.GridBased.scorer',
        '.global_costmap.global_costmap.ros__parameters.loc_layer.scorer',
        '.global_costmap.global_costmap.ros__parameters.loc_layer.enabled'}
    for mode in ('baseline', 'active', 'control'):
        data, provenance = nav_parameters(REPO, mode, landmarks)
        assert data['controller_server']['ros__parameters']['FollowPath']['wz_max'] == .8
        assert data['velocity_smoother']['ros__parameters']['max_velocity'] == [.6, .6, .8]
        assert len(provenance['source_sha256']) == 64


@pytest.mark.parametrize('value', [-1, 2**32-1, True, 2.5, '42'])
def test_invalid_seed_rejected(value):
    with pytest.raises(ValueError):
        validate_seed(value)


def test_reject_source_pose_or_wrong_landmark_set(landmarks):
    content = yaml.safe_load(landmarks.read_text())
    content['source'] = 'raw_graph'
    landmarks.write_text(yaml.safe_dump(content))
    with pytest.raises(AssertionError):
        nav_parameters(REPO, 'active', landmarks)
    content['source'] = 'optimized_graph'
    content['big_markers'].append({'id': 0})
    landmarks.write_text(yaml.safe_dump(content))
    with pytest.raises(AssertionError):
        nav_parameters(REPO, 'control', landmarks)
