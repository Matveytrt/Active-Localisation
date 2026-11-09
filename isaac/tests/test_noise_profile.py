from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from experiment_config import noise_parameters, VARIANCE_KEYS, BIAS_KEYS
from run_series import protocol
REPO = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('scale', [1., 4., 6.])
def test_noise_contract(scale):
    base = noise_parameters(REPO, 42, 1.)['odom_noise_node']['ros__parameters']
    actual = noise_parameters(REPO, 46, scale)['odom_noise_node']['ros__parameters']
    for key in VARIANCE_KEYS:
        assert actual[key] == base[key]*scale**2
    for key in BIAS_KEYS:
        assert actual[key] == base[key]*scale
    assert actual['seed'] == 46
    for key in set(base)-set(VARIANCE_KEYS)-set(BIAS_KEYS)-{'seed'}:
        assert actual[key] == base[key]


@pytest.mark.parametrize('scale', [0., -1., float('nan'), float('inf'), True])
def test_invalid_scale(scale):
    with pytest.raises(ValueError): noise_parameters(REPO, 42, scale)


def test_complete_paired_protocol():
    jobs = protocol()
    assert len(jobs) == 30
    for scale in (4,6):
        for seed in range(42,47):
            group = [j for j in jobs if j['noise_scale']==scale and j['seed']==seed]
            assert {j['mode'] for j in group} == {'baseline','active','control'}
            assert {j['position'] for j in group} == {1,2,3}
