"""Seed parsing must agree for noise and lidars before any Docker operation."""
from pathlib import Path
import subprocess
import pytest

SCRIPT = Path(__file__).resolve().parents[1]/'start_bridge.sh'


@pytest.mark.parametrize('value,expected', [('0', 0), ('08', 8), ('42', 42), ('4294967294', 4294967294)])
def test_decimal_seed(value, expected):
    prefix = SCRIPT.read_text().split('mkdir -p', 1)[0]
    proc = subprocess.run(['bash', '-c', prefix+'echo "$AL_ISAAC_SEED"', str(SCRIPT), '--seed', value],
                          text=True, capture_output=True)
    assert proc.returncode == 0, proc.stderr
    assert int(proc.stdout.strip()) == expected


@pytest.mark.parametrize('value', ['-1', 'NaN', '4294967295', '99999999999999999999', ''])
def test_bad_seed_before_docker(value):
    prefix = SCRIPT.read_text().split('mkdir -p', 1)[0]
    proc = subprocess.run(['bash', '-c', prefix+'exit 0', str(SCRIPT), '--seed', value],
                          text=True, capture_output=True)
    assert proc.returncode == 2
    assert 'Invalid seed' in proc.stderr
