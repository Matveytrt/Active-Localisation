"""Invalid or colliding jobs must never start/stop containers or overwrite runs."""
from pathlib import Path
from types import SimpleNamespace
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_experiment as runner


@pytest.fixture
def environment(tmp_path, monkeypatch):
    database = tmp_path/'map.db'
    database.write_bytes(b'fixture, not a real map')
    monkeypatch.setattr(runner, 'MAP', database)
    monkeypatch.setattr(runner, 'WORK', tmp_path)
    monkeypatch.setattr(runner, 'command', lambda *a, **k: pytest.fail('Docker/process action before preflight'))
    monkeypatch.setattr(runner, 'running', lambda name: False)
    return SimpleNamespace(batch='test', seed=42, modes=['baseline', 'active', 'control'])


def test_wrong_map_fails_before_any_process(environment):
    with pytest.raises(AssertionError, match='Pristine'):
        runner.execute(environment)


def test_existing_run_never_overwritten(environment, monkeypatch):
    monkeypatch.setattr(runner, 'FROZEN_SHA', runner.sha(runner.MAP))
    folder = runner.WORK/'experiments/test'
    folder.mkdir(parents=True)
    evidence = folder/'keep.txt'
    evidence.write_text('original')
    with pytest.raises(AssertionError, match='already exists'):
        runner.execute(environment)
    assert evidence.read_text() == 'original'


def test_busy_simulator_is_not_stopped(environment, monkeypatch):
    monkeypatch.setattr(runner, 'FROZEN_SHA', runner.sha(runner.MAP))
    monkeypatch.setattr(runner, 'running', lambda name: name == 'al-isaac-astribot')
    with pytest.raises(AssertionError, match='busy'):
        runner.execute(environment)


def test_path_traversal_before_mutation(environment):
    environment.batch = '../other'
    with pytest.raises(AssertionError, match='Unsafe'):
        runner.execute(environment)
