"""Tests for al_eval.scenario (pure Python, no ROS) and the dry run of
scripts/run_experiment.sh (skipped if bash or the script is not found)."""

import importlib
import math
import os
import shutil
import subprocess
import sys
import time
import types

import pytest
import yaml

# allow `pytest navigation/src/al_eval/test` from anywhere (no colcon)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from al_eval import scenario as sc  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(PKG, '..', '..', '..'))
EXAMPLE = os.path.join(PKG, 'config', 'scenarios', 'example.yaml')
SCRIPT = os.path.join(REPO, 'scripts', 'run_experiment.sh')


def _valid(**over):
    d = {'name': 'loop', 'frame_id': 'map', 'goal_timeout_s': 60,
         'settle_s': 1.5,
         'goals': [{'x': 1, 'y': 0.0, 'yaw': 0.0},
                   {'x': 1.0, 'y': 2.5, 'yaw': -1.0}]}
    d.update(over)
    return d


# ------------------------------------------------------------ parsing
def test_parse_valid():
    s = sc.parse_scenario(_valid())
    assert s.name == 'loop' and s.frame_id == 'map'
    assert s.goal_timeout_s == 60.0 and s.settle_s == 1.5
    assert s.goals == [sc.Goal(1.0, 0.0, 0.0), sc.Goal(1.0, 2.5, -1.0)]
    assert isinstance(s.goals[0].x, float)


def test_defaults_applied():
    s = sc.parse_scenario({'name': 'n',
                           'goals': [{'x': 0, 'y': 0, 'yaw': 0}]})
    assert s.frame_id == 'map'
    assert s.goal_timeout_s == sc.DEFAULTS['goal_timeout_s']
    assert s.settle_s == sc.DEFAULTS['settle_s']


def test_example_file_is_valid_and_marked_placeholder():
    s = sc.load_scenario(EXAMPLE)
    assert s.name == 'example'
    assert 3 <= len(s.goals) <= 4
    with open(EXAMPLE) as f:
        assert 'PLACEHOLDER goals: ENG replaces after mapping (T-002)' \
            in f.read()


@pytest.mark.parametrize('data, pattern', [
    ({'name': 'x', 'frame_id': 'map'}, r"missing 'goals'"),
    (_valid(goals=[]), r"non-empty list"),
    (_valid(goals=None), r"non-empty list"),
    (_valid(goals={'x': 1}), r"non-empty list"),
    (_valid(goals=[{'x': 1, 'y': 2}]), r"goals\[0\]: missing \['yaw'\]"),
    (_valid(goals=[{'x': 'one', 'y': 2, 'yaw': 0}]),
     r"goals\[0\]\.x must be a number"),
    (_valid(goals=[{'x': 1, 'y': True, 'yaw': 0}]),
     r"goals\[0\]\.y must be a number"),
    (_valid(goals=[{'x': 1, 'y': 0, 'yaw': float('nan')}]),
     r"goals\[0\]\.yaw must be finite"),
    (_valid(goals=[{'x': 1, 'y': 0, 'yaw': 0, 'z': 0}]),
     r"goals\[0\]: unknown key"),
    (_valid(goals=[[1, 2, 3]]), r"goals\[0\] must be a mapping"),
    (_valid(frame_id='odom'), r"unknown frame_id 'odom'"),
    (_valid(frame_id='world'), r"unknown frame_id 'world'"),
    (_valid(goal_timeout_s='120'), r"'goal_timeout_s' must be a number"),
    (_valid(goal_timeout_s=0), r"'goal_timeout_s' must be > 0"),
    (_valid(settle_s=-1), r"'settle_s' must be >= 0"),
    (_valid(name=''), r"'name'"),
    (_valid(name='has space'), r"'name'"),
    (_valid(name=5), r"'name'"),
    (_valid(extra=1), r"unknown key\(s\) \['extra'\]"),
    ([1, 2], r"top level must be a mapping"),
    (None, r"top level must be a mapping"),
])
def test_invalid(data, pattern):
    with pytest.raises(sc.ScenarioError, match=pattern):
        sc.parse_scenario(data, source='s.yaml')


def test_error_names_source():
    with pytest.raises(sc.ScenarioError, match=r'^my\.yaml: '):
        sc.parse_scenario({'name': 'x'}, source='my.yaml')


def test_allowed_frames_override():
    s = sc.parse_scenario(_valid(frame_id='odom'),
                          allowed_frames=('map', 'odom'))
    assert s.frame_id == 'odom'


def test_load_errors(tmp_path):
    with pytest.raises(sc.ScenarioError, match='cannot read'):
        sc.load_scenario(str(tmp_path / 'nope.yaml'))
    bad = tmp_path / 'bad.yaml'
    bad.write_text('name: [unclosed\n')
    with pytest.raises(sc.ScenarioError, match='invalid YAML'):
        sc.load_scenario(str(bad))
    empty = tmp_path / 'empty.yaml'
    empty.write_text('')
    with pytest.raises(sc.ScenarioError, match='top level must be'):
        sc.load_scenario(str(empty))


def test_cli(tmp_path, capsys):
    assert sc.main([EXAMPLE]) == 0
    assert capsys.readouterr().out.strip() == 'example'
    assert sc.main([EXAMPLE, '--json']) == 0
    assert '"goals"' in capsys.readouterr().out
    bad = tmp_path / 'bad.yaml'
    bad.write_text('name: x\nframe_id: odom\ngoals: [{x: 0, y: 0, yaw: 0}]\n')
    assert sc.main([str(bad)]) == sc.EXIT_BAD_INPUT
    assert 'unknown frame_id' in capsys.readouterr().err


# ------------------------------------------------------------ naming / seeds
def test_run_name():
    assert sc.run_name('baseline', 1) == 'baseline_01'
    assert sc.run_name('active', 12) == 'active_12'
    assert sc.run_name('active', 100) == 'active_100'
    for bad in [('other', 1), ('baseline', 0), ('baseline', -1),
                ('baseline', True), ('baseline', 1.0)]:
        with pytest.raises(ValueError):
            sc.run_name(*bad)


def test_parse_run_name_roundtrip():
    for mode in sc.MODES:
        for i in (1, 9, 10, 123):
            assert sc.parse_run_name(sc.run_name(mode, i)) == (mode, i)
    for bad in ('baseline_1', 'base_01', 'active_00', 'active-01', ''):
        with pytest.raises(ValueError):
            sc.parse_run_name(bad)


def test_seeds_identical_for_both_modes():
    seeds = [sc.run_seed(42, i) for i in (1, 2, 3)]
    assert seeds == [42, 43, 44]
    with pytest.raises(ValueError):
        sc.run_seed(42, 0)


# ------------------------------------------------------------ results
def test_result_success_and_yaml(tmp_path):
    s = sc.parse_scenario(_valid())
    rr = sc.RunResult.for_scenario(s, run_name='baseline_01',
                                   mode='baseline', seed=42,
                                   start_wall='t0')
    assert [g.status for g in rr.goals] == ['skipped', 'skipped']
    assert rr.exit_code == sc.EXIT_GOAL_FAILED  # nothing run yet
    for g in rr.goals:
        g.status = 'succeeded'
        g.duration_wall_s = 10.0
        g.path_length_m = 1.25
        g.set_final_pose(g.x + 0.03, g.y - 0.04, g.yaw + 2 * math.pi + 0.1)
    assert rr.all_succeeded and rr.exit_code == sc.EXIT_OK
    assert rr.goals[0].final_xy_error_m == pytest.approx(0.05)
    assert rr.goals[0].final_yaw_error_rad == pytest.approx(0.1)

    p = tmp_path / 'r' / 'result.yaml'
    sc.write_result(str(p), rr)
    d = yaml.safe_load(p.read_text())
    assert d['run_name'] == 'baseline_01' and d['seed'] == 42
    assert d['scenario']['name'] == 'loop'
    assert d['summary'] == {
        'n_goals': 2, 'n_succeeded': 2, 'all_succeeded': True,
        'total_duration_wall_s': 20.0, 'total_path_length_m': 2.5,
        'exit_code': 0, 'error': ''}
    assert d['goals'][1]['goal'] == {'x': 1.0, 'y': 2.5, 'yaw': -1.0}
    assert d['goals'][1]['status'] == 'succeeded'
    assert not os.path.exists(str(p) + '.tmp')


def test_result_failure_codes():
    s = sc.parse_scenario(_valid())
    rr = sc.RunResult.for_scenario(s)
    rr.goals[0].status = 'succeeded'
    rr.goals[1].status = 'timeout'
    assert not rr.all_succeeded and rr.n_succeeded == 1
    assert rr.exit_code == sc.EXIT_GOAL_FAILED
    rr.exit_code_override = sc.EXIT_NOT_READY
    assert rr.to_dict()['summary']['exit_code'] == sc.EXIT_NOT_READY


def test_bad_goal_status_rejected():
    with pytest.raises(ValueError):
        sc.GoalResult(index=0, x=0, y=0, yaw=0, status='done')


# ------------------------------------------------------------ dry run
def _dry_run(mode, runs=3, seed0=7, extra=()):
    env = dict(os.environ)
    env.pop('ACTIVE_LAUNCH', None)
    return subprocess.run(
        ['bash', SCRIPT, '--mode', mode, '--runs', str(runs),
         '--scenario', EXAMPLE, '--seed0', str(seed0),
         '--out', '/nonexistent/al_runs', '--dry-run', *extra],
        capture_output=True, text=True, env=env, cwd=REPO, timeout=60)


needs_script = pytest.mark.skipif(
    shutil.which('bash') is None or not os.path.isfile(SCRIPT),
    reason='bash or scripts/run_experiment.sh not available')


@needs_script
@pytest.mark.parametrize('mode', sc.MODES)
def test_dry_run_sequence(mode):
    r = _dry_run(mode)
    assert r.returncode == 0, r.stderr + r.stdout
    out = r.stdout
    for i in (1, 2, 3):
        name = sc.run_name(mode, i)
        seed = sc.run_seed(7, i)
        assert 'seed:=%d' % seed in out
        assert 'run_name:=%s' % name in out
        assert '--run-name %s' % name in out
        assert '/nonexistent/al_runs/%s/run.yaml' % name in out
    # order inside a run: stop -> gazebo -> loc -> noise -> nav -> recorder
    # -> (active) -> scenario_runner -> recorder SIGINT -> stop
    first = out.split('=== run 2/3')[0]
    keys = ['pkill', 'world_and_robot gazebo_sim.launch.py',
            'rtabmap_landmarks complete_system.launch.py',
            'al_odom_noise odom_noise.launch.py',
            'nav2_planner_custom navigation_launch.py',
            'al_eval traj_recorder.launch.py']
    if mode == 'active':
        keys.append('ACTIVE')
    keys += ['al_eval scenario_runner', 'kill -INT $(cat']
    pos = [first.find(k) for k in keys]
    assert all(p >= 0 for p in pos), list(zip(keys, pos))
    assert pos == sorted(pos), list(zip(keys, pos))
    if mode == 'baseline':
        assert 'ACTIVE' not in first.split('al_eval scenario_runner')[0] \
            .split('al_eval traj_recorder.launch.py')[1]
    # no docker command is actually executed in dry-run
    assert not os.path.exists('/nonexistent/al_runs')


@needs_script
def test_dry_run_rejects_bad_args():
    r = _dry_run('both')
    assert r.returncode == 2
    r = _dry_run('baseline', runs=0)
    assert r.returncode == 2


@needs_script
def test_dry_run_rejects_bad_scenario(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('name: x\ngoals: []\n')
    r = subprocess.run(
        ['bash', SCRIPT, '--mode', 'baseline', '--runs', '1',
         '--scenario', str(bad), '--dry-run'],
        capture_output=True, text=True, cwd=REPO, timeout=60)
    assert r.returncode == 2
    assert 'non-empty list' in r.stderr


# ------------------------------------------------------------ node logic
# scenario_runner_node imports rclpy / nav2_msgs at module top. Here those
# modules are replaced by minimal stubs so that the goal loop, timeout and
# exit-code logic can be exercised without ROS. This checks OUR control flow
# only; the real rclpy / Nav2 behaviour is [unverified] until ENG runs it.
class _Auto:
    """Attribute bag that creates nested bags on first access."""

    def __getattr__(self, name):
        v = _Auto()
        setattr(self, name, v)
        return v


class _Future:
    def __init__(self, result=None, done=True):
        self._result, self._done = result, done

    def done(self):
        return self._done

    def result(self):
        return self._result


SUCCEEDED, CANCELED, ABORTED = 4, 5, 6


class _Handle:
    def __init__(self, accepted, status, finishes):
        self.accepted = accepted
        self._result = _Future(types.SimpleNamespace(status=status),
                               done=finishes)

    def get_result_async(self):
        return self._result

    def cancel_goal_async(self):
        # like Nav2: a cancelled goal ends with STATUS_CANCELED
        self._result._result = types.SimpleNamespace(status=CANCELED)
        self._result._done = True
        return _Future(done=True)


class _ActionClient:
    server_ok = True
    script = []      # one (accepted, status, finishes) per goal
    sent = []

    def __init__(self, node, action_type, name):
        pass

    def wait_for_server(self, timeout_sec=None):
        return _ActionClient.server_ok

    def send_goal_async(self, goal, feedback_callback=None):
        _ActionClient.sent.append(goal)
        if feedback_callback is not None:
            feedback_callback(types.SimpleNamespace(
                feedback=types.SimpleNamespace(number_of_recoveries=1)))
        accepted, status, finishes = _ActionClient.script.pop(0)
        return _Future(_Handle(accepted, status, finishes))


class _Node:
    def __init__(self, name):
        self._log = types.SimpleNamespace(
            info=lambda *a, **k: None, warn=lambda *a, **k: None,
            error=lambda *a, **k: None)

    def get_logger(self):
        return self._log

    def get_clock(self):
        ns = time.monotonic_ns()
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(
            nanoseconds=ns, to_msg=lambda: _Auto()))

    def create_subscription(self, *a, **k):
        return None

    def destroy_node(self):
        pass


class _Buffer:
    def can_transform(self, *a):
        return True

    def lookup_transform(self, *a):
        tf = _Auto()
        tf.transform.translation.x = 1.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = 0.0, 0.0, 0.0, 1.0
        return tf


class _NavigateToPose:
    Goal = _Auto


@pytest.fixture
def node_mod(monkeypatch):
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, m)
        return m

    rclpy = mod('rclpy', init=lambda args=None: None,
                try_shutdown=lambda: None,
                spin_once=lambda node, timeout_sec=None: time.sleep(0.005))
    rclpy.utilities = mod('rclpy.utilities',
                          remove_ros_args=lambda args: [
                              a for a in args if a != '--ros-args'])
    mod('rclpy.action', ActionClient=_ActionClient)
    mod('rclpy.node', Node=_Node)
    mod('rclpy.qos', HistoryPolicy=_Auto(), QoSProfile=lambda **k: None,
        ReliabilityPolicy=_Auto())
    mod('rclpy.time', Time=lambda: None)
    mod('rclpy.executors', ExternalShutdownException=RuntimeError)
    mod('action_msgs')
    mod('action_msgs.msg', GoalStatus=types.SimpleNamespace(
        STATUS_SUCCEEDED=SUCCEEDED, STATUS_CANCELED=CANCELED,
        STATUS_ABORTED=ABORTED))
    mod('nav2_msgs')
    mod('nav2_msgs.action', NavigateToPose=_NavigateToPose)
    mod('nav_msgs')
    mod('nav_msgs.msg', Odometry=object)
    mod('tf2_ros', TransformException=RuntimeError)
    mod('tf2_ros.buffer', Buffer=_Buffer)
    mod('tf2_ros.transform_listener',
        TransformListener=lambda buf, node: None)
    monkeypatch.delitem(sys.modules, 'al_eval.scenario_runner_node',
                        raising=False)
    _ActionClient.server_ok = True
    _ActionClient.script = []
    _ActionClient.sent = []
    m = importlib.import_module('al_eval.scenario_runner_node')
    yield m
    sys.modules.pop('al_eval.scenario_runner_node', None)


def _scenario_file(tmp_path, n_goals=2, timeout=0.3):
    p = tmp_path / 's.yaml'
    goals = ''.join('  - {x: %d.0, y: 0.0, yaw: 0.0}\n' % (i + 1)
                    for i in range(n_goals))
    p.write_text('name: t\nframe_id: map\ngoal_timeout_s: %s\n'
                 'settle_s: 0\ngoals:\n%s' % (timeout, goals))
    return str(p)


def _main(m, args):
    with pytest.raises(SystemExit) as e:
        m.main(['scenario_runner'] + args)
    return e.value.code


def test_node_all_succeed(node_mod, tmp_path):
    _ActionClient.script = [(True, SUCCEEDED, True)] * 2
    out = tmp_path / 'result.yaml'
    rc = _main(node_mod, ['--scenario', _scenario_file(tmp_path), '--out',
                          str(out), '--run-name', 'baseline_01', '--mode',
                          'baseline', '--seed', '42', '--ros-args'])
    assert rc == sc.EXIT_OK
    d = yaml.safe_load(out.read_text())
    assert [g['status'] for g in d['goals']] == ['succeeded', 'succeeded']
    assert d['seed'] == 42 and d['mode'] == 'baseline'
    assert d['goals'][0]['recoveries'] == 1
    # fake TF pose is (1, 0, 0): goal 0 at (1, 0) -> 0 error, goal 1 -> 1 m
    assert d['goals'][0]['final_xy_error_m'] == pytest.approx(0.0)
    assert d['goals'][1]['final_xy_error_m'] == pytest.approx(1.0)
    g = _ActionClient.sent[1]
    assert g.pose.header.frame_id == 'map' and g.pose.pose.position.x == 2.0
    assert g.pose.pose.orientation.w == pytest.approx(1.0)


def test_node_timeout_rejected_aborted(node_mod, tmp_path):
    _ActionClient.script = [(True, SUCCEEDED, False),   # never finishes
                            (False, SUCCEEDED, True),   # rejected
                            (True, ABORTED, True)]
    out = tmp_path / 'result.yaml'
    t0 = time.monotonic()
    rc = _main(node_mod, ['--scenario', _scenario_file(tmp_path, 3, 0.3),
                          '--out', str(out)])
    assert rc == sc.EXIT_GOAL_FAILED
    d = yaml.safe_load(out.read_text())
    assert [g['status'] for g in d['goals']] == \
        ['timeout', 'rejected', 'aborted']
    assert d['goals'][0]['duration_wall_s'] >= 0.3
    assert time.monotonic() - t0 < 30


def test_node_stop_on_failure(node_mod, tmp_path):
    _ActionClient.script = [(True, ABORTED, True)]
    out = tmp_path / 'result.yaml'
    rc = _main(node_mod, ['--scenario', _scenario_file(tmp_path, 2),
                          '--out', str(out), '--stop-on-failure'])
    assert rc == sc.EXIT_GOAL_FAILED
    d = yaml.safe_load(out.read_text())
    assert [g['status'] for g in d['goals']] == ['aborted', 'skipped']


def test_node_not_ready(node_mod, tmp_path):
    _ActionClient.server_ok = False
    out = tmp_path / 'result.yaml'
    rc = _main(node_mod, ['--scenario', _scenario_file(tmp_path),
                          '--out', str(out), '--server-timeout', '0.1'])
    assert rc == sc.EXIT_NOT_READY
    d = yaml.safe_load(out.read_text())
    assert 'not available' in d['summary']['error']
    assert d['summary']['exit_code'] == sc.EXIT_NOT_READY


def test_node_bad_scenario(node_mod, tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('name: x\nframe_id: odom\ngoals: [{x: 0, y: 0, yaw: 0}]\n')
    out = tmp_path / 'result.yaml'
    assert _main(node_mod, ['--scenario', str(bad), '--out', str(out)]) \
        == sc.EXIT_BAD_INPUT
    assert not out.exists()
