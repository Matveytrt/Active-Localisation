"""Tests for al_eval.recorder_helpers and the secondary recordings of
traj_recorder_node (landmarks.csv, map_odom.tum) with stubbed ROS modules.

The node tests check OUR control flow only (lazy import, dedup, counters,
files); the real rclpy / rtabmap_msgs behaviour is [unverified] until ENG
runs the recorder on the testbed.
"""

import importlib
import os
import sys
import types

import pytest
import yaml

# allow `pytest navigation/src/al_eval/test` from anywhere (no colcon)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from al_eval.recorder_helpers import (  # noqa: E402
    LANDMARKS_CSV_HEADER, LandmarkCsvWriter, StampDeduper, detections_stamp,
    format_landmark_row, parse_landmark_row, read_landmarks_csv)
from al_eval.tum import read_tum  # noqa: E402


# ------------------------------------------------------------ pure helpers
def test_detections_stamp_priority():
    # array header wins when set
    assert detections_stamp((5, 1), [(7, 0)], (9, 0)) == ((5, 1), 'header')
    # tf_to_fiducial leaves the array header empty: newest landmark stamp
    assert detections_stamp((0, 0), [(7, 5), (7, 9), (0, 0)], (9, 0)) == (
        (7, 9), 'landmark')
    assert detections_stamp(None, [(3, 0)], (9, 0)) == ((3, 0), 'landmark')
    # nothing usable: receipt time
    assert detections_stamp((0, 0), [(0, 0)], (9, 2)) == ((9, 2), 'receipt')
    assert detections_stamp(None, [], (9, 2)) == ((9, 2), 'receipt')


def test_row_format_roundtrip():
    line = format_landmark_row('12.000000005', [3, 1])
    assert line == '12.000000005,2,3;1'
    assert parse_landmark_row(line) == (pytest.approx(12.000000005), 2,
                                        (3, 1))
    assert format_landmark_row('1.0', []) == '1.0,0,'
    assert parse_landmark_row('1.0,0,') == (1.0, 0, ())
    with pytest.raises(ValueError):
        parse_landmark_row('1.0,2,1')        # n does not match ids
    with pytest.raises(ValueError):
        parse_landmark_row('1.0,2')


def test_writer_reader(tmp_path):
    p = str(tmp_path / 'sub' / 'landmarks.csv')
    with LandmarkCsvWriter(p) as w:
        w.write(100.5, [1, 2])
        w.write_stamp(101, 7, [5])
        assert len(read_landmarks_csv(p)) == 2   # flushed before close
    assert w.closed and w.count == 2 and w.detections == 3
    w.close()  # idempotent
    with pytest.raises(ValueError):
        w.write(1.0, [1])
    with open(p) as f:
        assert f.readline().strip() == LANDMARKS_CSV_HEADER
    rows = read_landmarks_csv(p)
    assert rows[0] == (100.5, 2, (1, 2))
    assert rows[1][0] == pytest.approx(101.000000007, abs=1e-9)
    assert rows[1][1:] == (1, (5,))


def test_reader_truncated_tail_only(tmp_path):
    p = tmp_path / 'l.csv'
    p.write_text(LANDMARKS_CSV_HEADER + '\n1.0,1,3\n2.0,2,3;')
    assert read_landmarks_csv(str(p)) == [(1.0, 1, (3,))]
    p.write_text(LANDMARKS_CSV_HEADER + '\n1.0,1,3;4\n2.0,1,3\n')
    with pytest.raises(ValueError):
        read_landmarks_csv(str(p))


def test_deduper():
    d = StampDeduper()
    assert d.accept((1, 0))
    assert not d.accept((1, 0))
    assert d.accept((1, 1))
    assert d.accept((1, 0))   # only consecutive repeats are dropped
    assert d.skipped == 1


# ------------------------------------------------------------ node (stubs)
class _Auto:
    """Attribute bag that creates nested bags on first access."""

    def __getattr__(self, name):
        v = _Auto()
        setattr(self, name, v)
        return v


def _stamp(sec, nsec=0):
    return types.SimpleNamespace(sec=sec, nanosec=nsec)


class _Node:
    params = {}

    def __init__(self, name):
        self.subs = {}
        self.warnings = []
        self._log = types.SimpleNamespace(
            info=lambda *a, **k: None,
            warn=lambda msg, **k: self.warnings.append(msg),
            error=lambda *a, **k: None)
        self._declared = {}

    def declare_parameter(self, name, default):
        self._declared[name] = _Node.params.get(name, default)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._declared[name])

    def get_logger(self):
        return self._log

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(
            to_msg=lambda: _stamp(50, 0)))

    def create_subscription(self, msg_type, topic, cb, qos):
        self.subs[topic] = (msg_type, cb)

    def create_timer(self, period, cb):
        return None

    def destroy_node(self):
        pass


class _Buffer:
    """TF stub: frames -> list of (stamp, x, y) returned one per lookup."""

    script = {}

    def lookup_transform(self, target, source, t):
        seq = _Buffer.script.get((target, source))
        if not seq:
            raise RuntimeError('no tf %s->%s' % (target, source))
        st, x, y = seq.pop(0) if len(seq) > 1 else seq[0]
        tf = _Auto()
        tf.header.stamp = st
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = 0.0, 0.0, 0.0, 1.0
        return tf


class _LandmarkDetections:
    pass


def _make_mod(monkeypatch, name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    monkeypatch.setitem(sys.modules, name, m)
    return m


@pytest.fixture
def rec_mod(monkeypatch):
    mod = lambda name, **a: _make_mod(monkeypatch, name, **a)  # noqa: E731
    mod('rclpy', init=lambda args=None: None, try_shutdown=lambda: None,
        spin=lambda node: None)
    mod('rclpy.node', Node=_Node)
    mod('rclpy.qos', HistoryPolicy=_Auto(), QoSProfile=lambda **k: k,
        ReliabilityPolicy=_Auto())
    mod('rclpy.time', Time=lambda: None)
    mod('rclpy.executors', ExternalShutdownException=RuntimeError)
    mod('nav_msgs')
    mod('nav_msgs.msg', Odometry=object)
    mod('tf2_ros', TransformException=RuntimeError)
    mod('tf2_ros.buffer', Buffer=_Buffer)
    mod('tf2_ros.transform_listener', TransformListener=lambda b, n: None)
    mod('rtabmap_msgs')
    mod('rtabmap_msgs.msg', LandmarkDetections=_LandmarkDetections)
    monkeypatch.delitem(sys.modules, 'al_eval.traj_recorder_node',
                        raising=False)
    _Buffer.script = {}
    m = importlib.import_module('al_eval.traj_recorder_node')
    yield m
    sys.modules.pop('al_eval.traj_recorder_node', None)


def _det(sec, ids, header_sec=0):
    lms = [types.SimpleNamespace(id=i, header=types.SimpleNamespace(
        stamp=_stamp(sec, 1000 * k))) for k, i in enumerate(ids)]
    return types.SimpleNamespace(header=types.SimpleNamespace(
        stamp=_stamp(header_sec)), landmarks=lms)


def test_node_records_landmarks_and_map_odom(rec_mod, tmp_path):
    _Node.params = {'output_dir': str(tmp_path), 'run_name': 'active_01'}
    _Buffer.script = {
        ('map', 'odom'): [(_stamp(10), 0.0, 0.0), (_stamp(10), 0.0, 0.0),
                          (_stamp(11), 0.1, 0.0)],
        ('map', 'base_link'): [(_stamp(10), 1.0, 0.0)],
    }
    node = rec_mod.TrajRecorder()
    msg_type, cb = node.subs['/landmark_detections']
    assert msg_type is _LandmarkDetections
    cb(_det(20, [1, 3]))
    cb(_det(21, [2], header_sec=22))
    for _ in range(3):
        node._on_timer()
    node.finalize()

    run = tmp_path / 'active_01'
    rows = read_landmarks_csv(str(run / 'landmarks.csv'))
    assert [r[1:] for r in rows] == [(2, (1, 3)), (1, (2,))]
    assert rows[0][0] == pytest.approx(20.000001)   # newest landmark stamp
    assert rows[1][0] == 22.0                        # array header stamp
    mo = read_tum(str(run / 'map_odom.tum'))
    assert [r[0] for r in mo] == [10.0, 11.0]        # duplicate dropped
    meta = yaml.safe_load((run / 'meta.yaml').read_text())
    c = meta['counters']
    assert c['map_odom_written'] == 2
    assert c['map_odom_skipped_duplicate_stamp'] == 1
    assert c['landmark_msgs_written'] == 2
    assert c['landmark_detections_total'] == 3
    assert meta['landmark_stamp_sources'] == {'landmark': 1, 'header': 1}
    assert meta['params']['record_landmarks'] is True
    assert meta['secondary_disabled'] == {}


def test_node_without_rtabmap_msgs_warns(rec_mod, tmp_path, monkeypatch):
    # Simulate rtabmap_msgs not importable: None in sys.modules -> ImportError
    monkeypatch.setitem(sys.modules, 'rtabmap_msgs.msg', None)
    _Node.params = {'output_dir': str(tmp_path), 'run_name': 'baseline_01',
                    'record_map_odom': False}
    node = rec_mod.TrajRecorder()
    assert '/landmark_detections' not in node.subs
    assert any('landmark recording disabled' in w for w in node.warnings)
    node._on_timer()   # map_odom off: no lookups of map->odom needed
    node.finalize()
    run = tmp_path / 'baseline_01'
    assert not (run / 'landmarks.csv').exists()
    assert not (run / 'map_odom.tum').exists()
    meta = yaml.safe_load((run / 'meta.yaml').read_text())
    assert 'rtabmap_msgs' in meta['secondary_disabled']['landmarks']
    assert meta['secondary_disabled']['map_odom'] == 'record_map_odom:=false'


def test_node_map_odom_unavailable_counted(rec_mod, tmp_path):
    _Node.params = {'output_dir': str(tmp_path), 'run_name': 'control_01',
                    'record_landmarks': False}
    node = rec_mod.TrajRecorder()
    node._on_timer()
    node._on_timer()
    node.finalize()
    meta = yaml.safe_load(
        (tmp_path / 'control_01' / 'meta.yaml').read_text())
    assert meta['counters']['map_odom_skipped_tf_unavailable'] == 2
    assert 'map->odom' in meta['first_map_odom_error']
    assert meta['secondary_disabled'] == {
        'landmarks': 'record_landmarks:=false'}


def test_node_refuses_existing_secondary_files(rec_mod, tmp_path):
    run = tmp_path / 'active_02'
    run.mkdir()
    (run / 'landmarks.csv').write_text(LANDMARKS_CSV_HEADER + '\n')
    _Node.params = {'output_dir': str(tmp_path), 'run_name': 'active_02'}
    with pytest.raises(RuntimeError, match='already has recordings'):
        rec_mod.TrajRecorder()
