"""Tests for eval/crop_run.py (evaluation window, pure Python)."""

import os
import subprocess
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
EVAL = os.path.dirname(HERE)
sys.path.insert(0, EVAL)

import crop_run as cw  # noqa: E402
import make_synthetic as ms  # noqa: E402
from al_eval.recorder_helpers import read_landmarks_csv  # noqa: E402
from al_eval.tum import read_tum, write_tum  # noqa: E402


def _goals(*pairs):
    return {'goals': [{'start_sim_s': a, 'end_sim_s': b} for a, b in pairs]}


def test_window_from_result():
    assert cw.window_from_result(_goals((10, 20), (20, 35))) == (
        (10.0, 35.0), '')
    # a skipped goal (no stamps) does not shrink or break the window
    assert cw.window_from_result(_goals((10, 20), (None, None)))[0] == (
        10.0, 20.0)
    win, why = cw.window_from_result({'goals': [{'duration_sim_s': 3}]})
    assert win is None and 'start_sim_s' in why
    assert cw.window_from_result({})[0] is None
    assert cw.window_from_result(None)[0] is None
    win, why = cw.window_from_result(_goals((30, 20)))
    assert win is None and '<=' in why
    # bools / strings are not stamps
    assert cw.window_from_result(_goals((True, 'x')))[0] is None


def test_crop_rows_and_names():
    rows = [(float(t), 0) for t in range(10)]
    assert [r[0] for r in cw.crop_rows(rows, 3, 5)] == [3, 4, 5]
    assert [r[0] for r in cw.crop_rows(rows, 3, 5, pad=1)] == [2, 3, 4, 5, 6]
    assert cw.crop_rows([], 0, 1) == []
    assert cw.eval_name('gt.tum') == 'gt_eval.tum'
    assert cw.eval_name('landmarks.csv') == 'landmarks_eval.csv'
    assert cw.eval_name('gt_offset') == 'gt_offset_eval'


def _gt(t0, t1, hz=10):
    n = int(round((t1 - t0) * hz))
    return [(t0 + i / hz, 0.1 * i, 0, 0, 0, 0, 0, 1) for i in range(n + 1)]


def test_resolve_window(tmp_path):
    gt = _gt(90, 130)
    d = str(tmp_path)
    t0, t1, src, note = cw.resolve_window(d, gt)
    assert (t0, t1, src) == (90, 130, 'full range')
    assert 'no result.yaml' in note
    (tmp_path / 'result.yaml').write_text(yaml.safe_dump(
        _goals((100, 110), (110, 120))))
    assert cw.resolve_window(d, gt) == (100, 120, 'result.yaml', '')
    manual = cw.resolve_window(d, gt, manual=(95, 99))
    assert manual[:3] == (95, 99, 'manual')
    with pytest.raises(ValueError):
        cw.resolve_window(d, gt, manual=(99, 95))
    # wall-clock stamps in result.yaml vs sim-time recording: no overlap
    (tmp_path / 'result.yaml').write_text(yaml.safe_dump(
        _goals((1.7e9, 1.7e9 + 30))))
    t0, t1, src, note = cw.resolve_window(d, gt)
    assert (t0, t1, src) == (90, 130, 'full range')
    assert 'clock mismatch' in note
    (tmp_path / 'result.yaml').write_text(': not yaml : [')
    assert 'cannot read result.yaml' in cw.resolve_window(d, gt)[3]


def test_crop_run_synthetic(tmp_path):
    d = str(tmp_path / 'active_01')
    ms.write_experiment_run(d, 'active', 1, seed=0)
    info = cw.crop_run(d)
    assert (info['t0'], info['t1'], info['source']) == (100.0, 166.0,
                                                        'result.yaml')
    assert info['recording'] == {'t0': 96.0, 't1': 168.0}
    gt = read_tum(os.path.join(d, 'gt_eval.tum'))
    est = read_tum(os.path.join(d, 'est_eval.tum'))
    mo = read_tum(os.path.join(d, 'map_odom_eval.tum'))
    # GT padded by 0.1 s on both sides, the rest strictly inside
    assert gt[0][0] == pytest.approx(99.9) and gt[-1][0] == pytest.approx(
        166.1)
    assert all(100.0 <= r[0] <= 166.0 for r in est + mo)
    assert len(est) == 660 and len(mo) == 660
    lm = read_landmarks_csv(os.path.join(d, 'landmarks_eval.csv'))
    assert lm == read_landmarks_csv(os.path.join(d, 'landmarks.csv'))
    saved = yaml.safe_load(open(os.path.join(d, cw.WINDOW_FILE)))
    assert saved['files']['est.tum'] == {'out': 'est_eval.tum',
                                         'rows_in': 720, 'rows_out': 660}
    assert saved['source'] == 'result.yaml' and saved['note'] == ''


def test_crop_run_landmarks_and_missing_files(tmp_path):
    d = tmp_path / 'r'
    d.mkdir()
    write_tum(str(d / 'gt.tum'), _gt(0, 10))
    (d / 'result.yaml').write_text(yaml.safe_dump(_goals((2, 4), (4, 6))))
    (d / 'landmarks.csv').write_text(
        'stamp,n_detections,ids\n1.0,1,3\n2.5,2,1;2\n6.5,0,\n')
    info = cw.crop_run(str(d), gt_pad=0.0)
    assert set(info['files']) == {'gt.tum', 'landmarks.csv'}
    assert read_landmarks_csv(str(d / 'landmarks_eval.csv')) == [
        (2.5, 2, (1, 2))]
    gt = read_tum(str(d / 'gt_eval.tum'))
    assert gt[0][0] == 2.0 and gt[-1][0] == 6.0
    with pytest.raises(FileNotFoundError):
        cw.crop_run(str(tmp_path / 'nothing'))


def test_crop_run_cli(tmp_path):
    ms.write_run(str(tmp_path / 'shift'), 'shift', 0.0, 0, 0.1, 0.0)
    r = subprocess.run([sys.executable, os.path.join(EVAL, 'crop_run.py'),
                        str(tmp_path / 'shift')], capture_output=True,
                       text=True)
    assert r.returncode == 0, r.stderr
    assert 'WARNING' in r.stderr and 'full GT range used' in r.stderr
    # full-range fallback: the est file is unchanged
    assert read_tum(str(tmp_path / 'shift' / 'est_eval.tum')) == read_tum(
        str(tmp_path / 'shift' / 'est.tum'))
    r = subprocess.run([sys.executable, os.path.join(EVAL, 'crop_run.py'),
                        str(tmp_path / 'shift'), '--window', '110', '120'],
                       capture_output=True, text=True)
    assert r.returncode == 0 and 'manual' in r.stdout
    assert len(read_tum(str(tmp_path / 'shift' / 'est_eval.tum'))) == 100
    r = subprocess.run([sys.executable, os.path.join(EVAL, 'crop_run.py'),
                        str(tmp_path / 'none')], capture_output=True,
                       text=True)
    assert r.returncode == 2 and 'ERROR' in r.stderr
