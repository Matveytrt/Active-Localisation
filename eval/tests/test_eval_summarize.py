"""Tests for eval/summarize.py and the experiment case of make_synthetic.py
(pure Python; evo is mocked, see _evo_mock.py)."""

import csv
import math
import os
import shutil
import subprocess
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
EVAL = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, EVAL)

import make_synthetic as ms  # noqa: E402
import summarize as sm  # noqa: E402
from _evo_mock import mock_run_evo  # noqa: E402


# ------------------------------------------------------------ pure metrics
def test_coverage_fraction():
    # no stamps -> 0; one stamp covers [s, s + 1)
    assert sm.coverage_fraction([], 0.0, 10.0) == 0.0
    assert sm.coverage_fraction([2.0], 0.0, 10.0) == pytest.approx(0.1)
    # overlapping intervals are merged, not double counted
    assert sm.coverage_fraction([2.0, 2.5], 0.0, 10.0) == pytest.approx(0.15)
    # a stamp just before t0 still covers the start; one at t1 covers nothing
    assert sm.coverage_fraction([-0.5, 10.0], 0.0, 10.0) == pytest.approx(
        0.05)
    # dense 10 Hz stamps -> full coverage from the first stamp on
    st = [0.1 * k for k in range(100)]
    assert sm.coverage_fraction(st, 0.0, 10.0) == pytest.approx(1.0)
    assert sm.coverage_fraction([1.0], 0.0, 10.0, window=2.0) == (
        pytest.approx(0.2))
    assert math.isnan(sm.coverage_fraction([1.0], 5.0, 5.0))
    with pytest.raises(ValueError):
        sm.coverage_fraction([1.0], 0.0, 1.0, window=0.0)


def test_detection_stats():
    rows = [(1.0, 2, (1, 2)), (1.1, 1, (1,)), (5.0, 0, ())]
    d = sm.detection_stats(rows, 0.0, 10.0, path_m=3.0)
    assert d['det_total'] == 3 and d['det_msgs'] == 3
    assert d['det_per_m'] == pytest.approx(1.0)
    # n=0 rows do not count as coverage: [1.0, 2.1) of 10 s
    assert d['det_coverage_pct'] == pytest.approx(11.0)
    assert math.isnan(sm.detection_stats(rows, 0, 10, 0.0)['det_per_m'])


def test_map_odom_stats():
    poses = [(-2.0, -0.5, 0.0), (-2.0, -0.5, 0.0), (-1.9, -0.5, 0.02),
             (-2.05, -0.5, math.pi - 0.01), (-2.0, -0.5, -math.pi + 0.01)]
    s = sm.map_odom_stats(poses)          # w.r.t. the first sample
    assert s['mo_n'] == 5
    assert s['mo_max_step_m'] == pytest.approx(0.15)
    # pi - 0.01 -> -pi + 0.01 is a 0.02 rad step, not 2 pi - 0.02
    assert s['mo_max_step_yaw_deg'] == pytest.approx(
        math.degrees(math.pi - 0.03))
    assert s['mo_max_corr_m'] == pytest.approx(0.1)
    assert s['mo_max_corr_yaw_deg'] == pytest.approx(
        math.degrees(math.pi - 0.01))
    z = sm.map_odom_stats(poses, reference='zero')
    assert z['mo_max_corr_m'] == pytest.approx(math.hypot(2.05, 0.5))
    one = sm.map_odom_stats([(1.0, 0.0, 0.0)])
    assert one['mo_max_step_m'] == 0.0 and one['mo_max_corr_m'] == 0.0
    assert math.isnan(sm.map_odom_stats([])['mo_max_step_m'])
    with pytest.raises(ValueError):
        sm.map_odom_stats(poses, reference='bad')


def test_result_and_stats_over_runs():
    r = {'summary': {'n_goals': 3, 'n_succeeded': 2},
         'goals': [{'duration_sim_s': 1.5}, {'duration_sim_s': 2.0},
                   {'duration_sim_s': 0.5}]}
    assert sm.result_stats(r) == {'goals_succeeded': 2.0, 'goals_total': 3,
                                  'duration_s': 4.0}
    r['goals'][1]['duration_sim_s'] = None
    assert math.isnan(sm.result_stats(r)['duration_s'])
    s = sm.stats_over_runs([1.0, 2.0, float('nan'), 3.0])
    assert (s['mean'], s['std'], s['min'], s['max'], s['n']) == (
        2.0, 1.0, 1.0, 3.0, 3)
    assert math.isnan(sm.stats_over_runs([4.0])['std'])
    assert sm.stats_over_runs([])['n'] == 0


def test_mode_detection(tmp_path):
    a = tmp_path / 'active_01'
    a.mkdir()
    assert sm.run_mode(str(a)) == ('active', 'dir name')
    (a / 'run.yaml').write_text('mode: control\n')
    assert sm.run_mode(str(a)) == ('control', 'run.yaml')
    b = tmp_path / 'weird'
    b.mkdir()
    assert sm.run_mode(str(b)) == ('unknown', 'unknown')
    assert sm.mode_from_name('baseline_12') == 'baseline'
    assert sm.mode_from_name('baseline') is None
    assert sorted(['control', 'x', 'active', 'baseline'],
                  key=sm.mode_sort_key) == ['baseline', 'active', 'control',
                                            'x']


# ------------------------------------------------------------ synthetic data
@pytest.fixture(scope='module')
def experiment(tmp_path_factory):
    root = str(tmp_path_factory.mktemp('exp'))
    ms.write_experiment(root, runs=3, seed=0)
    for d in sm.find_runs(root):
        mock_run_evo(d)
    return root


def test_make_synthetic_experiment_files(experiment):
    runs = sm.find_runs(experiment)
    assert [os.path.basename(r) for r in runs] == sorted(
        '%s_%02d' % (m, i) for m in ms.EXP_MODES for i in (1, 2, 3))
    for r in runs:
        for f in ('gt.tum', 'est.tum', 'landmarks.csv', 'map_odom.tum',
                  'result.yaml', 'run.yaml'):
            assert os.path.getsize(os.path.join(r, f)) > 0, (r, f)
    ry = yaml.safe_load(open(os.path.join(experiment, 'control_02',
                                          'run.yaml')))
    assert ry['mode'] == 'control' and ry['run_index'] == 2
    res = yaml.safe_load(open(os.path.join(experiment, 'baseline_03',
                                           'result.yaml')))
    assert res['summary']['n_succeeded'] == 3
    assert res['summary']['exit_code'] == 3


def test_make_synthetic_cli_keeps_old_cases(tmp_path):
    r = subprocess.run([sys.executable, os.path.join(EVAL, 'make_synthetic.py'),
                        str(tmp_path)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert sorted(os.listdir(tmp_path)) == ['drift', 'noise', 'shift']
    assert sorted(os.listdir(tmp_path / 'shift')) == [
        'est.tum', 'gt.tum', 'synthetic.txt']


def test_load_run_values(experiment):
    row = sm.load_run(os.path.join(experiment, 'active_01'))
    assert row['mode'] == 'active' and row['mode_source'] == 'run.yaml'
    assert row['notes'] == ''
    assert row['duration_s'] == pytest.approx(66.0)
    assert row['duration_source'] == 'result.yaml'
    assert row['path_length_m'] == pytest.approx(13.58, abs=0.02)
    assert row['goals_succeeded'] == 4 and row['goals_total'] == 4
    assert row['det_per_m'] == pytest.approx(row['det_total']
                                             / row['path_length_m'])
    assert 0.0 < row['det_coverage_pct'] < 100.0
    assert 0.0 < row['ape_rmse'] < 0.1
    assert row['align'] == 'not aligned'
    assert 0.0 < row['mo_max_step_m'] < row['mo_max_corr_m'] < 1.0


def test_load_run_missing_inputs(tmp_path, experiment):
    d = tmp_path / 'baseline_07'
    shutil.copytree(os.path.join(experiment, 'baseline_01'), d)
    for f in ('result.yaml', 'landmarks.csv', 'map_odom.tum', 'run.yaml'):
        os.remove(d / f)
    shutil.rmtree(d / 'evo_none')
    row = sm.load_run(str(d))
    assert row['mode'] == 'baseline' and row['mode_source'] == 'dir name'
    assert row['duration_source'] == 'gt.tum span'
    assert row['duration_s'] == pytest.approx(60.0)
    for k in ('ape_rmse', 'det_total', 'mo_max_corr_m', 'goals_succeeded'):
        assert math.isnan(row[k]), k
    for n in ('evo_none', 'result.yaml', 'landmarks.csv', 'map_odom.tum'):
        assert n in row['notes']


def test_summary_outputs(experiment, tmp_path):
    out = tmp_path / 'summary'
    assert sm.main([experiment, '--out', str(out)]) == 0
    for f in ('summary.csv', 'summary_modes.csv', 'summary.md',
              'summary_table.tex'):
        assert (out / f).stat().st_size > 0
    with open(out / 'summary.csv') as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 9
    assert [r['mode'] for r in rows][::3] == ['baseline', 'active', 'control']
    with open(out / 'summary_modes.csv') as f:
        modes = {r['mode']: r for r in csv.DictReader(f)}
    assert set(modes) == {'baseline', 'active', 'control'}
    # the synthetic active mode sees more landmarks and has a lower APE
    assert float(modes['active']['det_per_m_mean']) > 2 * float(
        modes['baseline']['det_per_m_mean'])
    assert float(modes['active']['ape_rmse_mean']) < float(
        modes['baseline']['ape_rmse_mean'])
    for m in modes.values():
        assert float(m['ape_rmse_min']) <= float(m['ape_rmse_mean']) <= float(
            m['ape_rmse_max'])
    md = (out / 'summary.md').read_text()
    assert '| goals reached (sum) | 11/12 | 12/12 | 12/12 |' in md
    assert 'max \\|correction\\|' in md
    tex = (out / 'summary_table.tex').read_text()
    assert '\\toprule' in tex and '\\bottomrule' in tex
    assert 'baseline ($n{=}3$)' in tex and '\\pm' in tex
    body = tex.split('\\midrule')[1].split('\\bottomrule')[0]
    for line in body.strip().splitlines():
        assert line.count('&') == 3 and line.endswith('\\\\'), line


def test_summary_rejects_mixed_alignment(experiment, tmp_path):
    root = tmp_path / 'mixed'
    for name in ('baseline_01', 'active_01'):
        shutil.copytree(os.path.join(experiment, name), root / name)
    mock_run_evo(str(root / 'active_01'), align='none')
    # rewrite one result with another alignment title
    import _evo_mock
    d = str(root / 'baseline_01')
    os.rename(os.path.join(d, 'evo_none'), os.path.join(d, 'evo_tmp'))
    _evo_mock.mock_run_evo(d, align='se3')
    os.rename(os.path.join(d, 'evo_se3'), os.path.join(d, 'evo_none'))
    assert sm.main([str(root), '--out', str(tmp_path / 'o')]) == 2
    assert sm.main([str(root), '--out', str(tmp_path / 'o'),
                    '--allow_mixed_alignment']) == 0


def test_summary_cli_errors(tmp_path):
    with pytest.raises(SystemExit):
        sm.main([])
    assert sm.main([str(tmp_path)]) == 2   # no run dirs
