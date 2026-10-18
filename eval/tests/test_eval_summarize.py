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
    s = sm.map_odom_stats(poses, reference='first')
    assert s['mo_n'] == 5
    assert s['mo_max_step_m'] == pytest.approx(0.15)
    # pi - 0.01 -> -pi + 0.01 is a 0.02 rad step, not 2 pi - 0.02
    assert s['mo_max_step_yaw_deg'] == pytest.approx(
        math.degrees(math.pi - 0.03))
    assert s['mo_max_corr_m'] == pytest.approx(0.1)
    assert s['mo_max_corr_yaw_deg'] == pytest.approx(
        math.degrees(math.pi - 0.01))
    z = sm.map_odom_stats(poses)          # default: w.r.t. the identity
    assert z['mo_max_corr_m'] == pytest.approx(math.hypot(2.05, 0.5))
    assert z == sm.map_odom_stats(poses, reference='zero')
    one = sm.map_odom_stats([(1.0, 0.0, 0.0)], reference='first')
    assert one['mo_max_step_m'] == 0.0 and one['mo_max_corr_m'] == 0.0
    assert sm.map_odom_stats([(1.0, 0.0, 0.0)])['mo_max_corr_m'] == 1.0
    assert math.isnan(sm.map_odom_stats([])['mo_max_step_m'])
    with pytest.raises(ValueError):
        sm.map_odom_stats(poses, reference='bad')


def test_max_gap_rotation_total_init_check():
    assert sm.max_gap([2.0, 3.0, 7.5], 0.0, 10.0) == pytest.approx(4.5)
    assert sm.max_gap([], 0.0, 10.0) == pytest.approx(10.0)
    # stamps outside [t0, t1] are ignored; edges count
    assert sm.max_gap([-1.0, 9.0, 11.0], 0.0, 10.0) == pytest.approx(9.0)
    assert math.isnan(sm.max_gap([1.0], 5.0, 5.0))
    d = sm.detection_stats([(2.0, 1, (1,)), (3.0, 0, ())], 0.0, 10.0, 1.0)
    assert d['det_max_gap_s'] == pytest.approx(8.0)   # n=0 is no detection
    # wrapped increments: +170 deg -> -170 deg is a 20 deg turn
    yaws = [0.0, 0.5, math.radians(170), math.radians(-170), 0.0]
    assert sm.rotation_total(yaws) == pytest.approx(
        0.5 + (math.radians(170) - 0.5) + math.radians(20)
        + math.radians(170))
    assert sm.rotation_total([1.0]) == 0.0
    ok = sm.init_check((0.03, 0.03, math.radians(1.5)))
    assert ok['sanity_flag'] == 'ok'
    assert ok['mo_init_m'] == pytest.approx(math.hypot(0.03, 0.03))
    assert sm.init_check((0.04, 0.04, 0.0))['sanity_flag'] == 'bad_db_init'
    assert sm.init_check((0.0, 0.0, math.radians(2.5)))['sanity_flag'] == (
        'bad_db_init')
    bad = sm.init_check((-1.0, 0.0, -math.radians(157)))
    assert bad['mo_init_yaw_deg'] == pytest.approx(157.0)
    assert sm.init_check(None)['sanity_flag'] == ''
    assert sm.init_check((0.1, 0, 0), tol_m=0.2)['sanity_flag'] == 'ok'


def test_paired_rows():
    def row(run, mode, seed, ape, flag='ok'):
        r = {k: float('nan') for k in sm.METRIC_KEYS}
        r.update(run=run, mode=mode, seed=seed, ape_rmse=ape,
                 sanity_flag=flag)
        return r
    rows = [row('baseline_01', 'baseline', 42, 0.10),
            row('baseline_02', 'baseline', 43, 0.12),
            row('active_01', 'active', 42, 0.07),
            row('active_02', 'active', 43, 0.13),
            row('control_01', 'control', 42, 0.11, 'bad_db_init'),
            row('control_09', 'control', 99, 0.50)]
    prs, others = sm.paired_rows(rows)
    assert others == ['active', 'control']
    assert [p['pair'] for p in prs] == [42, 43, 99]
    p42 = prs[0]
    assert p42['run_active'] == 'active_01'
    assert p42['d_active_ape_rmse'] == pytest.approx(-0.03)
    assert p42['d_control_ape_rmse'] == pytest.approx(0.01)
    assert p42['flagged'] == 'control_01'
    assert math.isnan(prs[1]['d_control_ape_rmse'])     # no control seed 43
    assert math.isnan(prs[2]['d_control_ape_rmse'])     # no baseline seed 99
    ps = sm.paired_summary(prs, others, 'ape_rmse', 'active')
    assert ps['n'] == 2 and ps['mean'] == pytest.approx(-0.01)
    assert ps['consistent'] == 1
    # without seeds the run index pairs the runs
    for r in rows:
        r['seed'] = ''
    assert [p['pair'] for p in sm.paired_rows(rows)[0]] == [
        'run01', 'run02', 'run09']


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
    res = yaml.safe_load(open(os.path.join(experiment, 'active_01',
                                           'result.yaml')))
    assert [g['start_sim_s'] for g in res['goals']] == [100.0, 116.5, 133.0,
                                                        149.5]
    assert res['goals'][-1]['end_sim_s'] == 166.0
    # the recording is longer than the goal window (pre/post-roll)
    from al_eval.tum import read_tum
    gt = read_tum(os.path.join(experiment, 'active_01', 'gt.tum'))
    assert gt[0][0] == pytest.approx(96.0) and gt[-1][0] == pytest.approx(
        168.0)
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
    assert 0.0 < row['det_max_gap_s'] < 66.0
    assert 0.0 < row['ape_rmse'] < 0.1
    assert 0.0 < row['ape_max'] < 0.15          # init transient cropped
    assert 0.0 < row['ape_rot_rmse'] < 5.0 and row['ape_rot_max'] > 0
    assert row['rpe_rot_rmse'] > 0 and row['rpe_rot_max'] > 0
    assert row['align'] == 'not aligned'
    assert 0.0 < row['mo_max_step_m'] < row['mo_max_corr_m'] < 1.0
    # one lap of the ellipse: total heading change 2 pi
    assert row['rot_total_rad'] == pytest.approx(2 * math.pi, abs=0.01)
    assert row['mean_speed_mps'] == pytest.approx(13.58 / 66.0, abs=0.001)
    assert row['window_source'] == 'result.yaml'
    assert (row['window_t0'], row['window_t1']) == (100.0, 166.0)
    assert row['seed'] == 42 and row['sanity_flag'] == 'ok'
    assert row['mo_init_m'] == 0.0


def test_load_run_crop_vs_full(tmp_path, experiment):
    d = tmp_path / 'control_03'
    shutil.copytree(os.path.join(experiment, 'control_03'), d)
    mock_run_evo(str(d), crop=False)
    cropped = sm.load_run(str(d))
    full = sm.load_run(str(d), crop=False)
    # the 0.3 m localization-init transient is only in the full recording
    assert full['ape_max'] > 0.25 > 0.15 > cropped['ape_max']
    assert full['window_source'].startswith('off')
    assert full['path_length_m'] == pytest.approx(cropped['path_length_m'])
    # bad DB init: flagged from the first sample of the whole recording,
    # but the in-window correction is not polluted by it
    for r in (cropped, full):
        assert r['sanity_flag'] == 'bad_db_init'
        assert r['mo_init_m'] == pytest.approx(1.0)
        assert r['mo_init_yaw_deg'] == pytest.approx(157.0)
        assert 'stale DB init' in r['notes']
    assert cropped['mo_max_corr_m'] < 0.6 < full['mo_max_corr_m']
    first = sm.load_run(str(d), map_odom_ref='first')
    assert first['mo_max_corr_m'] != cropped['mo_max_corr_m']


def test_load_run_evo_window_mismatch(tmp_path, experiment):
    d = tmp_path / 'active_01'
    shutil.copytree(os.path.join(experiment, 'active_01'), d)
    os.remove(d / 'evo_none' / 'eval_window.yaml')
    assert 'no eval_window.yaml' in sm.load_run(str(d))['notes']
    (d / 'evo_none' / 'eval_window.yaml').write_text('t0: 1.0\nt1: 2.0\n')
    assert 'window [1.0, 2.0] != [100.000, 166.000]' in sm.load_run(
        str(d))['notes']
    # results of the old run_evo.sh (no rotation zips): noted, NaN
    os.remove(d / 'evo_none' / 'ape_rot.zip')
    row = sm.load_run(str(d))
    assert math.isnan(row['ape_rot_rmse']) and 'ape_rot.zip' in row['notes']
    assert not math.isnan(row['rpe_rot_rmse'])


def test_load_run_missing_inputs(tmp_path, experiment):
    d = tmp_path / 'baseline_07'
    shutil.copytree(os.path.join(experiment, 'baseline_01'), d)
    for f in ('result.yaml', 'landmarks.csv', 'map_odom.tum', 'run.yaml'):
        os.remove(d / f)
    shutil.rmtree(d / 'evo_none')
    row = sm.load_run(str(d))
    assert row['mode'] == 'baseline' and row['mode_source'] == 'dir name'
    # no result.yaml -> no goal window -> whole recording (4 + 60 + 2 s)
    assert row['window_source'] == 'full range'
    assert row['duration_source'] == 'gt.tum span'
    assert row['duration_s'] == pytest.approx(66.0)
    assert row['seed'] == '' and row['sanity_flag'] == ''
    for k in ('ape_rmse', 'det_total', 'mo_max_corr_m', 'goals_succeeded',
              'mo_init_m'):
        assert math.isnan(row[k]), k
    for n in ('evo_none', 'result.yaml', 'landmarks.csv', 'map_odom.tum',
              'full GT range used'):
        assert n in row['notes']


def test_summary_outputs(experiment, tmp_path):
    out = tmp_path / 'summary'
    assert sm.main([experiment, '--out', str(out)]) == 0
    for f in ('summary.csv', 'summary_modes.csv', 'summary_paired.csv',
              'summary.md', 'summary_table.tex'):
        assert (out / f).stat().st_size > 0
    with open(out / 'summary.csv') as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 9
    assert [r['mode'] for r in rows][::3] == ['baseline', 'active', 'control']
    flags = {r['run']: r['sanity_flag'] for r in rows}
    assert flags.pop('control_03') == 'bad_db_init'
    assert set(flags.values()) == {'ok'}
    by_run = {r['run']: r for r in rows}
    with open(out / 'summary_paired.csv') as f:
        paired = list(csv.DictReader(f))
    assert [p['pair'] for p in paired] == ['42', '43', '44']
    for p in paired:
        a = by_run[p['run_active']]
        b = by_run[p['run_baseline']]
        assert float(p['d_active_ape_rmse']) == pytest.approx(
            float(a['ape_rmse']) - float(b['ape_rmse']), abs=1e-5)
        assert float(p['d_control_duration_s']) == pytest.approx(0.0)
        assert float(p['d_active_det_per_m']) > 0
    assert paired[2]['flagged'] == 'control_03'
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
    assert '## Paired by seed' in md and '(3/3)' in md
    assert 'Sanity flag `bad_db_init`' in md and 'control_03 (1.000 m' in md
    assert 'evaluation window: **result.yaml**' in md
    tex = (out / 'summary_table.tex').read_text()
    assert '\\toprule' in tex and '\\bottomrule' in tex
    assert 'Metric & baseline ($n{=}3$) & active ($n{=}3$) & control ' \
        '($n{=}3$)' in tex and '\\pm' in tex
    assert 'sanity flag bad_db_init (kept in the means): control_03' in tex
    body = tex.split('\\midrule')[1].split('\\bottomrule')[0]
    lines = body.strip().splitlines()
    # report Table 1 (T-012b): exactly these rows, modes as columns
    assert [ln.split(' & ')[0] for ln in lines] == [
        'APE RMSE [m]', 'APE max [m]', 'Detections per m',
        'Max map$\\to$odom corr.\\ [m]', 'Time [s]', 'Path length [m]',
        'Goals reached']
    for line in lines:
        assert line.count('&') == 3 and line.endswith('\\\\'), line
    assert lines[-1] == 'Goals reached & 11/12 & 12/12 & 12/12 \\\\'


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
