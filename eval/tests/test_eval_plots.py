"""Tests for eval/plots.py and the synthetic end-to-end pipeline.

* mocked evo (always): make_synthetic -> mock evo zips -> summarize -> plots
* real evo (skipped if evo_ape is not installed): make_synthetic ->
  eval/run_evo.sh on each run -> summarize -> plots
"""

import os
import shutil
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
EVAL = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, EVAL)

import make_synthetic as ms  # noqa: E402
import plots  # noqa: E402
import summarize as sm  # noqa: E402
from _evo_mock import mock_run_evo  # noqa: E402

PLOT_FILES = ['%s.%s' % (b, e) for b in ('trajectories', 'ape_vs_time',
                                         'detections_path')
              for e in ('png', 'pdf')]


def _assert_plots(out):
    for f in PLOT_FILES:
        p = os.path.join(out, f)
        assert os.path.getsize(p) > 2000, p
    with open(os.path.join(out, 'trajectories.png'), 'rb') as f:
        assert f.read(8) == b'\x89PNG\r\n\x1a\n'
    with open(os.path.join(out, 'ape_vs_time.pdf'), 'rb') as f:
        assert f.read(5) == b'%PDF-'


# ------------------------------------------------------------ helpers
def test_mean_on_grid():
    a = (np.array([0.0, 1.0, 2.0]), np.array([0.0, 1.0, 2.0]))
    b = (np.array([0.0, 1.0]), np.array([2.0, 3.0]))
    grid, mean = plots.mean_on_grid([a, b], dt=0.5)
    assert grid.tolist() == [0.0, 0.5, 1.0, 1.5, 2.0]
    # both series up to 1 s, only `a` afterwards
    assert mean.tolist() == pytest.approx([1.0, 1.5, 2.0, 1.5, 2.0])
    g, m = plots.mean_on_grid([])
    assert g.size == 0 and m.size == 0


def test_positions_at_and_run_color():
    gt = [(0.0, 0.0, 0.0), (10.0, 10.0, -10.0)]
    p = plots.positions_at(gt, [5.0, 20.0])
    assert p.tolist() == [[5.0, -5.0], [10.0, -10.0]]   # clipped to the span
    assert plots.positions_at(gt, []).shape == (0, 2)
    assert plots.run_index('active_03') == 3
    assert plots.run_color('active_02', 7) == plots.RUN_COLORS[1]
    assert plots.run_color('odd', 1) == plots.RUN_COLORS[0]


def test_load_background(tmp_path):
    arr = np.arange(12.0).reshape(3, 4)
    np.save(tmp_path / 'bg.npy', arr)
    a, ext, origin = plots.load_background(str(tmp_path / 'bg.npy'),
                                           [0, 4, 0, 3])
    assert a.shape == (3, 4) and ext == (0.0, 4.0, 0.0, 3.0)
    assert origin == 'lower'
    plt = plots._plt()
    plt.imsave(tmp_path / 'bg.png', arr)
    a, _, origin = plots.load_background(str(tmp_path / 'bg.png'),
                                         [0, 4, 0, 3])
    assert a.shape[:2] == (3, 4) and origin == 'upper'
    with pytest.raises(ValueError):
        plots.load_background(str(tmp_path / 'bg.npy'), [1, 0, 0, 3])


def test_score_map_background():
    if not os.path.isdir(plots.AL_TOOLS):
        pytest.skip('active_localization tools not found')
    best, ext, origin = plots.score_map_background()
    assert best.ndim == 2 and origin == 'lower'
    assert 0.3 < float(best.max()) <= 1.0 and float(best.min()) >= 0.0
    assert ext[0] < ext[1] and ext[2] < ext[3]


# ------------------------------------------------------------ end to end
def test_end_to_end_mocked_evo(tmp_path):
    root = str(tmp_path / 'exp')
    ms.write_experiment(root, runs=3, seed=1)
    for d in sm.find_runs(root):
        mock_run_evo(d)
    assert sm.main([root]) == 0
    assert os.path.isfile(os.path.join(root, 'summary', 'summary_table.tex'))
    out = os.path.join(root, 'plots')
    np.save(tmp_path / 'bg.npy', np.random.default_rng(0).random((20, 30)))
    assert plots.main([root, '--out', out, '--background',
                       str(tmp_path / 'bg.npy'), '--extent', '-4.9', '3.3',
                       '-2.1', '5.4']) == 0
    _assert_plots(out)


def test_plots_tolerate_missing_inputs(tmp_path):
    root = str(tmp_path / 'exp')
    ms.write_experiment(root, runs=1, seed=2, modes=('baseline', 'active'))
    os.remove(os.path.join(root, 'active_01', 'landmarks.csv'))
    # no evo results at all: APE panels say so instead of failing
    assert plots.main([root]) == 0
    _assert_plots(os.path.join(root, 'plots'))


def test_plots_cli_errors(tmp_path):
    with pytest.raises(SystemExit):
        plots.main([str(tmp_path), '--background', 'x.png'])  # no extent
    with pytest.raises(SystemExit):
        plots.main([str(tmp_path), '--score-map', '--background', 'x.png',
                    '--extent', '0', '1', '0', '1'])
    assert plots.main([str(tmp_path)]) == 2


@pytest.mark.skipif(shutil.which('evo_ape') is None,
                    reason='evo not installed (pip install evo)')
def test_end_to_end_real_evo(tmp_path):
    root = str(tmp_path / 'exp')
    ms.write_experiment(root, runs=3, seed=0)
    for d in sm.find_runs(root):
        r = subprocess.run(['bash', os.path.join(EVAL, 'run_evo.sh'), d,
                            '--plot_format', 'none'],
                           capture_output=True, text=True)
        assert r.returncode == 0, r.stdout + r.stderr
        assert 'SUMMARY align=none' in r.stdout
    assert sm.main([root]) == 0
    rows = sm.collect(sm.find_runs(root), 'none', 1.0, 'first')
    modes = sm.summarize_modes(rows)
    assert modes['active']['ape_rmse']['mean'] < (
        modes['baseline']['ape_rmse']['mean'])
    assert all(r['align'] == 'not aligned' for r in rows)
    assert all(0.0 < r['rpe_rmse'] < 0.1 for r in rows)
    assert plots.main([root, '--score-map']) == 0
    _assert_plots(os.path.join(root, 'plots'))
