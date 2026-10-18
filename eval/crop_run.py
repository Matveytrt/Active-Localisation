#!/usr/bin/env python3
"""Crop the recordings of one run to the evaluation window.

  crop_run.py RUN_DIR [--gt gt.tum] [--est est.tum] [--gt-pad 0.1]
              [--window T0 T1]

Evaluation window = [start of the first goal, end of the last goal] in sim
time, from the per-goal ``start_sim_s`` / ``end_sim_s`` of ``result.yaml``
(``al_eval.scenario.RunResult``). The recording also contains the start-up
(localization init, relocalization jump, nudge) and the time after the last
goal; those parts must not enter APE/RPE or the secondary metrics
(analysis plan §5).

Fallback: without a usable window in result.yaml (no file, old result.yaml
without the stamps, end <= start, or a window that does not overlap the GT
time span - e.g. a wall/sim clock mismatch) the full GT time span is used
and a WARNING is printed. ``--window T0 T1`` overrides result.yaml.

Writes into RUN_DIR (only for inputs that exist):

  <gt>_eval.tum         GT in [t0 - gt_pad, t1 + gt_pad]  (the pad keeps the
                        est poses at the window edges interpolable by evo)
  <est>_eval.tum        est in [t0, t1]
  map_odom_eval.tum     map->odom in [t0, t1]
  landmarks_eval.csv    landmarks.csv rows in [t0, t1]
  eval_window.yaml      t0, t1, source, per-file row counts

``eval/run_evo.sh`` calls this by default; ``eval/summarize.py`` and
``eval/plots.py`` apply the same window in memory (same functions).
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'navigation', 'src', 'al_eval'))
from al_eval import scenario as sc  # noqa: E402
from al_eval.recorder_helpers import (  # noqa: E402
    LANDMARKS_CSV_HEADER, format_landmark_row, read_landmarks_csv)
from al_eval.tum import read_tum, write_tum  # noqa: E402

WINDOW_FILE = 'eval_window.yaml'
SRC_RESULT = 'result.yaml'
SRC_FULL = 'full range'
SRC_MANUAL = 'manual'
DEFAULT_GT_PAD = 0.1

Window = Tuple[float, float, str, str]  # t0, t1, source, note


def _finite(v) -> bool:
    return (v is not None and not isinstance(v, bool)
            and isinstance(v, (int, float)) and math.isfinite(v))


def window_from_result(result: Dict) -> Tuple[Optional[Tuple[float, float]],
                                              str]:
    """((t0, t1) or None, reason) from a parsed result.yaml.

    t0 = earliest ``start_sim_s``, t1 = latest ``end_sim_s`` over the goals
    that have them (skipped goals have none).
    """
    goals = (result or {}).get('goals') or []
    starts = [g.get('start_sim_s') for g in goals if isinstance(g, dict)]
    ends = [g.get('end_sim_s') for g in goals if isinstance(g, dict)]
    starts = [float(v) for v in starts if _finite(v)]
    ends = [float(v) for v in ends if _finite(v)]
    if not starts or not ends:
        return None, 'no per-goal start_sim_s/end_sim_s in result.yaml'
    t0, t1 = min(starts), max(ends)
    if t1 <= t0:
        return None, 'result.yaml window end %.3f <= start %.3f' % (t1, t0)
    return (t0, t1), ''


def _load_yaml(path: str):
    import yaml
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def resolve_window(run_dir: str, gt: Sequence[Sequence[float]],
                   manual: Optional[Sequence[float]] = None) -> Window:
    """(t0, t1, source, note) for a run; see the module docstring.

    ``source`` is ``'result.yaml' | 'manual' | 'full range'``; ``note`` is
    empty unless the full-range fallback was taken (then it says why).
    ``gt`` is the full GT trajectory (its span is the fallback and the
    overlap check).
    """
    span = (gt[0][0], gt[-1][0]) if gt else None
    if manual is not None:
        t0, t1 = float(manual[0]), float(manual[1])
        if t1 <= t0:
            raise ValueError('--window needs T0 < T1')
        return t0, t1, SRC_MANUAL, ''
    path = os.path.join(run_dir, 'result.yaml')
    if os.path.isfile(path):
        try:
            win, why = window_from_result(_load_yaml(path) or {})
        except Exception as e:  # noqa: BLE001 - fall back, but say why
            win, why = None, 'cannot read result.yaml: %s' % e
    else:
        win, why = None, 'no result.yaml'
    if win is not None and span is not None:
        if win[1] <= span[0] or win[0] >= span[1]:
            win, why = None, (
                'result.yaml window [%.3f, %.3f] does not overlap the GT '
                'span [%.3f, %.3f] (clock mismatch?)'
                % (win[0], win[1], span[0], span[1]))
    if win is not None:
        return win[0], win[1], SRC_RESULT, ''
    if span is None:
        return math.nan, math.nan, SRC_FULL, why + '; no GT either'
    return span[0], span[1], SRC_FULL, why + ' -> full GT range used'


def crop_rows(rows: Sequence[Sequence[float]], t0: float, t1: float,
              pad: float = 0.0) -> List:
    """Rows (first field = time) with ``t0 - pad <= t <= t1 + pad``."""
    lo, hi = t0 - pad, t1 + pad
    return [r for r in rows if lo <= r[0] <= hi]


def eval_name(name: str) -> str:
    """``gt.tum`` -> ``gt_eval.tum``; ``landmarks.csv`` -> ..._eval.csv."""
    stem, ext = os.path.splitext(name)
    return stem + '_eval' + ext


def write_landmarks(path: str, rows) -> None:
    with open(path, 'w', encoding='ascii') as f:
        f.write(LANDMARKS_CSV_HEADER + '\n')
        for t, _, ids in rows:
            f.write(format_landmark_row('%.9f' % t, ids) + '\n')


def crop_run(run_dir: str, gt_name: str = 'gt.tum',
             est_name: str = 'est.tum', gt_pad: float = DEFAULT_GT_PAD,
             manual: Optional[Sequence[float]] = None) -> Dict:
    """Write the cropped files and eval_window.yaml; returns its content."""
    gt_path = os.path.join(run_dir, gt_name)
    if not os.path.isfile(gt_path):
        raise FileNotFoundError(gt_path)
    gt = read_tum(gt_path)
    t0, t1, source, note = resolve_window(run_dir, gt, manual)
    if math.isnan(t0):
        raise ValueError('%s: empty GT, no window' % run_dir)
    files: Dict[str, Dict] = {}
    for name, pad in ((gt_name, gt_pad), (est_name, 0.0),
                      ('map_odom.tum', 0.0)):
        p = os.path.join(run_dir, name)
        if not os.path.isfile(p):
            continue
        rows = gt if name == gt_name else read_tum(p)
        out = crop_rows(rows, t0, t1, pad)
        write_tum(os.path.join(run_dir, eval_name(name)), out)
        files[name] = {'out': eval_name(name), 'rows_in': len(rows),
                       'rows_out': len(out)}
    p = os.path.join(run_dir, 'landmarks.csv')
    if os.path.isfile(p):
        rows = read_landmarks_csv(p)
        out = crop_rows(rows, t0, t1)
        write_landmarks(os.path.join(run_dir, 'landmarks_eval.csv'), out)
        files['landmarks.csv'] = {'out': 'landmarks_eval.csv',
                                  'rows_in': len(rows),
                                  'rows_out': len(out)}
    info = {'t0': round(t0, 9), 't1': round(t1, 9),
            'duration_s': round(t1 - t0, 6), 'source': source,
            'note': note, 'gt_pad_s': gt_pad,
            'recording': {'t0': gt[0][0], 't1': gt[-1][0]},
            'files': files}
    sc.write_yaml(os.path.join(run_dir, WINDOW_FILE), info)
    return info


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('--gt', default='gt.tum')
    ap.add_argument('--est', default='est.tum')
    ap.add_argument('--gt-pad', type=float, default=DEFAULT_GT_PAD,
                    help='extra GT kept on both sides [s] (default 0.1)')
    ap.add_argument('--window', nargs=2, type=float, metavar=('T0', 'T1'),
                    help='explicit window (overrides result.yaml)')
    a = ap.parse_args(argv)
    if a.gt_pad < 0:
        ap.error('--gt-pad must be >= 0')
    try:
        info = crop_run(a.run_dir, a.gt, a.est, a.gt_pad, a.window)
    except (OSError, ValueError) as e:
        print('ERROR: %s' % e, file=sys.stderr)
        return 2
    if info['note']:
        print('WARNING: %s: %s' % (a.run_dir, info['note']), file=sys.stderr)
    print('window [%.3f, %.3f] (%.2f s, %s); recording [%.3f, %.3f]' % (
        info['t0'], info['t1'], info['duration_s'], info['source'],
        info['recording']['t0'], info['recording']['t1']))
    for name, f in info['files'].items():
        print('  %s -> %s: %d -> %d rows' % (name, f['out'], f['rows_in'],
                                             f['rows_out']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
