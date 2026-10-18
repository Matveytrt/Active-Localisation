#!/usr/bin/env python3
"""Per-run and per-mode summary of an experiment (APE/RPE + secondary metrics).

  summarize.py RUNS_ROOT [--out DIR] [--align none] [--window 1.0]
               [--map-odom-ref zero|first] [--no-crop]
  summarize.py --runs R1 R2 ... --out DIR

RUNS_ROOT holds one directory per run (``<mode>_<NN>`` from
``scripts/run_experiment.sh``). The mode of a run is ``mode`` in its
``run.yaml``; without one, the directory name prefix before ``_<NN>``.

Evaluation window: every series (GT, landmarks, map->odom) is cropped to
[start of the first goal, end of the last goal] from the per-goal
``start_sim_s`` / ``end_sim_s`` in ``result.yaml`` (``eval/crop_run.py``,
same functions; full GT range + a note if absent). APE/RPE come from
``evo_<align>/`` written by ``eval/run_evo.sh`` (which crops the same way;
its ``eval_window.yaml`` is checked against this window). ``--no-crop``
uses the whole recording and ``evo_<align>_full/`` (``run_evo.sh
--no-crop``).

Per run (missing inputs give empty cells and a note, not an error):

* APE rmse / mean / max, RPE rmse [m] - ``ape.zip`` / ``rpe.zip``
* APE rotation rmse / max, RPE rotation rmse / max [deg] - ``ape_rot.zip``
  / ``rpe_rot.zip`` (evo ``-r angle_deg``)
* duration [s] - sum of the goals' ``duration_sim_s`` in ``result.yaml``;
  fallback: length of the window
* path length [m] - integrated xy distance of the GT in the window;
  total rotation [rad] - sum of |wrapped yaw increments| of the GT in the
  window; mean speed [m/s] = path length / duration
* goals succeeded / total - ``result.yaml``
* detections - total over ``landmarks.csv`` rows in the window, per metre
  of GT path, % of the window in which at least one detection arrived in
  the last ``--window`` seconds, longest interval without a detection [s]
* map->odom (``map_odom.tum``, window) - largest step between consecutive
  samples and largest correction, translation [m] and yaw [deg]; the
  correction is taken w.r.t. the identity (``--map-odom-ref zero``,
  default: with init_from_gt, map->odom = identity means "no correction")
  or the first sample in the window (``first``)
* sanity flag - ``bad_db_init`` if the *first* map->odom sample of the
  whole recording is farther than ``--init-tol-m`` (0.05 m) or
  ``--init-tol-deg`` (2 deg) from the identity (localization started from
  a stale DB pose), ``ok`` otherwise, empty without map_odom.tum. Flagged
  runs stay in the statistics and are listed in summary.md.

Per mode: mean, sample std (ddof=1), min and max over runs.
Paired: runs are paired across modes by ``seed`` (run.yaml, else
result.yaml; else the run index), and per pair the differences
``<mode> - baseline`` of the main metrics are written.

Outputs in --out (default RUNS_ROOT/summary): summary.csv (per run),
summary_modes.csv (per mode), summary_paired.csv (per seed), summary.md,
summary_table.tex (booktabs, compact: rows = metrics, columns = modes).
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import statistics
import sys
from collections import OrderedDict
from typing import Dict, List, Optional, Sequence, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'navigation', 'src', 'al_eval'))
import crop_run as cw  # noqa: E402
from aggregate import _alignment_of, _read_res  # noqa: E402
from al_eval.recorder_helpers import read_landmarks_csv  # noqa: E402
from al_eval.tum import quat_to_yaw, read_tum, wrap_angle  # noqa: E402

MODE_ORDER = ('baseline', 'active', 'control')
_RUN_DIR_RE = re.compile(r'^(.+?)_(\d{2,})$')
NAN = float('nan')

# key, label (md), label (tex), unit, decimals
METRICS: List[Tuple[str, str, str, str, int]] = [
    ('ape_rmse', 'APE RMSE', 'APE RMSE', 'm', 3),
    ('ape_mean', 'APE mean', 'APE mean', 'm', 3),
    ('ape_max', 'APE max', 'APE max', 'm', 3),
    ('rpe_rmse', 'RPE RMSE', 'RPE RMSE', 'm', 3),
    ('ape_rot_rmse', 'APE rot. RMSE', 'APE rot.\\ RMSE', 'deg', 2),
    ('ape_rot_max', 'APE rot. max', 'APE rot.\\ max', 'deg', 2),
    ('rpe_rot_rmse', 'RPE rot. RMSE', 'RPE rot.\\ RMSE', 'deg', 2),
    ('rpe_rot_max', 'RPE rot. max', 'RPE rot.\\ max', 'deg', 2),
    ('duration_s', 'duration', 'Time', 's', 1),
    ('path_length_m', 'path length (GT)', 'Path length', 'm', 2),
    ('rot_total_rad', 'total rotation (GT)', 'Total rotation', 'rad', 2),
    ('mean_speed_mps', 'mean speed (GT path / duration)', 'Mean speed',
     'm/s', 3),
    ('goals_succeeded', 'goals succeeded', 'Goals reached', '', 2),
    ('det_total', 'detections', 'Detections', '', 0),
    ('det_per_m', 'detections per m', 'Detections per m', '', 1),
    ('det_coverage_pct', 'time with a detection in the last window',
     'Time with detection', '%', 1),
    ('det_max_gap_s', 'longest gap without detection',
     'Longest detection gap', 's', 1),
    ('mo_max_step_m', 'map->odom max step', r'map$\to$odom max step', 'm', 3),
    ('mo_max_step_yaw_deg', 'map->odom max yaw step',
     r'map$\to$odom max yaw step', 'deg', 2),
    ('mo_max_corr_m', 'map->odom max |correction|',
     r'Max map$\to$odom corr.', 'm', 3),
    ('mo_max_corr_yaw_deg', 'map->odom max |yaw correction|',
     r'Max map$\to$odom yaw corr.', 'deg', 2),
    ('mo_init_m', 'map->odom first sample |t|', r'map$\to$odom init',
     'm', 3),
    ('mo_init_yaw_deg', 'map->odom first sample |yaw|',
     r'map$\to$odom init yaw', 'deg', 2),
]
METRIC_KEYS = [m[0] for m in METRICS]
RUN_COLS = (['run', 'mode', 'mode_source', 'seed', 'sanity_flag']
            + METRIC_KEYS
            + ['goals_total', 'duration_source', 'det_msgs', 'mo_n', 'align',
               'window_source', 'window_t0', 'window_t1', 'window_s',
               'notes'])
# Metrics for which "relative to baseline" is printed in summary.md.
REL_KEYS = ('ape_rmse', 'ape_max', 'rpe_rmse', 'ape_rot_rmse', 'duration_s',
            'rot_total_rad', 'det_per_m', 'det_coverage_pct',
            'mo_max_corr_m')
# Metrics of the paired (per seed) differences.
PAIRED_KEYS = ('ape_rmse', 'ape_max', 'rpe_rmse', 'ape_rot_rmse',
               'det_per_m', 'det_coverage_pct', 'det_max_gap_s',
               'mo_max_corr_m', 'duration_s', 'path_length_m',
               'rot_total_rad')
# Rows of summary_table.tex (report Table 1, T-012b): fixed set.
TEX_KEYS = ('ape_rmse', 'ape_max', 'det_per_m', 'mo_max_corr_m',
            'duration_s', 'path_length_m', 'goals_succeeded')
INIT_TOL_M = 0.05
INIT_TOL_DEG = 2.0
FLAG_BAD_INIT = 'bad_db_init'


# --------------------------------------------------------------- run discovery
def _load_yaml(path: str):
    import yaml
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def find_runs(root: str) -> List[str]:
    """Sub-directories of ``root`` that look like run dirs, sorted by name."""
    out = []
    for name in sorted(os.listdir(root)):
        d = os.path.join(root, name)
        if not os.path.isdir(d):
            continue
        if any(os.path.exists(os.path.join(d, f))
               for f in ('gt.tum', 'run.yaml', 'result.yaml')):
            out.append(d)
    return out


def mode_from_name(name: str) -> Optional[str]:
    """``'active_03'`` -> ``'active'``; None if not ``<mode>_<NN>``."""
    m = _RUN_DIR_RE.match(name)
    return m.group(1) if m else None


def run_mode(run_dir: str) -> Tuple[str, str]:
    """(mode, source) with source ``'run.yaml' | 'dir name' | 'unknown'``."""
    p = os.path.join(run_dir, 'run.yaml')
    if os.path.isfile(p):
        try:
            data = _load_yaml(p) or {}
        except Exception:  # noqa: BLE001 - fall back to the dir name
            data = {}
        mode = data.get('mode') if isinstance(data, dict) else None
        if isinstance(mode, str) and mode:
            return mode, 'run.yaml'
    mode = mode_from_name(os.path.basename(os.path.normpath(run_dir)))
    if mode:
        return mode, 'dir name'
    return 'unknown', 'unknown'


def mode_sort_key(mode: str):
    return (MODE_ORDER.index(mode) if mode in MODE_ORDER else len(MODE_ORDER),
            mode)


# --------------------------------------------------------------- pure metrics
def path_length_xy(xy: Sequence[Tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(xy, xy[1:]))


def coverage_fraction(stamps: Sequence[float], t0: float, t1: float,
                      window: float = 1.0) -> float:
    """Fraction of [t0, t1] in which a stamp lies in the last ``window`` s.

    A time t is covered if some stamp s satisfies ``t - window < s <= t``,
    i.e. the union of the intervals [s, s + window) clipped to [t0, t1].
    """
    if t1 <= t0:
        return NAN
    if window <= 0.0:
        raise ValueError('window must be > 0')
    covered = 0.0
    cur_lo = cur_hi = None
    for s in sorted(stamps):
        lo, hi = max(s, t0), min(s + window, t1)
        if hi <= lo:
            continue
        if cur_hi is None or lo > cur_hi:
            if cur_hi is not None:
                covered += cur_hi - cur_lo
            cur_lo, cur_hi = lo, hi
        else:
            cur_hi = max(cur_hi, hi)
    if cur_hi is not None:
        covered += cur_hi - cur_lo
    return covered / (t1 - t0)


def max_gap(stamps: Sequence[float], t0: float, t1: float) -> float:
    """Longest interval of [t0, t1] without a stamp (edges included)."""
    if t1 <= t0:
        return NAN
    pts = [t0] + sorted(s for s in stamps if t0 <= s <= t1) + [t1]
    return max(b - a for a, b in zip(pts, pts[1:]))


def rotation_total(yaws: Sequence[float]) -> float:
    """Sum of |wrapped yaw increments| [rad] (0 for < 2 samples)."""
    return sum(abs(wrap_angle(b - a)) for a, b in zip(yaws, yaws[1:]))


def detection_stats(rows: Sequence[Tuple[float, int, tuple]], t0: float,
                    t1: float, path_m: float, window: float = 1.0) -> Dict:
    """Totals of landmarks.csv rows ``(t, n, ids)`` over [t0, t1]."""
    total = sum(int(r[1]) for r in rows)
    stamps = [r[0] for r in rows if r[1] >= 1]
    return {
        'det_total': float(total),
        'det_msgs': len(rows),
        'det_per_m': total / path_m if path_m > 0 else NAN,
        'det_coverage_pct': 100.0 * coverage_fraction(stamps, t0, t1, window),
        'det_max_gap_s': max_gap(stamps, t0, t1),
    }


def map_odom_stats(poses: Sequence[Tuple[float, float, float]],
                   reference: str = 'zero') -> Dict:
    """Step / correction extremes of a map->odom sequence ``(x, y, yaw)``.

    step_i = |p_i - p_{i-1}| (xy) and |wrap(yaw_i - yaw_{i-1})|;
    correction_i = |p_i - p_ref| and |wrap(yaw_i - yaw_ref)|, with p_ref the
    first sample (``'first'``) or the identity (``'zero'``, default). These are
    properties of the transform itself (not the jump of the robot pose,
    which also depends on odom->base_link).
    """
    if reference not in ('first', 'zero'):
        raise ValueError('reference must be first or zero')
    out = {'mo_n': len(poses), 'mo_max_step_m': NAN,
           'mo_max_step_yaw_deg': NAN, 'mo_max_corr_m': NAN,
           'mo_max_corr_yaw_deg': NAN}
    if not poses:
        return out
    rx, ry, ryaw = poses[0] if reference == 'first' else (0.0, 0.0, 0.0)
    out['mo_max_corr_m'] = max(math.hypot(x - rx, y - ry)
                               for x, y, _ in poses)
    out['mo_max_corr_yaw_deg'] = math.degrees(max(
        abs(wrap_angle(yaw - ryaw)) for _, _, yaw in poses))
    if len(poses) > 1:
        pairs = list(zip(poses, poses[1:]))
        out['mo_max_step_m'] = max(math.hypot(b[0] - a[0], b[1] - a[1])
                                   for a, b in pairs)
        out['mo_max_step_yaw_deg'] = math.degrees(max(
            abs(wrap_angle(b[2] - a[2])) for a, b in pairs))
    else:
        out['mo_max_step_m'] = 0.0
        out['mo_max_step_yaw_deg'] = 0.0
    return out


def init_check(pose: Optional[Tuple[float, float, float]],
               tol_m: float = INIT_TOL_M,
               tol_deg: float = INIT_TOL_DEG) -> Dict:
    """Sanity check of the first map->odom sample ``(x, y, yaw)``.

    With ``init_from_gt`` map->odom must start at the identity; a larger
    offset means localization started from a stale DB pose.
    """
    if pose is None:
        return {'mo_init_m': NAN, 'mo_init_yaw_deg': NAN, 'sanity_flag': ''}
    d = math.hypot(pose[0], pose[1])
    a = math.degrees(abs(wrap_angle(pose[2])))
    bad = d > tol_m or a > tol_deg
    return {'mo_init_m': d, 'mo_init_yaw_deg': a,
            'sanity_flag': FLAG_BAD_INIT if bad else 'ok'}


def result_stats(result: Dict) -> Dict:
    """Goals and sim duration from a parsed result.yaml."""
    goals = result.get('goals') or []
    summ = result.get('summary') or {}
    n_total = int(summ.get('n_goals', len(goals)))
    n_ok = summ.get('n_succeeded')
    if n_ok is None:
        n_ok = sum(1 for g in goals if g.get('status') == 'succeeded')
    durs = [g.get('duration_sim_s') for g in goals]
    duration = (float(sum(durs)) if durs and all(d is not None for d in durs)
                else NAN)
    return {'goals_succeeded': float(n_ok), 'goals_total': n_total,
            'duration_s': duration}


def stats_over_runs(values: Sequence[float]) -> Dict[str, float]:
    """mean, sample std (ddof=1), min, max, n over the finite values."""
    xs = [float(v) for v in values
          if v is not None and not (isinstance(v, float) and math.isnan(v))]
    if not xs:
        return {'mean': NAN, 'std': NAN, 'min': NAN, 'max': NAN, 'n': 0}
    return {'mean': statistics.fmean(xs),
            'std': statistics.stdev(xs) if len(xs) > 1 else NAN,
            'min': min(xs), 'max': max(xs), 'n': len(xs)}


# --------------------------------------------------------------- one run
def evo_dir(run_dir: str, align: str, crop: bool = True) -> str:
    """``evo_<align>`` (cropped, default) or ``evo_<align>_full``."""
    return os.path.join(run_dir, 'evo_%s%s' % (align, '' if crop else '_full'))


def evo_stats(d: str) -> Tuple[Optional[Dict], List[str]]:
    """(stats or None, notes) from the evo result dir ``d``.

    Translation zips are required; the rotation zips are optional (results
    of the run_evo.sh before T-011c have none).
    """
    ape_p, rpe_p = os.path.join(d, 'ape.zip'), os.path.join(d, 'rpe.zip')
    name = os.path.basename(d)
    if not (os.path.isfile(ape_p) and os.path.isfile(rpe_p)):
        return None, ['no %s/ape.zip+rpe.zip' % name]
    ape, rpe = _read_res(ape_p), _read_res(rpe_p)
    out = {'ape_rmse': ape['stats']['rmse'], 'ape_mean': ape['stats']['mean'],
           'ape_max': ape['stats']['max'], 'rpe_rmse': rpe['stats']['rmse'],
           'align': _alignment_of(ape['info'].get('title', ''))}
    notes = []
    for kind in ('ape', 'rpe'):
        p = os.path.join(d, kind + '_rot.zip')
        if os.path.isfile(p):
            st = _read_res(p)['stats']
            out[kind + '_rot_rmse'] = st['rmse']
            out[kind + '_rot_max'] = st['max']
        else:
            notes.append('no %s/%s_rot.zip' % (name, kind))
    return out, notes


def run_seed(run_dir: str, result: Optional[Dict]):
    """``seed`` from run.yaml, else from result.yaml; None if neither."""
    p = os.path.join(run_dir, 'run.yaml')
    if os.path.isfile(p):
        try:
            data = _load_yaml(p) or {}
        except Exception:  # noqa: BLE001
            data = {}
        if isinstance(data, dict) and data.get('seed') is not None:
            return data['seed']
    if isinstance(result, dict) and result.get('seed') is not None:
        return result['seed']
    return None


def _check_evo_window(d: str, t0: float, t1: float, crop: bool) -> List[str]:
    """Notes if the evo results were not computed on the same window."""
    p = os.path.join(d, cw.WINDOW_FILE)
    if not crop or not os.path.isdir(d):
        return []
    if not os.path.isfile(p):
        return ['%s has no %s (evo not run on the cropped window? rerun '
                'eval/run_evo.sh)' % (os.path.basename(d), cw.WINDOW_FILE)]
    w = _load_yaml(p) or {}
    try:
        same = (abs(float(w['t0']) - t0) < 1e-6
                and abs(float(w['t1']) - t1) < 1e-6)
    except (KeyError, TypeError, ValueError):
        same = False
    if not same:
        return ['%s window [%s, %s] != [%.3f, %.3f]' % (
            os.path.basename(d), w.get('t0'), w.get('t1'), t0, t1)]
    return []


def load_run(run_dir: str, align: str = 'none', window: float = 1.0,
             map_odom_ref: str = 'zero', crop: bool = True,
             init_tol_m: float = INIT_TOL_M,
             init_tol_deg: float = INIT_TOL_DEG) -> Dict:
    """All per-run metrics; missing inputs -> NaN + a note."""
    name = os.path.basename(os.path.normpath(run_dir))
    mode, src = run_mode(run_dir)
    row: Dict = {k: NAN for k in METRIC_KEYS}
    row.update({'run': name, 'mode': mode, 'mode_source': src, 'seed': '',
                'sanity_flag': '', 'goals_total': '', 'duration_source': '',
                'det_msgs': '', 'mo_n': '', 'align': '', 'window_source': '',
                'window_t0': NAN, 'window_t1': NAN, 'window_s': NAN,
                'run_dir': run_dir})
    notes = []
    name_mode = mode_from_name(name)
    if src == 'run.yaml' and name_mode and name_mode != mode:
        notes.append('run.yaml mode %r != dir name %r' % (mode, name_mode))

    gt_path = os.path.join(run_dir, 'gt.tum')
    gt = read_tum(gt_path) if os.path.isfile(gt_path) else []
    if not os.path.isfile(gt_path):
        notes.append('no gt.tum')
    t0 = t1 = None
    if gt:
        if crop:
            t0, t1, wsrc, wnote = cw.resolve_window(run_dir, gt)
            if wnote:
                notes.append(wnote)
        else:
            t0, t1, wsrc = gt[0][0], gt[-1][0], 'off (whole recording)'
        row.update({'window_source': wsrc, 'window_t0': t0,
                    'window_t1': t1, 'window_s': t1 - t0})
        gw = cw.crop_rows(gt, t0, t1)
        row['path_length_m'] = path_length_xy([(r[1], r[2]) for r in gw])
        row['rot_total_rad'] = rotation_total(
            [quat_to_yaw(*r[4:8]) for r in gw])

    d = evo_dir(run_dir, align, crop)
    ev, ev_notes = evo_stats(d)
    notes += ev_notes
    if ev is not None:
        row.update(ev)
        if t0 is not None:
            notes += _check_evo_window(d, t0, t1, crop)

    res_path = os.path.join(run_dir, 'result.yaml')
    result = None
    if os.path.isfile(res_path):
        result = _load_yaml(res_path) or {}
        rs = result_stats(result)
        row['goals_succeeded'] = rs['goals_succeeded']
        row['goals_total'] = rs['goals_total']
        if not math.isnan(rs['duration_s']):
            row['duration_s'] = rs['duration_s']
            row['duration_source'] = 'result.yaml'
    else:
        notes.append('no result.yaml')
    if math.isnan(row['duration_s']) and t0 is not None:
        row['duration_s'] = t1 - t0
        row['duration_source'] = ('eval window'
                                  if row['window_source'] == cw.SRC_RESULT
                                  else 'gt.tum span')
    if row['duration_s'] > 0 and not math.isnan(row['path_length_m']):
        row['mean_speed_mps'] = row['path_length_m'] / row['duration_s']
    seed = run_seed(run_dir, result)
    row['seed'] = '' if seed is None else seed

    lm_path = os.path.join(run_dir, 'landmarks.csv')
    if os.path.isfile(lm_path) and t0 is not None:
        lm = cw.crop_rows(read_landmarks_csv(lm_path), t0, t1)
        row.update(detection_stats(lm, t0, t1, row['path_length_m'], window))
    elif not os.path.isfile(lm_path):
        notes.append('no landmarks.csv')

    mo_path = os.path.join(run_dir, 'map_odom.tum')
    if os.path.isfile(mo_path):
        mo = read_tum(mo_path)
        poses = [(r[1], r[2], quat_to_yaw(*r[4:8])) for r in mo]
        row.update(init_check(poses[0] if poses else None, init_tol_m,
                              init_tol_deg))
        if row['sanity_flag'] == FLAG_BAD_INIT:
            notes.append('first map->odom sample %.3f m / %.1f deg from '
                         'identity (stale DB init?)'
                         % (row['mo_init_m'], row['mo_init_yaw_deg']))
        if t0 is not None:
            poses = [(r[1], r[2], quat_to_yaw(*r[4:8]))
                     for r in cw.crop_rows(mo, t0, t1)]
        row.update(map_odom_stats(poses, map_odom_ref))
    else:
        notes.append('no map_odom.tum')
    row['notes'] = '; '.join(notes)
    return row


def group_by_mode(rows: Sequence[Dict]) -> 'OrderedDict[str, List[Dict]]':
    groups: 'OrderedDict[str, List[Dict]]' = OrderedDict()
    for mode in sorted({r['mode'] for r in rows}, key=mode_sort_key):
        groups[mode] = [r for r in rows if r['mode'] == mode]
    return groups


def summarize_modes(rows: Sequence[Dict]) -> 'OrderedDict[str, Dict]':
    out: 'OrderedDict[str, Dict]' = OrderedDict()
    for mode, rs in group_by_mode(rows).items():
        out[mode] = {k: stats_over_runs([r[k] for r in rs])
                     for k in METRIC_KEYS}
        out[mode]['_n_runs'] = len(rs)
        out[mode]['_goals'] = (
            int(sum(r['goals_succeeded'] for r in rs
                    if not math.isnan(r['goals_succeeded']))),
            int(sum(r['goals_total'] for r in rs if r['goals_total'] != '')))
    return out


# --------------------------------------------------------------- paired
def pair_key(row: Dict):
    """Seed if known, else ``'run<NN>'`` from the dir name."""
    if row.get('seed') not in ('', None):
        return row['seed']
    m = _RUN_DIR_RE.match(row['run'])
    return 'run%s' % m.group(2) if m else row['run']


def paired_rows(rows: Sequence[Dict], keys: Sequence[str] = PAIRED_KEYS,
                base: str = 'baseline') -> Tuple[List[Dict], List[str]]:
    """Per pair key: runs, baseline values and ``<mode> - baseline``.

    Returns (rows, other_modes). A pair without a baseline run gets empty
    differences; if a mode has two runs with the same key the first (by
    name) is used and a note is added.
    """
    groups = group_by_mode(rows)
    others = [m for m in groups if m != base]
    by: 'OrderedDict' = OrderedDict()
    notes: Dict = {}
    for mode, rs in groups.items():
        for r in rs:
            k = pair_key(r)
            slot = by.setdefault(k, {})
            if mode in slot:
                notes.setdefault(k, []).append(
                    'duplicate %s run %s ignored' % (mode, r['run']))
                continue
            slot[mode] = r
    out = []
    for k in sorted(by, key=lambda x: (str(type(x)), x)):
        slot = by[k]
        b = slot.get(base)
        pr: Dict = {'pair': k}
        for mode in [base] + others:
            pr['run_' + mode] = slot[mode]['run'] if mode in slot else ''
        flagged = [r['run'] for r in slot.values()
                   if r.get('sanity_flag') == FLAG_BAD_INIT]
        pr['flagged'] = ';'.join(sorted(flagged))
        pr['notes'] = '; '.join(notes.get(k, []))
        for key in keys:
            bv = b[key] if b is not None else NAN
            pr['%s_%s' % (key, base)] = bv
            for mode in others:
                v = slot[mode][key] if mode in slot else NAN
                pr['d_%s_%s' % (mode, key)] = (
                    NAN if _isnan(v) or _isnan(bv) else v - bv)
        out.append(pr)
    return out, others


def paired_summary(prs: Sequence[Dict], others: Sequence[str],
                   key: str, mode: str) -> Dict:
    """Mean difference and sign consistency over pairs (finite only)."""
    ds = [p['d_%s_%s' % (mode, key)] for p in prs]
    ds = [d for d in ds if not _isnan(d)]
    if not ds:
        return {'n': 0, 'mean': NAN, 'consistent': 0}
    mean = statistics.fmean(ds)
    sign = (mean > 0) - (mean < 0)
    return {'n': len(ds), 'mean': mean,
            'consistent': sum(1 for d in ds if (d > 0) - (d < 0) == sign)}


def write_paired_csv(path: str, prs: Sequence[Dict], others: Sequence[str],
                     keys: Sequence[str] = PAIRED_KEYS,
                     base: str = 'baseline') -> None:
    cols = (['pair'] + ['run_' + m for m in [base] + list(others)]
            + ['flagged', 'notes'])
    for key in keys:
        cols.append('%s_%s' % (key, base))
        cols += ['d_%s_%s' % (m, key) for m in others]
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
        w.writeheader()
        for p in prs:
            w.writerow({c: (_fmt(p[c], 6) if isinstance(p[c], float)
                            else p[c]) for c in cols})


# --------------------------------------------------------------- writers
def _isnan(v) -> bool:
    return v is None or (isinstance(v, float) and math.isnan(v))


def _fmt(v, nd: int) -> str:
    return '' if _isnan(v) else '%.*f' % (nd, v)


def _md(v, nd: int) -> str:
    return '—' if _isnan(v) else '%.*f' % (nd, v)


def write_run_csv(path: str, rows: Sequence[Dict]) -> None:
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=RUN_COLS, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: (_fmt(r[k], 6) if isinstance(r[k], float)
                            else r[k]) for k in RUN_COLS})


def write_mode_csv(path: str, modes: Dict) -> None:
    cols = ['mode', 'n_runs'] + ['%s_%s' % (k, s) for k in METRIC_KEYS
                                 for s in ('mean', 'std', 'min', 'max')]
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(cols)
        for mode, st in modes.items():
            w.writerow([mode, st['_n_runs']] + [
                _fmt(st[k][s], 6) for k in METRIC_KEYS
                for s in ('mean', 'std', 'min', 'max')])


def _label(m) -> str:
    """Markdown row label; '|' is escaped so it does not split the cell."""
    text = '%s [%s]' % (m[1], m[3]) if m[3] else m[1]
    return text.replace('|', '\\|')


def render_md(rows: Sequence[Dict], modes: Dict, align_labels: Sequence[str],
              window: float, map_odom_ref: str,
              paired: Optional[Tuple[List[Dict], List[str]]] = None) -> str:
    names = list(modes)
    wsrc = sorted({r['window_source'] for r in rows if r['window_source']})
    ref_txt = ('identity (zero)' if map_odom_ref == 'zero'
               else 'first sample in the window')
    lines = ['# Experiment summary', '',
             '- alignment: **%s**' % (', '.join(align_labels) or 'n/a'),
             '- evaluation window: **%s**' % (', '.join(wsrc) or 'n/a'),
             '- runs per mode: %s' % ', '.join(
                 '%s %d' % (m, modes[m]['_n_runs']) for m in names),
             '- cells: mean ± sample std (ddof=1) over runs; [min, max] below',
             '- detection coverage window: %g s; map->odom correction '
             'reference: %s' % (window, ref_txt), '']
    flagged = [r for r in rows if r['sanity_flag'] == FLAG_BAD_INIT]
    if flagged:
        lines.append('> **Sanity flag `%s`** (first map->odom sample not at '
                     'the identity; kept in the statistics): %s'
                     % (FLAG_BAD_INIT, ', '.join(
                         '%s (%.3f m, %.1f deg)' % (
                             r['run'], r['mo_init_m'], r['mo_init_yaw_deg'])
                         for r in flagged)))
        lines.append('')
    if any(r['window_source'] == cw.SRC_FULL for r in rows):
        lines.append('> Some runs have no goal window in result.yaml: they '
                     'are evaluated over the whole recording (see notes).')
        lines.append('')
    if any('not aligned' in a for a in align_labels):
        lines.append('> Without alignment APE includes any constant offset '
                     'between the GT and map frames.')
        lines.append('')
    hdr = ['metric'] + names
    lines += ['| ' + ' | '.join(hdr) + ' |',
              '|' + '|'.join(['---'] * len(hdr)) + '|']
    for m in METRICS:
        k, nd = m[0], m[4]
        cells = []
        for mode in names:
            s = modes[mode][k]
            cells.append('%s ± %s' % (_md(s['mean'], nd), _md(s['std'], nd)))
        lines.append('| %s | %s |' % (_label(m), ' | '.join(cells)))
    lines.append('| goals reached (sum) | %s |' % ' | '.join(
        '%d/%d' % modes[mode]['_goals'] for mode in names))
    lines += ['', '## Range over runs [min, max]', '',
              '| ' + ' | '.join(hdr) + ' |',
              '|' + '|'.join(['---'] * len(hdr)) + '|']
    for m in METRICS:
        k, nd = m[0], m[4]
        lines.append('| %s | %s |' % (_label(m), ' | '.join(
            '[%s, %s]' % (_md(modes[mode][k]['min'], nd),
                          _md(modes[mode][k]['max'], nd))
            for mode in names)))
    if 'baseline' in modes and len(names) > 1:
        lines += ['', '## Relative to baseline (difference of means)', '']
        base = modes['baseline']
        for mode in names:
            if mode == 'baseline':
                continue
            parts = []
            for k in REL_KEYS:
                b, c = base[k]['mean'], modes[mode][k]['mean']
                if _isnan(b) or _isnan(c) or b == 0:
                    continue
                parts.append('%s %+.1f %%' % (k, (c - b) / b * 100.0))
            lines.append('- %s: %s' % (mode, ', '.join(parts) or 'n/a'))
        if min(modes[m]['_n_runs'] for m in names) < 5:
            lines.append('- n < 5 runs per mode: differences are indicative, '
                         'not statistically established.')
    if paired is not None and paired[0] and paired[1]:
        prs, others = paired
        lines += ['', '## Paired by seed (<mode> - baseline)', '',
                  'Pairs: %s. "k/n": pairs whose difference has the sign of '
                  'the mean difference. Baseline spread = max - min over '
                  'baseline runs.' % ', '.join(str(p['pair']) for p in prs),
                  '']
        hdr2 = (['metric'] + ['Δ %s mean (k/n)' % m for m in others]
                + ['baseline spread'])
        lines += ['| ' + ' | '.join(hdr2) + ' |',
                  '|' + '|'.join(['---'] * len(hdr2)) + '|']
        meta = {m[0]: m for m in METRICS}
        for key in PAIRED_KEYS:
            m = meta[key]
            cells = []
            for mode in others:
                ps = paired_summary(prs, others, key, mode)
                cells.append('%s (%d/%d)' % (_md(ps['mean'], m[4] + 1),
                                             ps['consistent'], ps['n'])
                             if ps['n'] else '—')
            b = modes.get('baseline', {}).get(key)
            spread = (b['max'] - b['min']) if b and b['n'] else NAN
            lines.append('| %s | %s | %s |' % (_label(m), ' | '.join(cells),
                                               _md(spread, m[4] + 1)))
        lines.append('')
        lines.append('Same seed = same start, goals and noise stream, not '
                     'the same noise realization once the paths differ.')
    lines += ['', '## Per run', '']
    cols = ['run', 'mode', 'seed', 'sanity_flag'] + METRIC_KEYS + ['notes']
    lines += ['| ' + ' | '.join(cols) + ' |',
              '|' + '|'.join(['---'] * len(cols)) + '|']
    nds = {m[0]: m[4] for m in METRICS}
    for r in rows:
        lines.append('| ' + ' | '.join(
            [r['run'], r['mode'], str(r['seed']), r['sanity_flag']]
            + [_md(r[k], nds[k]) for k in METRIC_KEYS]
            + [r['notes'].replace('|', '\\|')]) + ' |')
    return '\n'.join(lines) + '\n'


def _tex_escape(s: str) -> str:
    return (s.replace('\\', r'\textbackslash{}').replace('_', r'\_')
            .replace('%', r'\%').replace('&', r'\&').replace('#', r'\#'))


# Row labels of summary_table.tex (TEX_KEYS); units in brackets.
TEX_LABELS = {
    'ape_rmse': 'APE RMSE [m]',
    'ape_max': 'APE max [m]',
    'det_per_m': 'Detections per m',
    'mo_max_corr_m': r'Max map$\to$odom corr.\ [m]',
    'duration_s': 'Time [s]',
    'path_length_m': 'Path length [m]',
    'goals_succeeded': 'Goals reached',
}
TEX_DECIMALS = {'ape_rmse': 3, 'ape_max': 3, 'det_per_m': 1,
                'mo_max_corr_m': 3, 'duration_s': 1, 'path_length_m': 1,
                'goals_succeeded': 1}


def render_tex(modes: Dict, align_labels: Sequence[str], window: float,
               window_sources: Sequence[str] = (),
               flagged: Sequence[str] = ()) -> str:
    """booktabs tabular, rows = TEX_KEYS, columns = modes (mean ± std).

    ``Goals reached`` is the sum over runs (``k/N``); everything else is
    mean ± sample std over runs.
    """
    names = list(modes)
    out = ['% Generated by eval/summarize.py. Needs \\usepackage{booktabs}.',
           '%% alignment: %s; window: %s; mean $\\pm$ sample std over runs; '
           'goals = sum over runs' % (', '.join(align_labels) or 'n/a',
                                      ', '.join(window_sources) or 'n/a'),
           '%% detection coverage window %g s (coverage not in this table)'
           % window]
    if flagged:
        out.append('%% sanity flag %s (kept in the means): %s'
                   % (FLAG_BAD_INIT, ', '.join(flagged)))
    out += [r'\setlength{\tabcolsep}{3pt}',
            r'\begin{tabular}{@{}l' + 'c' * len(names) + '@{}}',
            r'\toprule',
            'Metric & ' + ' & '.join(
                r'%s ($n{=}%d$)' % (_tex_escape(m), modes[m]['_n_runs'])
                for m in names) + r' \\',
            r'\midrule']
    for k in TEX_KEYS:
        if k == 'goals_succeeded':
            cells = ['%d/%d' % modes[m]['_goals'] for m in names]
        else:
            nd = TEX_DECIMALS[k]
            cells = []
            for mode in names:
                s = modes[mode][k]
                if _isnan(s['mean']):
                    cells.append('--')
                elif _isnan(s['std']):
                    cells.append('$%.*f$' % (nd, s['mean']))
                else:
                    cells.append(r'$%.*f \pm %.*f$'
                                 % (nd, s['mean'], nd, s['std']))
        out.append('%s & %s \\\\' % (TEX_LABELS[k], ' & '.join(cells)))
    out += [r'\bottomrule', r'\end{tabular}', '']
    return '\n'.join(out)


# --------------------------------------------------------------- main
def collect(run_dirs: Sequence[str], align: str, window: float,
            map_odom_ref: str = 'zero', crop: bool = True,
            init_tol_m: float = INIT_TOL_M,
            init_tol_deg: float = INIT_TOL_DEG) -> List[Dict]:
    return [load_run(d, align, window, map_odom_ref, crop, init_tol_m,
                     init_tol_deg) for d in run_dirs]


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('root', nargs='?', help='directory with the run dirs')
    ap.add_argument('--runs', nargs='+', help='explicit run dirs (no root)')
    ap.add_argument('--out', help='output dir (default ROOT/summary)')
    ap.add_argument('--align', default='none',
                    help='which evo_<align>/ subdir to read (default none)')
    ap.add_argument('--window', type=float, default=1.0,
                    help='detection coverage window [s] (default 1)')
    ap.add_argument('--map-odom-ref', choices=('zero', 'first'),
                    default='zero',
                    help='map->odom correction w.r.t. the identity (zero, '
                         'default) or the first sample in the window')
    ap.add_argument('--no-crop', action='store_true',
                    help='whole recording; reads evo_<align>_full/')
    ap.add_argument('--init-tol-m', type=float, default=INIT_TOL_M,
                    help='sanity flag: first map->odom |t| [m] (0.05)')
    ap.add_argument('--init-tol-deg', type=float, default=INIT_TOL_DEG,
                    help='sanity flag: first map->odom |yaw| [deg] (2)')
    ap.add_argument('--allow_mixed_alignment', action='store_true')
    a = ap.parse_args(argv)

    if bool(a.root) == bool(a.runs):
        ap.error('give either RUNS_ROOT or --runs')
    run_dirs = a.runs if a.runs else find_runs(a.root)
    if not run_dirs:
        print('ERROR: no run dirs found in %s' % a.root, file=sys.stderr)
        return 2
    out = a.out or os.path.join(a.root or '.', 'summary')
    if a.window <= 0:
        ap.error('--window must be > 0')

    rows = collect(run_dirs, a.align, a.window, a.map_odom_ref,
                   not a.no_crop, a.init_tol_m, a.init_tol_deg)
    aligns = sorted({r['align'] for r in rows if r['align']})
    if len(aligns) > 1 and not a.allow_mixed_alignment:
        print('ERROR: runs use different alignments: %s' % aligns,
              file=sys.stderr)
        return 2
    for r in rows:
        if r['notes']:
            print('WARNING: %s: %s' % (r['run'], r['notes']), file=sys.stderr)
    rows.sort(key=lambda r: (mode_sort_key(r['mode']), r['run']))
    modes = summarize_modes(rows)
    paired = paired_rows(rows)
    wsrc = sorted({r['window_source'] for r in rows if r['window_source']})
    flagged = [r['run'] for r in rows if r['sanity_flag'] == FLAG_BAD_INIT]

    os.makedirs(out, exist_ok=True)
    paths = [os.path.join(out, n) for n in (
        'summary.csv', 'summary_modes.csv', 'summary_paired.csv',
        'summary.md', 'summary_table.tex')]
    write_run_csv(paths[0], rows)
    write_mode_csv(paths[1], modes)
    write_paired_csv(paths[2], paired[0], paired[1])
    md = render_md(rows, modes, aligns, a.window, a.map_odom_ref, paired)
    with open(paths[3], 'w') as f:
        f.write(md)
    with open(paths[4], 'w') as f:
        f.write(render_tex(modes, aligns, a.window, wsrc, flagged))
    print(md)
    print('wrote ' + ', '.join(paths))
    return 0


if __name__ == '__main__':
    sys.exit(main())
