#!/usr/bin/env python3
"""Per-run and per-mode summary of an experiment (APE/RPE + secondary metrics).

  summarize.py RUNS_ROOT [--out DIR] [--align none] [--window 1.0]
  summarize.py --runs R1 R2 ... --out DIR

RUNS_ROOT holds one directory per run (``<mode>_<NN>`` from
``scripts/run_experiment.sh``). The mode of a run is ``mode`` in its
``run.yaml``; without one, the directory name prefix before ``_<NN>``.

Per run (missing inputs give empty cells and a note, not an error):

* APE rmse / mean / max, RPE rmse - ``evo_<align>/{ape,rpe}.zip`` written
  by ``eval/run_evo.sh`` (stats.json, same reader as aggregate.py)
* duration [s] - sum of the goals' ``duration_sim_s`` in ``result.yaml``;
  fallback: time span of gt.tum
* path length [m] - integrated xy distance of gt.tum
* goals succeeded / total - ``result.yaml``
* detections - total over ``landmarks.csv``, per metre of GT path, and the
  fraction of the GT time span in which at least one detection arrived in
  the last ``--window`` seconds
* map->odom (``map_odom.tum``) - largest step between consecutive samples
  and largest correction, translation [m] and yaw [deg]; the correction is
  taken w.r.t. the first sample (``--map-odom-ref first``, removes the
  constant spawn offset) or the identity (``zero``)

Per mode: mean, sample std (ddof=1), min and max over runs.

Outputs in --out (default RUNS_ROOT/summary): summary.csv (per run),
summary_modes.csv (per mode), summary.md, summary_table.tex (booktabs).
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
    ('duration_s', 'duration', 'Time', 's', 1),
    ('path_length_m', 'path length (GT)', 'Path length (GT)', 'm', 2),
    ('goals_succeeded', 'goals succeeded', 'Goals reached', '', 2),
    ('det_total', 'detections', 'Detections', '', 0),
    ('det_per_m', 'detections per m', 'Detections per m', '1/m', 1),
    ('det_coverage_pct', 'time with a detection in the last window',
     'Time with detection', '%', 1),
    ('mo_max_step_m', 'map->odom max step', r'map$\to$odom max step', 'm', 3),
    ('mo_max_step_yaw_deg', 'map->odom max yaw step',
     r'map$\to$odom max yaw step', 'deg', 2),
    ('mo_max_corr_m', 'map->odom max |correction|',
     r'max $|$map$\to$odom corr.$|$', 'm', 3),
    ('mo_max_corr_yaw_deg', 'map->odom max |yaw correction|',
     r'max $|$map$\to$odom yaw corr.$|$', 'deg', 2),
]
METRIC_KEYS = [m[0] for m in METRICS]
RUN_COLS = (['run', 'mode', 'mode_source'] + METRIC_KEYS
            + ['goals_total', 'duration_source', 'det_msgs', 'mo_n', 'align',
               'notes'])
# Metrics for which "relative to baseline" is printed in summary.md.
REL_KEYS = ('ape_rmse', 'ape_max', 'rpe_rmse', 'duration_s', 'det_per_m',
            'det_coverage_pct', 'mo_max_corr_m')


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


def detection_stats(rows: Sequence[Tuple[float, int, tuple]], t0: float,
                    t1: float, path_m: float, window: float = 1.0) -> Dict:
    """Totals of landmarks.csv rows ``(t, n, ids)``."""
    total = sum(int(r[1]) for r in rows)
    stamps = [r[0] for r in rows if r[1] >= 1]
    return {
        'det_total': float(total),
        'det_msgs': len(rows),
        'det_per_m': total / path_m if path_m > 0 else NAN,
        'det_coverage_pct': 100.0 * coverage_fraction(stamps, t0, t1, window),
    }


def map_odom_stats(poses: Sequence[Tuple[float, float, float]],
                   reference: str = 'first') -> Dict:
    """Step / correction extremes of a map->odom sequence ``(x, y, yaw)``.

    step_i = |p_i - p_{i-1}| (xy) and |wrap(yaw_i - yaw_{i-1})|;
    correction_i = |p_i - p_ref| and |wrap(yaw_i - yaw_ref)|, with p_ref the
    first sample (``'first'``) or the identity (``'zero'``). These are
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
def evo_stats(run_dir: str, align: str) -> Optional[Dict]:
    d = os.path.join(run_dir, 'evo_' + align)
    ape_p, rpe_p = os.path.join(d, 'ape.zip'), os.path.join(d, 'rpe.zip')
    if not (os.path.isfile(ape_p) and os.path.isfile(rpe_p)):
        return None
    ape, rpe = _read_res(ape_p), _read_res(rpe_p)
    return {'ape_rmse': ape['stats']['rmse'], 'ape_mean': ape['stats']['mean'],
            'ape_max': ape['stats']['max'], 'rpe_rmse': rpe['stats']['rmse'],
            'align': _alignment_of(ape['info'].get('title', ''))}


def load_run(run_dir: str, align: str = 'none', window: float = 1.0,
             map_odom_ref: str = 'first') -> Dict:
    """All per-run metrics; missing inputs -> NaN + a note."""
    name = os.path.basename(os.path.normpath(run_dir))
    mode, src = run_mode(run_dir)
    row: Dict = {k: NAN for k in METRIC_KEYS}
    row.update({'run': name, 'mode': mode, 'mode_source': src,
                'goals_total': '', 'duration_source': '', 'det_msgs': '',
                'mo_n': '', 'align': '', 'run_dir': run_dir})
    notes = []
    name_mode = mode_from_name(name)
    if src == 'run.yaml' and name_mode and name_mode != mode:
        notes.append('run.yaml mode %r != dir name %r' % (mode, name_mode))

    ev = evo_stats(run_dir, align)
    if ev is None:
        notes.append('no evo_%s/ape.zip+rpe.zip' % align)
    else:
        row.update(ev)

    gt_path = os.path.join(run_dir, 'gt.tum')
    t0 = t1 = None
    if os.path.isfile(gt_path):
        gt = read_tum(gt_path)
        if gt:
            t0, t1 = gt[0][0], gt[-1][0]
            row['path_length_m'] = path_length_xy([(r[1], r[2]) for r in gt])
    else:
        notes.append('no gt.tum')

    res_path = os.path.join(run_dir, 'result.yaml')
    if os.path.isfile(res_path):
        rs = result_stats(_load_yaml(res_path) or {})
        row['goals_succeeded'] = rs['goals_succeeded']
        row['goals_total'] = rs['goals_total']
        if not math.isnan(rs['duration_s']):
            row['duration_s'] = rs['duration_s']
            row['duration_source'] = 'result.yaml'
    else:
        notes.append('no result.yaml')
    if math.isnan(row['duration_s']) and t0 is not None:
        row['duration_s'] = t1 - t0
        row['duration_source'] = 'gt.tum span'

    lm_path = os.path.join(run_dir, 'landmarks.csv')
    if os.path.isfile(lm_path) and t0 is not None:
        row.update(detection_stats(read_landmarks_csv(lm_path), t0, t1,
                                   row['path_length_m'], window))
    elif not os.path.isfile(lm_path):
        notes.append('no landmarks.csv')

    mo_path = os.path.join(run_dir, 'map_odom.tum')
    if os.path.isfile(mo_path):
        poses = [(r[1], r[2], quat_to_yaw(*r[4:8])) for r in read_tum(mo_path)]
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
            w.writerow({k: ('%.6f' % r[k] if k in METRIC_KEYS
                            and not _isnan(r[k]) else
                            ('' if k in METRIC_KEYS else r[k]))
                        for k in RUN_COLS})


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
              window: float, map_odom_ref: str) -> str:
    names = list(modes)
    lines = ['# Experiment summary', '',
             '- alignment: **%s**' % (', '.join(align_labels) or 'n/a'),
             '- runs per mode: %s' % ', '.join(
                 '%s %d' % (m, modes[m]['_n_runs']) for m in names),
             '- cells: mean ± sample std (ddof=1) over runs; [min, max] below',
             '- detection coverage window: %g s; map->odom correction '
             'reference: %s sample' % (window, map_odom_ref), '']
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
    lines += ['', '## Per run', '']
    cols = ['run', 'mode'] + METRIC_KEYS + ['notes']
    lines += ['| ' + ' | '.join(cols) + ' |',
              '|' + '|'.join(['---'] * len(cols)) + '|']
    nds = {m[0]: m[4] for m in METRICS}
    for r in rows:
        lines.append('| ' + ' | '.join(
            [r['run'], r['mode']]
            + [_md(r[k], nds[k]) for k in METRIC_KEYS] + [r['notes']]) + ' |')
    return '\n'.join(lines) + '\n'


def _tex_escape(s: str) -> str:
    return (s.replace('\\', r'\textbackslash{}').replace('_', r'\_')
            .replace('%', r'\%').replace('&', r'\&').replace('#', r'\#'))


# Compact row set for the 2-page PDF.
TEX_KEYS = ('ape_rmse', 'ape_max', 'rpe_rmse', 'duration_s', 'path_length_m',
            'det_per_m', 'det_coverage_pct', 'mo_max_step_m', 'mo_max_corr_m',
            'mo_max_corr_yaw_deg')


def render_tex(modes: Dict, align_labels: Sequence[str],
               window: float) -> str:
    names = list(modes)
    units = {'%': r'\%', 'deg': r'$^\circ$', '1/m': r'm$^{-1}$'}
    out = ['% Generated by eval/summarize.py. Needs \\usepackage{booktabs}.',
           '%% alignment: %s; mean $\\pm$ sample std over runs; '
           'coverage window %g s' % (', '.join(align_labels) or 'n/a',
                                     window),
           r'\begin{tabular}{@{}l' + 'c' * len(names) + '@{}}',
           r'\toprule',
           'Metric & ' + ' & '.join(
               r'%s ($n{=}%d$)' % (_tex_escape(m), modes[m]['_n_runs'])
               for m in names) + r' \\',
           r'\midrule']
    meta = {m[0]: m for m in METRICS}
    for k in TEX_KEYS:
        _, _, label, unit, nd = meta[k]
        u = units.get(unit, unit)
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
        unit_txt = ' [%s]' % u if u else ''
        out.append('%s%s & %s \\\\' % (label, unit_txt, ' & '.join(cells)))
    out.append('Goals reached & ' + ' & '.join(
        '%d/%d' % modes[m]['_goals'] for m in names) + r' \\')
    out += [r'\bottomrule', r'\end{tabular}', '']
    return '\n'.join(out)


# --------------------------------------------------------------- main
def collect(run_dirs: Sequence[str], align: str, window: float,
            map_odom_ref: str) -> List[Dict]:
    return [load_run(d, align, window, map_odom_ref) for d in run_dirs]


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
    ap.add_argument('--map-odom-ref', choices=('first', 'zero'),
                    default='first')
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

    rows = collect(run_dirs, a.align, a.window, a.map_odom_ref)
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

    os.makedirs(out, exist_ok=True)
    paths = [os.path.join(out, n) for n in (
        'summary.csv', 'summary_modes.csv', 'summary.md',
        'summary_table.tex')]
    write_run_csv(paths[0], rows)
    write_mode_csv(paths[1], modes)
    md = render_md(rows, modes, aligns, a.window, a.map_odom_ref)
    with open(paths[2], 'w') as f:
        f.write(md)
    with open(paths[3], 'w') as f:
        f.write(render_tex(modes, aligns, a.window))
    print(md)
    print('wrote ' + ', '.join(paths))
    return 0


if __name__ == '__main__':
    sys.exit(main())
