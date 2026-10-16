#!/usr/bin/env python3
"""Aggregate evo APE/RPE results of several runs into csv / markdown / png.

  aggregate.py --runs R1 R2 ... --labels L1 L2 ... --out results/ \
               [--align none]

Each RUN is either a directory containing ape.zip and rpe.zip, or a run dir
processed by run_evo.sh, in which case RUN/evo_<align>/ is used.
``--labels`` gives one label per run (e.g. baseline baseline baseline active
active active).

Statistics are read directly from stats.json inside the evo result zips
(that is exactly what evo.tools.file_interface.load_res_file loads; reading
the zip avoids importing evo/matplotlib settings here).

Outputs in --out: results.csv, summary.md, ape_rmse_boxplot.png
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import sys
import zipfile
from collections import OrderedDict
from typing import Dict, List

METRICS = ['ape_rmse', 'ape_mean', 'ape_max', 'rpe_rmse', 'rpe_mean']
CSV_COLS = ['run', 'label'] + METRICS + ['align', 'ape_n', 'rpe_n']


def _read_res(zip_path: str) -> Dict:
    with zipfile.ZipFile(zip_path) as z:
        names = set(z.namelist())
        if not {'info.json', 'stats.json'} <= names:
            raise ValueError('%s is not an evo result file' % zip_path)
        info = json.loads(z.read('info.json'))
        stats = json.loads(z.read('stats.json'))
        n = None
        if 'error_array.npy' in names:
            try:
                import io

                import numpy as np
                n = int(np.load(io.BytesIO(z.read('error_array.npy'))).size)
            except ImportError:  # pragma: no cover
                n = None
    return {'info': info, 'stats': stats, 'n': n}


def _alignment_of(title: str) -> str:
    """The alignment line(s) evo puts into the result title."""
    parts = [ln.strip('()') for ln in title.splitlines()[1:]
             if 'align' in ln]
    return '; '.join(parts) if parts else '?'


def _result_dir(run: str, align: str) -> str:
    if os.path.isfile(os.path.join(run, 'ape.zip')):
        return run
    d = os.path.join(run, 'evo_' + align)
    if os.path.isfile(os.path.join(d, 'ape.zip')):
        return d
    raise FileNotFoundError(
        'no ape.zip in %s or %s (run eval/run_evo.sh %s --align %s first)'
        % (run, d, run, align))


def _run_name(run: str) -> str:
    name = os.path.basename(os.path.normpath(run))
    if name.startswith('evo_'):  # user passed the result dir itself
        name = os.path.basename(os.path.dirname(os.path.normpath(run)))
    return name


def _fmt(v: float, nd: int = 4) -> str:
    return '—' if v is None or (isinstance(v, float) and math.isnan(v)) \
        else '%.*f' % (nd, v)


def mean_std(xs: List[float]):
    m = statistics.fmean(xs)
    s = statistics.stdev(xs) if len(xs) > 1 else float('nan')
    return m, s


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--runs', nargs='+', required=True)
    ap.add_argument('--labels', nargs='+', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--align', default='none',
                    help='which evo_<align>/ subdir to read (default none)')
    ap.add_argument('--allow_mixed_alignment', action='store_true',
                    help='do not fail if runs were evaluated with '
                         'different alignment settings')
    a = ap.parse_args()

    if len(a.labels) != len(a.runs):
        ap.error('--labels must have exactly one label per run '
                 '(%d runs, %d labels)' % (len(a.runs), len(a.labels)))

    rows = []
    for run, label in zip(a.runs, a.labels):
        try:
            rd = _result_dir(run, a.align)
        except FileNotFoundError as e:
            print('ERROR: %s' % e, file=sys.stderr)
            return 2
        ape = _read_res(os.path.join(rd, 'ape.zip'))
        rpe = _read_res(os.path.join(rd, 'rpe.zip'))
        rows.append({
            'run': _run_name(run),
            'label': label,
            'ape_rmse': ape['stats']['rmse'],
            'ape_mean': ape['stats']['mean'],
            'ape_max': ape['stats']['max'],
            'rpe_rmse': rpe['stats']['rmse'],
            'rpe_mean': rpe['stats']['mean'],
            'align': _alignment_of(ape['info'].get('title', '')),
            'ape_n': ape['n'],
            'rpe_n': rpe['n'],
            '_rpe_title': rpe['info'].get('title', ''),
        })

    aligns = sorted({r['align'] for r in rows})
    if len(aligns) > 1 and not a.allow_mixed_alignment:
        print('ERROR: runs use different alignments: %s' % aligns,
              file=sys.stderr)
        return 2
    rpe_titles = sorted({r['_rpe_title'].splitlines()[1]
                         if '\n' in r['_rpe_title'] else ''
                         for r in rows})

    os.makedirs(a.out, exist_ok=True)
    csv_path = os.path.join(a.out, 'results.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLS, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: ('%.6f' % r[k] if k in METRICS else r[k])
                        for k in CSV_COLS})

    groups: 'OrderedDict[str, List[Dict]]' = OrderedDict()
    for r in rows:
        groups.setdefault(r['label'], []).append(r)

    lines = ['# APE / RPE summary', '',
             '- alignment: **%s**' % ', '.join(aligns),
             '- RPE: %s' % ', '.join(t for t in rpe_titles if t),
             '- metric: translation part, metres; mean ± std (sample std, '
             'ddof=1) over runs', '']
    if any('not aligned' in x for x in aligns):
        lines.append('> Without alignment APE includes any constant offset '
                     'between the GT and map frames (e.g. a wrong '
                     'gt_offset).')
    else:
        lines.append('> Alignment is ON: a constant rigid offset between '
                     'est and GT is absorbed and NOT reflected in APE; state '
                     'this explicitly in the report.')
    lines.append('')
    hdr = ['label', 'n'] + METRICS
    lines.append('| ' + ' | '.join(hdr) + ' |')
    lines.append('|' + '|'.join(['---'] * len(hdr)) + '|')
    summary: Dict[str, Dict[str, tuple]] = {}
    for label, rs in groups.items():
        summary[label] = {m: mean_std([r[m] for r in rs]) for m in METRICS}
        cells = [label, str(len(rs))] + [
            '%s ± %s' % (_fmt(summary[label][m][0]),
                         _fmt(summary[label][m][1])) for m in METRICS]
        lines.append('| ' + ' | '.join(cells) + ' |')
    labels = list(groups)
    if len(labels) == 2:
        b, c = labels
        lines.append('')
        for m in ('ape_rmse', 'rpe_rmse'):
            mb, mc = summary[b][m][0], summary[c][m][0]
            rel = (mc - mb) / mb * 100.0 if mb else float('nan')
            lines.append('- %s: %s vs %s = %s vs %s m (%+.1f %%)'
                         % (m, c, b, _fmt(mc), _fmt(mb), rel))
        if min(len(groups[b]), len(groups[c])) < 5:
            lines.append('- n < 5 per group: treat differences as '
                         'indicative, not statistically established.')
    lines += ['', '## Per run', '',
              '| run | label | ' + ' | '.join(METRICS) + ' | APE poses |',
              '|' + '|'.join(['---'] * (len(METRICS) + 3)) + '|']
    for r in rows:
        lines.append('| %s | %s | %s | %s |' % (
            r['run'], r['label'],
            ' | '.join(_fmt(r[m]) for m in METRICS), r['ape_n']))
    md_path = os.path.join(a.out, 'summary.md')
    with open(md_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')

    png_path = os.path.join(a.out, 'ape_rmse_boxplot.png')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(9, 4))
        for ax, m, title in ((axes[0], 'ape_rmse', 'APE RMSE'),
                             (axes[1], 'rpe_rmse', 'RPE RMSE')):
            data = [[r[m] for r in groups[lb]] for lb in labels]
            ax.boxplot(data)
            ax.set_xticks(range(1, len(labels) + 1))
            ax.set_xticklabels(labels)
            for i, ys in enumerate(data, 1):
                ax.plot([i] * len(ys), ys, 'o', color='k', alpha=0.6, ms=4)
            ax.set_title(title)
            ax.set_ylabel('m')
            ax.set_ylim(bottom=0)
            ax.grid(axis='y', alpha=0.3)
        fig.suptitle('alignment: %s' % ', '.join(aligns), fontsize=9)
        fig.tight_layout()
        fig.savefig(png_path, dpi=120)
        plt.close(fig)
    except ImportError:
        png_path = None
        print('matplotlib not available: no plot', file=sys.stderr)

    print(open(md_path).read())
    print('wrote %s, %s%s' % (csv_path, md_path,
                              ', ' + png_path if png_path else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
