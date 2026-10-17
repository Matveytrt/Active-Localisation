"""Write evo-like result zips (APE/RPE) without evo, for tests.

Same members as evo 1.38 ``--save_results``: info.json, stats.json,
error_array.npy, timestamps.npy, seconds_from_start.npy. The errors are a
simple xy translation APE (GT linearly interpolated at est stamps) and a
consecutive-pose RPE; they are only meant to be plausible, not identical to
evo's numbers.
"""

import io
import json
import os
import sys
import zipfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'navigation', 'src',
                                'al_eval'))
from al_eval.tum import read_tum  # noqa: E402


def _npy(arr):
    b = io.BytesIO()
    np.save(b, np.asarray(arr))
    return b.getvalue()


def _write(path, title, err, ts):
    err = np.asarray(err, float)
    stats = {'rmse': float(np.sqrt(np.mean(err ** 2))),
             'mean': float(err.mean()), 'median': float(np.median(err)),
             'std': float(err.std()), 'min': float(err.min()),
             'max': float(err.max()), 'sse': float(np.sum(err ** 2))}
    with zipfile.ZipFile(path, 'w') as z:
        z.writestr('info.json', json.dumps({'title': title, 'label': 'm'}))
        z.writestr('stats.json', json.dumps(stats))
        z.writestr('error_array.npy', _npy(err))
        z.writestr('timestamps.npy', _npy(ts))
        z.writestr('seconds_from_start.npy', _npy(np.asarray(ts) - ts[0]))
    return stats


def mock_run_evo(run_dir, align='none'):
    gt = np.asarray(read_tum(os.path.join(run_dir, 'gt.tum')))
    est = np.asarray(read_tum(os.path.join(run_dir, 'est.tum')))
    t = est[:, 0]
    gx = np.interp(t, gt[:, 0], gt[:, 1])
    gy = np.interp(t, gt[:, 0], gt[:, 2])
    ex, ey = est[:, 1] - gx, est[:, 2] - gy
    ape = np.hypot(ex, ey)
    rpe = np.hypot(np.diff(ex), np.diff(ey))
    out = os.path.join(run_dir, 'evo_' + align)
    os.makedirs(out, exist_ok=True)
    tag = '(not aligned)' if align == 'none' else '(%s)' % align
    a = _write(os.path.join(out, 'ape.zip'),
               'APE w.r.t. translation part (m)\n' + tag, ape, t)
    _write(os.path.join(out, 'rpe.zip'),
           'RPE w.r.t. translation part (m)\nfor delta = 1 (frames) '
           'using consecutive pairs\n' + tag, rpe, t[1:])
    return a
