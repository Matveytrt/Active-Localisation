#!/usr/bin/env python3
"""Experiment plots per mode (PNG + PDF).

  plots.py RUNS_ROOT [--out DIR] [--align none] [--no-crop]
           [--arrow-spacing 0.5]
           [--score-map | --background FILE --extent XMIN XMAX YMIN YMAX]

All series are cropped to the evaluation window of eval/crop_run.py
([start of goal 1, end of the last goal] from result.yaml; full range if
absent), APE from evo_<align>/ (run_evo.sh crops the same way);
--no-crop: whole recording and evo_<align>_full/.

Writes into --out (default RUNS_ROOT/plots):

* trajectories.{png,pdf}  one panel per mode: GT thin dashed, est thick,
                          one colour per run index, est heading arrows every
                          --arrow-spacing m of est path (0 = off), on an
                          optional background
* ape_vs_time.{png,pdf}   one panel per mode: APE(t) of every run (thin) and
                          the mean over runs (thick), from evo_<align>/ape.zip;
                          below zero one rug row per run with a tick at each
                          landmark detection (landmarks.csv, n >= 1)
* detections_path.{png,pdf} one panel per mode: GT path (grey) and the GT
                          position at each landmarks.csv row, coloured by
                          n_detections

Background (optional):
  --score-map          the S2* best-over-yaw score s*(x, y), computed with
                       navigation/src/active_localization/tools (score_maps.
                       best_over_yaw + _common.build_scorer/room_grid; numpy
                       only, takes ~1-2 s)
  --background F       a PNG (row 0 = top, i.e. ymax) or a .npy 2D array
                       (row 0 = ymin, the score_maps convention), placed at
                       --extent in map coordinates
"""

from __future__ import annotations

import argparse
import io
import os
import sys
import zipfile
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import crop_run as cw  # noqa: E402
import summarize as sm  # noqa: E402
from al_eval.recorder_helpers import read_landmarks_csv  # noqa: E402
from al_eval.tum import quat_to_yaw, read_tum  # noqa: E402

AL_TOOLS = os.path.normpath(os.path.join(
    HERE, '..', 'navigation', 'src', 'active_localization', 'tools'))

# Categorical slots in fixed order (run index 1, 2, 3, ...).
RUN_COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4',
              '#008300', '#6250d6', '#e34948']
INK = '#0b0b0b'
INK_2 = '#52514e'
GRID = '#d9d8d4'
FORMATS = ('png', 'pdf')

Background = Tuple[np.ndarray, Tuple[float, float, float, float], str]


# --------------------------------------------------------------- data
def run_index(name: str) -> int:
    """1-based index from ``<mode>_<NN>``; 0 if the name has none."""
    m = sm._RUN_DIR_RE.match(name)
    return int(m.group(2)) if m else 0


def run_color(name: str, fallback: int) -> str:
    i = run_index(name) or fallback
    return RUN_COLORS[(i - 1) % len(RUN_COLORS)]


def read_ape_series(zip_path: str, with_t0: bool = False):
    """(seconds_from_start, error) arrays from an evo result zip.

    With ``with_t0`` also the absolute stamp of the first pose (NaN if the
    zip has no timestamps.npy), to put other events on the same time axis.
    """
    with zipfile.ZipFile(zip_path) as z:
        names = set(z.namelist())
        err = np.load(io.BytesIO(z.read('error_array.npy')))
        ts = (np.load(io.BytesIO(z.read('timestamps.npy')))
              if 'timestamps.npy' in names else None)
        if 'seconds_from_start.npy' in names:
            t = np.load(io.BytesIO(z.read('seconds_from_start.npy')))
        else:
            t = ts - ts[0]
    t, err = np.asarray(t, float), np.asarray(err, float)
    if not with_t0:
        return t, err
    t0 = float(ts[0]) if ts is not None and len(ts) else float('nan')
    return t, err, t0


def heading_samples(rows: Sequence[Sequence[float]],
                    spacing: float = 0.5) -> np.ndarray:
    """(x, y, yaw) of TUM rows every ``spacing`` m of xy path (first pose
    included); empty (0, 3) for spacing <= 0 or no rows."""
    if spacing <= 0 or not len(rows):
        return np.zeros((0, 3))
    r = np.asarray(rows, float)
    xy = r[:, 1:3]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    idx, nxt = [], 0.0
    for i, si in enumerate(s):
        if si >= nxt:
            idx.append(i)
            nxt = si + spacing
    yaw = np.array([quat_to_yaw(*r[i, 4:8]) for i in idx])
    return np.column_stack([xy[idx, 0], xy[idx, 1], yaw])


def mean_on_grid(series: Sequence[Tuple[np.ndarray, np.ndarray]],
                 dt: float = 0.1) -> Tuple[np.ndarray, np.ndarray]:
    """Mean of several (t, y) series on a common grid.

    Each series is linearly interpolated inside its own time range only;
    the mean at a grid time uses the series that cover it (NaN if none).
    """
    if not series:
        return np.zeros(0), np.zeros(0)
    t_end = max(float(t[-1]) for t, _ in series if len(t))
    grid = np.arange(0.0, t_end + 0.5 * dt, dt)
    stack = np.full((len(series), grid.size), np.nan)
    for k, (t, y) in enumerate(series):
        if len(t) == 0:
            continue
        inside = (grid >= t[0]) & (grid <= t[-1])
        stack[k, inside] = np.interp(grid[inside], t, y)
    with np.errstate(invalid='ignore'):
        valid = np.isfinite(stack).any(axis=0)
        mean = np.full(grid.size, np.nan)
        mean[valid] = np.nanmean(stack[:, valid], axis=0)
    return grid, mean


def positions_at(gt: Sequence[Sequence[float]],
                 stamps: Sequence[float]) -> np.ndarray:
    """GT xy linearly interpolated at ``stamps`` (clipped to the GT span)."""
    g = np.asarray(gt, float)
    s = np.asarray(stamps, float)
    if g.size == 0 or s.size == 0:
        return np.zeros((0, 2))
    return np.column_stack([np.interp(s, g[:, 0], g[:, 1]),
                            np.interp(s, g[:, 0], g[:, 2])])


def load_runs(run_dirs: Sequence[str], align: str,
              crop: bool = True) -> 'Dict[str, List]':
    """mode -> [run dict(name, gt, est, ape, ape_t0, landmarks)], modes
    ordered; series cropped to the evaluation window unless ``crop`` is
    False."""
    runs = []
    for d in run_dirs:
        name = os.path.basename(os.path.normpath(d))
        mode, _ = sm.run_mode(d)
        r = {'name': name, 'mode': mode, 'gt': [], 'est': [], 'ape': None,
             'ape_t0': float('nan'), 'landmarks': [], 'window': None}
        for key, fn in (('gt', 'gt.tum'), ('est', 'est.tum')):
            p = os.path.join(d, fn)
            if os.path.isfile(p):
                r[key] = read_tum(p)
        p = os.path.join(d, 'landmarks.csv')
        if os.path.isfile(p):
            r['landmarks'] = read_landmarks_csv(p)
        if crop and r['gt']:
            t0, t1, _, _ = cw.resolve_window(d, r['gt'])
            r['window'] = (t0, t1)
            for key in ('gt', 'est', 'landmarks'):
                r[key] = cw.crop_rows(r[key], t0, t1)
        z = os.path.join(sm.evo_dir(d, align, crop), 'ape.zip')
        if os.path.isfile(z):
            t, e, r['ape_t0'] = read_ape_series(z, with_t0=True)
            r['ape'] = (t, e)
        runs.append(r)
    out: Dict[str, List] = {}
    for mode in sorted({r['mode'] for r in runs}, key=sm.mode_sort_key):
        out[mode] = sorted([r for r in runs if r['mode'] == mode],
                           key=lambda r: r['name'])
    return out


# --------------------------------------------------------------- background
def load_background(path: str, extent: Sequence[float]) -> Background:
    """(array, extent, origin) for imshow; see the module docstring."""
    if len(extent) != 4:
        raise ValueError('extent must be xmin xmax ymin ymax')
    ext = tuple(float(v) for v in extent)
    if ext[1] <= ext[0] or ext[3] <= ext[2]:
        raise ValueError('extent must have xmin < xmax and ymin < ymax')
    if path.lower().endswith('.npy'):
        arr = np.load(path)
        if arr.ndim != 2:
            raise ValueError('background array must be 2D, got %s'
                             % (arr.shape,))
        return arr, ext, 'lower'
    import matplotlib.image as mpimg
    return mpimg.imread(path), ext, 'upper'


def score_map_background() -> Background:
    """s* (best over yaw) of the S2* scorer over the room grid."""
    if AL_TOOLS not in sys.path:
        sys.path.insert(0, AL_TOOLS)
    from _common import LAYER, build_scorer, room_grid
    from score_maps import best_over_yaw
    grid = room_grid()
    _, _, best, _ = best_over_yaw(build_scorer(), grid, LAYER['yaw_bins'])
    ext = (grid.origin_x, grid.origin_x + grid.width * grid.resolution,
           grid.origin_y, grid.origin_y + grid.height * grid.resolution)
    return best, ext, 'lower'


# --------------------------------------------------------------- plotting
def _plt():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    return plt


def _panels(n: int, sharey: bool = True, size=(4.2, 4.0)):
    plt = _plt()
    fig, axes = plt.subplots(1, max(n, 1), figsize=(size[0] * max(n, 1),
                                                    size[1]),
                             sharex=True, sharey=sharey, squeeze=False)
    return plt, fig, list(axes[0])


def _style(ax) -> None:
    ax.grid(color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(INK_2)
    ax.tick_params(colors=INK_2, labelsize=8)


def _save(fig, out_base: str) -> List[str]:
    paths = []
    for ext in FORMATS:
        p = '%s.%s' % (out_base, ext)
        fig.savefig(p, dpi=150, bbox_inches='tight')
        paths.append(p)
    _plt().close(fig)
    return paths


def _draw_background(ax, bg: Optional[Background]):
    if bg is None:
        return None
    arr, ext, origin = bg
    kw = {'cmap': 'Greys', 'vmin': 0.0} if arr.ndim == 2 else {}
    return ax.imshow(arr, extent=ext, origin=origin, alpha=0.45, zorder=0,
                     **kw)


def _xy_limits(ax, bg, xy_all: List[np.ndarray]) -> None:
    pts = [p for p in xy_all if p.size]
    if bg is not None:
        ext = bg[1]
        ax.set_xlim(ext[0], ext[1])
        ax.set_ylim(ext[2], ext[3])
    elif pts:
        allp = np.vstack(pts)
        lo, hi = allp.min(axis=0), allp.max(axis=0)
        pad = 0.1 * max(hi[0] - lo[0], hi[1] - lo[1], 1.0)
        ax.set_xlim(lo[0] - pad, hi[0] + pad)
        ax.set_ylim(lo[1] - pad, hi[1] + pad)
    ax.set_aspect('equal', adjustable='box')


def plot_trajectories(by_mode: Dict, out_base: str,
                      bg: Optional[Background] = None,
                      arrow_spacing: float = 0.5) -> List[str]:
    plt, fig, axes = _panels(len(by_mode))
    im = None
    for ax, (mode, runs) in zip(axes, by_mode.items()):
        im = _draw_background(ax, bg) or im
        xy_all = []
        for k, r in enumerate(runs, 1):
            col = run_color(r['name'], k)
            if r['est']:
                e = np.asarray(r['est'])[:, 1:3]
                ax.plot(e[:, 0], e[:, 1], color=col, lw=2.0, zorder=3,
                        label='%s est' % r['name'])
                xy_all.append(e)
                h = heading_samples(r['est'], arrow_spacing)
                if len(h):
                    ln = 0.6 * arrow_spacing
                    ax.quiver(h[:, 0], h[:, 1], ln * np.cos(h[:, 2]),
                              ln * np.sin(h[:, 2]), color=col,
                              angles='xy', scale_units='xy', scale=1.0,
                              width=0.006, headwidth=3.5, headlength=3.5,
                              headaxislength=3.0, edgecolor=INK,
                              linewidth=0.5, zorder=5)
            if r['gt']:
                g = np.asarray(r['gt'])[:, 1:3]
                ax.plot(g[:, 0], g[:, 1], color=INK, lw=0.7, ls='--',
                        zorder=4, label='GT' if k == 1 else None)
                xy_all.append(g)
        _xy_limits(ax, bg, xy_all)
        _style(ax)
        ax.set_title('%s (%d runs)' % (mode, len(runs)), fontsize=10,
                     color=INK)
        ax.set_xlabel('x [m] (map)', fontsize=9, color=INK_2)
        ax.legend(fontsize=7, loc='upper right', frameon=False)
    axes[0].set_ylabel('y [m] (map)', fontsize=9, color=INK_2)
    if im is not None and bg is not None and bg[0].ndim == 2:
        fig.colorbar(im, ax=axes, shrink=0.7, pad=0.01,
                     label='background (s* if --score-map)')
    title = 'Trajectories: GT (thin dashed) vs estimate (thick)'
    if arrow_spacing > 0:
        title += ', est heading every %g m' % arrow_spacing
    fig.suptitle(title, fontsize=11, color=INK)
    return _save(fig, out_base)


def plot_ape_vs_time(by_mode: Dict, out_base: str,
                     align: str = 'none') -> List[str]:
    plt, fig, axes = _panels(len(by_mode), size=(4.6, 3.4))
    for ax, (mode, runs) in zip(axes, by_mode.items()):
        series = []
        for k, r in enumerate(runs, 1):
            if r['ape'] is None:
                continue
            t, e = r['ape']
            series.append((t, e))
            ax.plot(t, e, color=run_color(r['name'], k), lw=0.8, alpha=0.6,
                    label=r['name'])
        if series:
            grid, mean = mean_on_grid(series)
            ax.plot(grid, mean, color=INK, lw=2.4, label='mean over runs')
        else:
            ax.text(0.5, 0.5, 'no evo_%s/ape.zip' % align, ha='center',
                    va='center', transform=ax.transAxes, color=INK_2)
        _style(ax)
        _detection_rug(ax, runs)
        ax.set_title('%s (%d runs)' % (mode, len(series)), fontsize=10,
                     color=INK)
        ax.set_xlabel('time from start [s]', fontsize=9, color=INK_2)
        if series:
            ax.legend(fontsize=7, frameon=False, ncol=2, loc='upper center',
                      bbox_to_anchor=(0.5, -0.22))
    axes[0].set_ylabel('APE, translation [m]', fontsize=9, color=INK_2)
    fig.suptitle('APE vs time (alignment: %s)' % align, fontsize=11,
                 color=INK)
    fig.tight_layout()
    return _save(fig, out_base)


def _detection_rug(ax, runs: Sequence[Dict]) -> None:
    """One row of ticks per run below APE = 0, at the detection times.

    Times are relative to the first APE pose of the run (``ape_t0``), i.e.
    on the same axis as the APE curve; runs without APE are skipped.
    """
    top = max(ax.get_ylim()[1], 1e-6)
    rows = [r for r in runs if r['ape'] is not None
            and np.isfinite(r['ape_t0'])]
    h = 0.06 * top
    ax.set_ylim(-(len(rows) * h + 0.02 * top), top)
    ax.axhline(0.0, color=INK_2, lw=0.6)
    for k, r in enumerate(rows, 1):
        st = np.array([row[0] for row in r['landmarks'] if row[1] >= 1])
        if st.size:
            ax.plot(st - r['ape_t0'], np.full(st.size, -(k - 0.5) * h),
                    ls='none', marker='|', ms=4, mew=0.8,
                    color=run_color(r['name'], k))
    if rows:
        ax.text(1.0, -0.5 * len(rows) * h, ' detections', fontsize=6,
                color=INK_2, va='center', ha='left',
                transform=ax.get_yaxis_transform())


def plot_detections(by_mode: Dict, out_base: str,
                    bg: Optional[Background] = None) -> List[str]:
    import matplotlib.colors as mcolors
    plt, fig, axes = _panels(len(by_mode))
    n_max = max([1] + [int(row[1]) for runs in by_mode.values()
                       for r in runs for row in r['landmarks']])
    cmap = plt.get_cmap('Oranges', n_max + 1)
    # discrete colours for n = 1..n_max, skipping the near-white n = 0 step
    colors = [cmap(i) for i in range(1, n_max + 1)]
    lcmap = mcolors.ListedColormap(colors)
    norm = mcolors.BoundaryNorm(np.arange(0.5, n_max + 1.5), lcmap.N)
    sc = None
    for ax, (mode, runs) in zip(axes, by_mode.items()):
        _draw_background(ax, bg)
        xy_all = []
        n_det = 0
        for r in runs:
            if r['gt']:
                g = np.asarray(r['gt'])[:, 1:3]
                ax.plot(g[:, 0], g[:, 1], color='#a8a7a2', lw=0.8, zorder=2)
                xy_all.append(g)
            rows = [row for row in r['landmarks'] if row[1] >= 1]
            if rows and r['gt']:
                p = positions_at(r['gt'], [row[0] for row in rows])
                sc = ax.scatter(p[:, 0], p[:, 1],
                                c=[row[1] for row in rows], cmap=lcmap,
                                norm=norm, s=14, edgecolors='none',
                                zorder=3)
                n_det += sum(row[1] for row in rows)
        _xy_limits(ax, bg, xy_all)
        _style(ax)
        ax.set_title('%s: %d detections in %d runs' % (mode, n_det,
                                                       len(runs)),
                     fontsize=10, color=INK)
        ax.set_xlabel('x [m] (map)', fontsize=9, color=INK_2)
    axes[0].set_ylabel('y [m] (map)', fontsize=9, color=INK_2)
    if sc is not None:
        cb = fig.colorbar(sc, ax=axes, shrink=0.7, pad=0.01,
                          ticks=range(1, n_max + 1))
        cb.set_label('n_detections per message', fontsize=9)
    fig.suptitle('Landmark detections along the GT path', fontsize=11,
                 color=INK)
    return _save(fig, out_base)


def make_all(by_mode: Dict, out_dir: str, align: str = 'none',
             bg: Optional[Background] = None,
             arrow_spacing: float = 0.5) -> List[str]:
    os.makedirs(out_dir, exist_ok=True)
    paths = []
    paths += plot_trajectories(by_mode, os.path.join(out_dir,
                                                     'trajectories'), bg,
                               arrow_spacing)
    paths += plot_ape_vs_time(by_mode, os.path.join(out_dir, 'ape_vs_time'),
                              align)
    paths += plot_detections(by_mode, os.path.join(out_dir,
                                                   'detections_path'), bg)
    return paths


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('root', nargs='?', help='directory with the run dirs')
    ap.add_argument('--runs', nargs='+', help='explicit run dirs (no root)')
    ap.add_argument('--out', help='output dir (default ROOT/plots)')
    ap.add_argument('--align', default='none')
    ap.add_argument('--no-crop', action='store_true',
                    help='whole recording; APE from evo_<align>_full/')
    ap.add_argument('--arrow-spacing', type=float, default=0.5,
                    help='est heading arrow every N m of path (0 = off)')
    ap.add_argument('--score-map', action='store_true',
                    help='S2* s* map as background')
    ap.add_argument('--background', help='PNG or .npy background')
    ap.add_argument('--extent', nargs=4, type=float,
                    metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'))
    a = ap.parse_args(argv)

    if bool(a.root) == bool(a.runs):
        ap.error('give either RUNS_ROOT or --runs')
    if a.score_map and a.background:
        ap.error('--score-map and --background are exclusive')
    if a.background and not a.extent:
        ap.error('--background needs --extent XMIN XMAX YMIN YMAX')
    run_dirs = a.runs if a.runs else sm.find_runs(a.root)
    if not run_dirs:
        print('ERROR: no run dirs found', file=sys.stderr)
        return 2
    bg = None
    if a.score_map:
        bg = score_map_background()
    elif a.background:
        bg = load_background(a.background, a.extent)
    by_mode = load_runs(run_dirs, a.align, crop=not a.no_crop)
    out = a.out or os.path.join(a.root or '.', 'plots')
    for p in make_all(by_mode, out, a.align, bg, a.arrow_spacing):
        print('wrote', p)
    return 0


if __name__ == '__main__':
    sys.exit(main())
