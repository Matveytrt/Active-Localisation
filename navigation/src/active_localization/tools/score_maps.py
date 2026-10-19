#!/usr/bin/env python3
"""Offline S2* score maps for the report (spec T-009 §2, §3).

    python3 -I tools/score_maps.py --out <dir>

Writes into <dir>:
  s_star.png      best-over-yaw score s*(x, y) with markers, normals, shelf rows
  layer_cost.png  §3 LocalizabilityLayer cost (c_max = 40, costmap units)
  best_yaw.png    argmax-yaw field (quiver, subsampled) where s* > 0.05
and prints a few numbers (free-area fraction with s* > 0.5, band in front of id 1).

"free" = room box minus the shelf rectangles (no walls, no inflation) [assumption].
Poses are base_link poses; the camera sits 0.07 m ahead of the base.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from _common import LAYER, ROOM, build_scorer, room_grid  # noqa: E402
from active_localization.cost_field import layer_cost_from_score  # noqa: E402


def best_over_yaw(scorer, grid, yaw_bins):
    X, Y = grid.cell_centers()
    yaws = -np.pi + 2.0 * np.pi * np.arange(yaw_bins) / yaw_bins
    best = np.full(X.shape, -np.inf)
    arg = np.zeros(X.shape)
    for yaw in yaws:
        s = scorer.score(X, Y, yaw)
        better = s > best
        best = np.where(better, s, best)
        arg = np.where(better, yaw, arg)
    return X, Y, best, arg


def free_mask(X, Y, rows):
    m = np.ones(X.shape, dtype=bool)
    for xmin, xmax, ymin, ymax in rows:
        m &= ~((X >= xmin) & (X <= xmax) & (Y >= ymin) & (Y <= ymax))
    return m


def runs(mask, values):
    """(lo, hi) of `values` over the contiguous True run(s) of a 1D mask."""
    out, start = [], None
    for i, v in enumerate(list(mask) + [False]):
        if v and start is None:
            start = i
        elif not v and start is not None:
            out.append((values[start], values[i - 1]))
            start = None
    return out


def band_numbers(scorer, X, Y, best, cost, marker_id=1):
    i = int(np.where(scorer.lm.ids == marker_id)[0][0])
    mx, my = scorer.lm.xy[i]
    col = int(np.argmin(np.abs(X[0] - mx)))
    ys = Y[:, col]
    front = ys < my
    h = my - ys[front]                       # distance base -> marker face along -y
    order = np.argsort(h)
    h = h[order]
    s_col = best[front, col][order]
    c_col = cost[front, col][order]
    res = {'marker': marker_id, 'marker_xy': (float(mx), float(my)),
           'column_x': float(X[0, col]),
           'h_s_gt_0.5': runs(s_col > 0.5, h),
           'h_cost_le_10': runs(c_col <= 10, h),
           'h_peak': float(h[int(np.argmax(s_col))]), 's_peak': float(s_col.max())}
    row = int(np.argmin(np.abs(Y[:, 0] - (my - 1.2))))
    dx = X[row] - mx
    near = np.abs(dx) < 2.5
    res['x_at_h1.2_s_gt_0.5'] = runs((best[row] > 0.5) & near, dx)
    res['h_row'] = float(my - Y[row, 0])
    return res


def draw_scene(ax, scorer):
    import matplotlib.patches as mpatches
    for xmin, xmax, ymin, ymax in scorer.occlusion_rows:
        ax.add_patch(mpatches.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                                        fill=True, facecolor='#d9d9d9', edgecolor='#555555',
                                        lw=1.0, zorder=3))
    for lid, p, n in zip(scorer.lm.ids, scorer.lm.xy, scorer.normals):
        ax.plot(p[0], p[1], 'o', ms=8, mfc='#111111', mec='white', mew=1.5, zorder=5)
        ax.annotate('', xy=(p[0] + 0.5 * n[0], p[1] + 0.5 * n[1]), xytext=(p[0], p[1]),
                    arrowprops=dict(arrowstyle='->', color='#111111', lw=1.5), zorder=5)
        ax.text(p[0] + 0.08, p[1] + 0.12, f'id {lid}', fontsize=9, color='#111111', zorder=6)
    ax.set_xlim(ROOM['xmin'], ROOM['xmax'])
    ax.set_ylim(ROOM['ymin'], ROOM['ymax'])
    ax.set_aspect('equal')
    ax.set_xlabel('x [m] (map)')
    ax.set_ylabel('y [m] (map)')
    ax.grid(color='#cccccc', lw=0.4, alpha=0.6)


def plot_all(out, scorer, grid, X, Y, best, arg, cost):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    ext = (grid.origin_x, grid.origin_x + grid.width * grid.resolution,
           grid.origin_y, grid.origin_y + grid.height * grid.resolution)
    paths = []

    fig, ax = plt.subplots(figsize=(9, 8))
    im = ax.imshow(best, origin='lower', extent=ext, cmap='Blues', vmin=0, vmax=0.7)
    ax.contour(X, Y, best, levels=[0.5], colors='#08306b', linewidths=1.0)
    draw_scene(ax, scorer)
    fig.colorbar(im, ax=ax, shrink=0.8, label='s* = max over 36 yaws of s(x, y, yaw)')
    ax.set_title('S2* best-over-yaw score (contour: s* = 0.5); base_link position')
    paths.append(os.path.join(out, 's_star.png'))
    fig.savefig(paths[-1], dpi=130, bbox_inches='tight')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 8))
    im = ax.imshow(cost, origin='lower', extent=ext, cmap='Oranges', vmin=0,
                   vmax=LAYER['c_max'])
    draw_scene(ax, scorer)
    fig.colorbar(im, ax=ax, shrink=0.8,
                 label=f'layer cost (costmap units, c_max = {LAYER["c_max"]})')
    ax.set_title('LocalizabilityLayer cost (spec §3): round(c_max (1 - min(s*/s_ref, 1)))')
    paths.append(os.path.join(out, 'layer_cost.png'))
    fig.savefig(paths[-1], dpi=130, bbox_inches='tight')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 8))
    ax.imshow(best, origin='lower', extent=ext, cmap='Blues', vmin=0, vmax=0.7, alpha=0.6)
    k = 5
    sub = (slice(k // 2, None, k), slice(k // 2, None, k))
    m = best[sub] > 0.05
    ax.quiver(X[sub][m], Y[sub][m], np.cos(arg[sub][m]), np.sin(arg[sub][m]),
              color='#111111', scale=40, width=0.0025, zorder=4)
    draw_scene(ax, scorer)
    ax.set_title(f'argmax-yaw heading where s* > 0.05 (every {k}th cell)')
    paths.append(os.path.join(out, 'best_yaw.png'))
    fig.savefig(paths[-1], dpi=130, bbox_inches='tight')
    plt.close(fig)
    return paths


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', required=True, help='output directory for the PNGs')
    ap.add_argument('--no-plots', action='store_true', help='numbers only')
    a = ap.parse_args(argv)

    scorer = build_scorer()
    grid = room_grid()
    X, Y, best, arg = best_over_yaw(scorer, grid, LAYER['yaw_bins'])
    cost = layer_cost_from_score(best, c_max=LAYER['c_max'], power=LAYER['power'])
    free = free_mask(X, Y, scorer.occlusion_rows)

    print(f'grid {grid.width} x {grid.height} cells at {grid.resolution} m, '
          f'{int(free.sum())} free cells')
    for thr in (0.5, 0.3, 0.1):
        print(f'fraction of free area with s* > {thr}: {np.mean(best[free] > thr):.4f}')
    print(f'fraction of free area with cost < c_max: {np.mean(cost[free] < LAYER["c_max"]):.4f}'
          f'; cost == 0: {np.mean(cost[free] == 0):.4f}; max s* = {best.max():.4f}')
    b = band_numbers(scorer, X, Y, best, cost, marker_id=1)
    fmt = '; '.join
    print(f'band in front of id 1 (marker at x={b["marker_xy"][0]:.3f}, y={b["marker_xy"][1]:.3f};'
          f' column x={b["column_x"]:.3f}; h = marker y - base y):')
    print('  s* > 0.5 for h in ' + fmt(f'[{lo:.3f}, {hi:.3f}] m' for lo, hi in b['h_s_gt_0.5']))
    print('  cost <= 10 for h in ' + fmt(f'[{lo:.3f}, {hi:.3f}] m'
                                         for lo, hi in b['h_cost_le_10']))
    print(f'  peak s* = {b["s_peak"]:.4f} at h = {b["h_peak"]:.3f} m')
    print(f'  at h = {b["h_row"]:.3f} m: s* > 0.5 for x - x_marker in '
          + fmt(f'[{lo:.3f}, {hi:.3f}] m' for lo, hi in b['x_at_h1.2_s_gt_0.5']))

    if not a.no_plots:
        os.makedirs(a.out, exist_ok=True)
        for p in plot_all(a.out, scorer, grid, X, Y, best, arg, cost):
            print('wrote', p)
    return 0


if __name__ == '__main__':
    sys.exit(main())
