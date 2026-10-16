#!/usr/bin/env python3
"""4-DoF (x, y, z-shift ignored, yaw) alignment of est to GT for planar robots.

evo's CLI offers only SE(3)/Sim(3) Umeyama (-a/-s) and --align_origin; a
yaw+translation ("posyaw") alignment is not available there. This helper
computes the closed-form 2D Umeyama (rotation about z + xy translation) on
time-associated poses and writes the transformed est trajectory, which
run_evo.sh then evaluates WITHOUT further alignment.

Prints the transform T with GT ~= T * est, and its inverse, i.e. the
gt_offset_{x,y,yaw} that would map GT into the est (map) frame.

Usage: align_posyaw.py GT.tum EST.tum OUT.tum [--t_max_diff 0.02]
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np
from evo.core import sync
from evo.tools import file_interface


def umeyama_2d(p: np.ndarray, q: np.ndarray):
    """Return (theta, t) minimising sum |p_i - (R(theta) q_i + t)|^2.

    p, q: (N, 2) arrays (reference, estimate).
    """
    pm = p.mean(axis=0)
    qm = q.mean(axis=0)
    pc = p - pm
    qc = q - qm
    num = np.sum(pc[:, 1] * qc[:, 0] - pc[:, 0] * qc[:, 1])
    den = np.sum(pc[:, 0] * qc[:, 0] + pc[:, 1] * qc[:, 1])
    theta = math.atan2(num, den)
    c, s = math.cos(theta), math.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    t = pm - rot @ qm
    return theta, t


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('gt')
    ap.add_argument('est')
    ap.add_argument('out')
    ap.add_argument('--t_max_diff', type=float, default=0.02)
    ap.add_argument('--sync_method', default='interpolation',
                    choices=['nearest_time', 'interpolation'])
    a = ap.parse_args()

    ref = file_interface.read_tum_trajectory_file(a.gt)
    est = file_interface.read_tum_trajectory_file(a.est)
    ref_s, est_s = sync.associate_trajectories(
        ref, est, sync_method=sync.SyncMethod(a.sync_method),
        max_diff=a.t_max_diff)
    theta, t = umeyama_2d(ref_s.positions_xyz[:, :2],
                          est_s.positions_xyz[:, :2])

    T = np.eye(4)
    c, s = math.cos(theta), math.sin(theta)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:2, 3] = t
    est.transform(T)  # left-multiplies every est pose (full trajectory)
    file_interface.write_tum_trajectory_file(a.out, est)

    # inverse: offset that maps GT into the est/map frame
    inv_t = -(T[:2, :2].T @ t)
    print('posyaw alignment on %d associated poses' % ref_s.num_poses)
    print('  GT ~= T*est : x=%.4f y=%.4f yaw=%.5f rad (%.3f deg)'
          % (t[0], t[1], theta, math.degrees(theta)))
    print('  inverse (gt_offset_x/y/yaw candidate): x=%.4f y=%.4f yaw=%.5f'
          % (inv_t[0], inv_t[1], -theta))
    print('  wrote %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
