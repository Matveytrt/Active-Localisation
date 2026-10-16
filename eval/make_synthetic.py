#!/usr/bin/env python3
"""Generate synthetic runs (gt.tum + est.tum) to sanity-check run_evo.sh.

GT: circle r=2 m centred at the origin, 60 s, 50 Hz, heading tangent to the
circle, t0 = 100 s (sim-time-like). est: 10 Hz, stamps shifted by +4 ms with
respect to the GT grid (so time association/interpolation is exercised),
pose computed analytically at its own stamp, then perturbed:

  shift  est = GT + (0.10, 0, 0) m                -> APE(none)=0.10, RPE=0
  noise  est = GT + N(0, sigma^2) on x and y      -> APE RMSE ~ sigma*sqrt(2)
  drift  est = GT + (k * (t - t0), 0, 0), k=5 mm/s -> APE grows linearly

Usage:
  make_synthetic.py OUT_DIR                     # OUT_DIR/{shift,noise,drift}
  make_synthetic.py OUT_DIR --case noise --sigma 0.02 --seed 7 --name a_1
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'navigation', 'src', 'al_eval'))
from al_eval.tum import TumWriter, yaw_to_quat  # noqa: E402

R = 2.0
DURATION = 60.0
GT_HZ = 50.0
EST_HZ = 10.0
T0 = 100.0
EST_STAMP_SHIFT = 0.004
OMEGA = 2.0 * math.pi / DURATION  # one lap
Z = 0.0


def gt_pose(t: float):
    a = OMEGA * (t - T0)
    return R * math.cos(a), R * math.sin(a), a + math.pi / 2.0


def write_run(run_dir: str, case: str, sigma: float, seed: int,
              shift: float, drift_rate: float) -> None:
    rnd = random.Random(seed)
    with TumWriter(os.path.join(run_dir, 'gt.tum')) as w:
        n = int(round(DURATION * GT_HZ))
        for i in range(n + 1):
            t = T0 + i / GT_HZ
            x, y, yaw = gt_pose(t)
            w.write(t, x, y, Z, *yaw_to_quat(yaw))
    with TumWriter(os.path.join(run_dir, 'est.tum')) as w:
        n = int(round(DURATION * EST_HZ))
        for i in range(n):  # last est stamp stays inside the GT time range
            t = T0 + i / EST_HZ + EST_STAMP_SHIFT
            x, y, yaw = gt_pose(t)
            if case == 'shift':
                x += shift
            elif case == 'noise':
                x += rnd.gauss(0.0, sigma)
                y += rnd.gauss(0.0, sigma)
            elif case == 'drift':
                x += drift_rate * (t - T0)
            else:
                raise ValueError(case)
            w.write(t, x, y, Z, *yaw_to_quat(yaw))
    with open(os.path.join(run_dir, 'synthetic.txt'), 'w') as f:
        f.write('case=%s sigma=%g seed=%d shift=%g drift_rate=%g\n'
                % (case, sigma, seed, shift, drift_rate))
    print('wrote %s (case=%s)' % (run_dir, case))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('out_dir')
    ap.add_argument('--case', choices=['all', 'shift', 'noise', 'drift'],
                    default='all')
    ap.add_argument('--name', default=None,
                    help='run dir name (default: the case name)')
    ap.add_argument('--sigma', type=float, default=0.05)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--shift', type=float, default=0.10)
    ap.add_argument('--drift_rate', type=float, default=0.005,
                    help='m/s along x')
    a = ap.parse_args()

    cases = ['shift', 'noise', 'drift'] if a.case == 'all' else [a.case]
    if a.name and len(cases) > 1:
        ap.error('--name requires a single --case')
    for c in cases:
        write_run(os.path.join(a.out_dir, a.name or c), c, a.sigma, a.seed,
                  a.shift, a.drift_rate)
    return 0


if __name__ == '__main__':
    sys.exit(main())
