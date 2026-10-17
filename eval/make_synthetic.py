#!/usr/bin/env python3
"""Generate synthetic runs to sanity-check run_evo.sh / summarize / plots.

GT: circle r=2 m centred at the origin, 60 s, 50 Hz, heading tangent to the
circle, t0 = 100 s (sim-time-like). est: 10 Hz, stamps shifted by +4 ms with
respect to the GT grid (so time association/interpolation is exercised),
pose computed analytically at its own stamp, then perturbed:

  shift  est = GT + (0.10, 0, 0) m                -> APE(none)=0.10, RPE=0
  noise  est = GT + N(0, sigma^2) on x and y      -> APE RMSE ~ sigma*sqrt(2)
  drift  est = GT + (k * (t - t0), 0, 0), k=5 mm/s -> APE grows linearly

Experiment case (``--case experiment``): <mode>_<NN> run dirs for the
modes baseline / active / control x ``--runs`` (default 3), each with the
files a real run has: gt.tum, est.tum, landmarks.csv, map_odom.tum,
result.yaml (al_eval.scenario format), run.yaml (run_experiment.sh keys).
GT: ellipse centred (-0.8, -0.6), semi-axes 3.0 x 1.1 m (inside the room,
south of the shelf rows), one lap; 60 s (baseline/control), 66 s (active).
Error model (translation + yaw, 10 Hz ticks; a toy, not a simulator):

  odometry drift d += v dt + random walk     (v ~ 6 mm/s, seeded per run)
  correction     c += alpha (d - c)          alpha = 0.25 on a tick with a
                                             landmark detection, 0.02 else
                                             (lidar ICP)
  est = GT + (d - c) + N(0, 0.01^2);  map->odom = -c  (pure translation +
  yaw, written as map_odom.tum)

Detections happen in arcs of the lap (wider arcs for active), with a
per-tick probability 0.9; ids depend on the arc. Run baseline_03 has one
aborted goal so that "goals succeeded" is exercised.

Usage:
  make_synthetic.py OUT_DIR                     # OUT_DIR/{shift,noise,drift}
  make_synthetic.py OUT_DIR --case noise --sigma 0.02 --seed 7 --name a_1
  make_synthetic.py OUT_DIR --case experiment [--runs 3] [--seed 0]
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'navigation', 'src', 'al_eval'))
from al_eval import scenario as sc  # noqa: E402
from al_eval.recorder_helpers import LandmarkCsvWriter  # noqa: E402
from al_eval.tum import TumWriter, wrap_angle, yaw_to_quat  # noqa: E402

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


# ------------------------------------------------------------------ experiment
EXP_MODES = ('baseline', 'active', 'control')
ELLIPSE = {'cx': -0.8, 'cy': -0.6, 'a': 3.0, 'b': 1.1}
EXP_DURATION = {'baseline': 60.0, 'active': 66.0, 'control': 60.0}
# Arcs of the lap phase [rad] with landmark detections, and their ids.
EXP_ARCS = {
    'baseline': [(0.4, 1.2, (1,)), (3.4, 4.0, (3, 4))],
    'control': [(0.4, 1.2, (1,)), (3.4, 4.0, (3, 4))],
    'active': [(0.1, 1.7, (1, 2)), (2.9, 4.6, (3, 4)), (5.4, 6.0, (5,))],
}
ALPHA_DET = 0.25
ALPHA_ICP = 0.02
EST_NOISE = 0.01
N_GOALS = 4


def ellipse_pose(phase: float):
    """(x, y, yaw) on the experiment ellipse, heading along the path."""
    e = ELLIPSE
    x = e['cx'] + e['a'] * math.cos(phase)
    y = e['cy'] + e['b'] * math.sin(phase)
    yaw = math.atan2(e['b'] * math.cos(phase), -e['a'] * math.sin(phase))
    return x, y, yaw


def _arc_length(p0: float, p1: float, n: int = 2000) -> float:
    pts = [ellipse_pose(p0 + (p1 - p0) * k / n)[:2] for k in range(n + 1)]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(pts, pts[1:]))


def write_experiment_run(run_dir: str, mode: str, index: int,
                         seed: int) -> None:
    rnd = random.Random(seed * 1000 + EXP_MODES.index(mode) * 100 + index)
    duration = EXP_DURATION[mode]
    omega = 2.0 * math.pi / duration

    def phase(t: float) -> float:
        return omega * (t - T0)

    with TumWriter(os.path.join(run_dir, 'gt.tum')) as w:
        for i in range(int(round(duration * GT_HZ)) + 1):
            t = T0 + i / GT_HZ
            x, y, yaw = ellipse_pose(phase(t))
            w.write(t, x, y, Z, *yaw_to_quat(yaw))

    # per-run jitter of the detection arcs (seeded) and odometry drift
    arcs = [(lo + rnd.uniform(-0.1, 0.1), hi + rnd.uniform(-0.1, 0.1), ids)
            for lo, hi, ids in EXP_ARCS[mode]]
    ang = rnd.uniform(0.0, 2.0 * math.pi)
    v = (0.006 * math.cos(ang), 0.006 * math.sin(ang))
    v_yaw = rnd.choice((-1.0, 1.0)) * 0.002
    d = [0.0, 0.0, 0.0]
    c = [0.0, 0.0, 0.0]
    dt = 1.0 / EST_HZ
    est_w = TumWriter(os.path.join(run_dir, 'est.tum'))
    mo_w = TumWriter(os.path.join(run_dir, 'map_odom.tum'))
    lm_w = LandmarkCsvWriter(os.path.join(run_dir, 'landmarks.csv'))
    try:
        for i in range(int(round(duration * EST_HZ))):
            t = T0 + i / EST_HZ + EST_STAMP_SHIFT
            ph = phase(t) % (2.0 * math.pi)
            ids = ()
            for lo, hi, arc_ids in arcs:
                if lo <= ph <= hi and rnd.random() < 0.9:
                    ids = arc_ids
            d[0] += v[0] * dt + rnd.gauss(0.0, 0.002)
            d[1] += v[1] * dt + rnd.gauss(0.0, 0.002)
            d[2] += v_yaw * dt + rnd.gauss(0.0, 0.0005)
            alpha = ALPHA_DET if ids else ALPHA_ICP
            for k in range(3):
                c[k] += alpha * (d[k] - c[k])
            x, y, yaw = ellipse_pose(phase(t))
            ex, ey, eyaw = d[0] - c[0], d[1] - c[1], d[2] - c[2]
            est_w.write(t, x + ex + rnd.gauss(0.0, EST_NOISE),
                        y + ey + rnd.gauss(0.0, EST_NOISE), Z,
                        *yaw_to_quat(wrap_angle(yaw + eyaw)))
            mo_w.write(t, -c[0], -c[1], 0.0, *yaw_to_quat(-c[2]))
            if ids:
                lm_w.write(t, ids)
    finally:
        est_w.close()
        mo_w.close()
        lm_w.close()

    # result.yaml: four goals at the quarter points of the lap
    goals = [sc.Goal(*ellipse_pose(2.0 * math.pi * (k + 1) / N_GOALS))
             for k in range(N_GOALS)]
    scen = sc.Scenario(name='synthetic_loop', frame_id='map',
                       goal_timeout_s=120.0, settle_s=0.0, goals=goals)
    name = '%s_%02d' % (mode, index)
    res = sc.RunResult.for_scenario(scen, run_name=name, mode=mode,
                                    seed=42 + index - 1,
                                    start_wall='2026-10-18T00:00:00',
                                    end_wall='2026-10-18T00:01:10')
    for k, g in enumerate(res.goals):
        g.status = 'succeeded'
        g.duration_sim_s = duration / N_GOALS
        g.duration_wall_s = 1.05 * duration / N_GOALS
        g.path_length_m = _arc_length(2.0 * math.pi * k / N_GOALS,
                                      2.0 * math.pi * (k + 1) / N_GOALS)
        g.recoveries = 0
        g.set_final_pose(g.x + 0.05, g.y, g.yaw)
    if mode == 'baseline' and index == 3:
        res.goals[2].status = 'aborted'
        res.goals[2].message = 'synthetic failure'
    sc.write_result(os.path.join(run_dir, 'result.yaml'), res)

    run_yaml = {
        'run_name': name, 'mode': mode, 'run_index': index,
        'seed': 42 + index - 1, 'scenario': 'synthetic_loop',
        'scenario_file': 'synthetic', 'git_commit': 'synthetic',
        'git_dirty': False, 'start_wall': res.start_wall,
        'end_wall': res.end_wall, 'runner_exit_code': res.exit_code,
        'error': '', 'launch': {},
    }
    sc.write_yaml(os.path.join(run_dir, 'run.yaml'), run_yaml)
    with open(os.path.join(run_dir, 'synthetic.txt'), 'w') as f:
        f.write('case=experiment mode=%s index=%d seed=%d duration=%g\n'
                % (mode, index, seed, duration))
    print('wrote %s (experiment, %s)' % (run_dir, mode))


def write_experiment(out_dir: str, runs: int, seed: int,
                     modes=EXP_MODES) -> list:
    """All <mode>_<NN> run dirs; returns their paths."""
    paths = []
    for mode in modes:
        for i in range(1, runs + 1):
            p = os.path.join(out_dir, '%s_%02d' % (mode, i))
            write_experiment_run(p, mode, i, seed)
            paths.append(p)
    return paths


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('out_dir')
    ap.add_argument('--case', choices=['all', 'shift', 'noise', 'drift',
                                       'experiment'],
                    default='all',
                    help="'all' = shift, noise and drift; 'experiment' = "
                         '3 modes x --runs run dirs')
    ap.add_argument('--name', default=None,
                    help='run dir name (default: the case name)')
    ap.add_argument('--sigma', type=float, default=0.05)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--shift', type=float, default=0.10)
    ap.add_argument('--drift_rate', type=float, default=0.005,
                    help='m/s along x')
    ap.add_argument('--runs', type=int, default=3,
                    help='runs per mode for --case experiment')
    a = ap.parse_args()

    if a.case == 'experiment':
        if a.name:
            ap.error('--name is not used with --case experiment')
        if a.runs < 1:
            ap.error('--runs must be >= 1')
        write_experiment(a.out_dir, a.runs, a.seed)
        return 0
    cases = ['shift', 'noise', 'drift'] if a.case == 'all' else [a.case]
    if a.name and len(cases) > 1:
        ap.error('--name requires a single --case')
    for c in cases:
        write_run(os.path.join(a.out_dir, a.name or c), c, a.sigma, a.seed,
                  a.shift, a.drift_rate)
    return 0


if __name__ == '__main__':
    sys.exit(main())
