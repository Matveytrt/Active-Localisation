"""Preregistered reproduction of the supplied Gazebo protocol in Isaac."""
import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import sys
import time
from run_experiment import REPO, WORK, OWN, running, sha, MAP, FROZEN_SHA

ORDERS = [('baseline', 'active', 'control'), ('active', 'control', 'baseline'),
          ('control', 'baseline', 'active')]


def protocol():
    jobs = []
    for level_index, scale in enumerate((4, 6)):
        for index, seed in enumerate(range(42, 47)):
            for position, mode in enumerate(ORDERS[(index+level_index) % 3]):
                jobs.append({'noise_scale': scale, 'seed': seed, 'mode': mode,
                             'position': position+1})
    return jobs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    args = parser.parse_args()
    assert args.name.replace('-', '').isalnum(), 'Unsafe series name'
    root = WORK/'series'/args.name
    assert not root.exists(), 'Never overwrite a series'
    lock = (WORK/'series.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    assert not any(running(n) for n in OWN), 'Own services busy'
    assert sha(MAP) == FROZEN_SHA
    root.mkdir(parents=True)
    jobs = protocol()
    manifest = {'status': 'running', 'scenario': 'shelves_loop_a', 'jobs': jobs,
        'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip(),
        'map_sha256': FROZEN_SHA, 'policy': 'No selective reruns or algorithm tuning; all failures retained',
        'evaluation': {'evo': '1.38.0', 'plane': 'xy', 'align': 'none',
                       'sync': 'interpolation', 'max_time_difference_s': .02, 'rpe_delta_m': 1.}}
    path = root/'manifest.json'
    path.write_text(json.dumps(manifest, indent=2))
    failures = 0
    for index, job in enumerate(jobs):
        batch = f'{args.name}-k{job["noise_scale"]}-s{job["seed"]}-{job["mode"]}'
        output = root/(batch+'.log')
        job.update(batch=batch, status='running')
        path.write_text(json.dumps(manifest, indent=2))
        print(f'SERIES_JOB {index+1}/{len(jobs)} {batch}', flush=True)
        start = time.monotonic()
        with output.open('w') as log:
            proc = subprocess.run([sys.executable, '-u', str(REPO/'isaac/run_experiment.py'),
                '--batch', batch, '--seed', str(job['seed']), '--modes', job['mode'],
                '--scenario', 'shelves_loop_a', '--noise-scale', str(job['noise_scale']), '--complete-scenario'],
                cwd=REPO, stdout=log, stderr=subprocess.STDOUT)
        job.update(status='passed' if proc.returncode == 0 else 'failed',
                   exit_code=proc.returncode, wall_seconds=time.monotonic()-start)
        failures += proc.returncode != 0
        path.write_text(json.dumps(manifest, indent=2))
        if any(running(n) for n in OWN):
            manifest['status'] = 'blocked-busy-after-job'
            path.write_text(json.dumps(manifest, indent=2))
            raise RuntimeError('Previous job left own services running; inspect without interference')
        assert sha(MAP) == FROZEN_SHA
    manifest.update(status='complete', failures=failures)
    path.write_text(json.dumps(manifest, indent=2))
    print('SERIES_COMPLETE', 'failures', failures, flush=True)


if __name__ == '__main__':
    main()
