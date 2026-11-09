"""Remote Linux entry point: fresh, paired Isaac episodes; no team containers."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
import traceback

REPO = Path(__file__).resolve().parents[1]
WORK = REPO/'isaac/runtime/navigation'
MAP = WORK/'maps/isaac-20261109-global.db'
FROZEN_SHA = '70a236237a60e4c4294d31ab308f449bdaa1e5be5dab5399742701a8ed6397fe'
OWN = ('al-isaac-episode', 'al-isaac-recorder', 'al-isaac-baseline', 'al-isaac-slam', 'al-isaac-fiducial',
       'al-isaac-astribot', 'al-isaac-odom-noise', 'al-isaac-lidar-stitching',
       'al-isaac-stag-hd17', 'al-isaac-stag-hd15')
ROS = 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && '


def sha(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for part in iter(lambda: stream.read(1024*1024), b''):
            result.update(part)
    return result.hexdigest()


def command(args, *, env=None, check=True, timeout=180):
    return subprocess.run(args, cwd=REPO, env=env, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, check=check, timeout=timeout)


def state(name):
    proc = command(['docker', 'inspect', name], check=False)
    return json.loads(proc.stdout)[0] if proc.returncode == 0 else None


def running(name):
    value = state(name)
    return value is not None and value['State']['Running']


def ros(args, *, timeout=180, check=True):
    import shlex
    return command(['docker', 'exec', 'al-isaac-navigation', 'bash', '-lc',
                    ROS+'exec '+shlex.join(args)], timeout=timeout, check=check)


def compose(env, *args):
    proc = command(['docker', 'compose', '-f', 'isaac/compose.yaml', '--profile', 'slam',
                   '--profile', 'nav', '--profile', 'experiment', *args], env=env)
    print(proc.stdout, flush=True)


def cleanup(output):
    """Only the containers preflight reserved for this runner. Preserve evidence."""
    records = {}
    for name in OWN:
        if name == 'al-isaac-astribot' and running(name):
            proc = ros(['ros2', 'service', 'call', '/isaac/shutdown', 'std_srvs/srv/Trigger', '{}'],
                       timeout=20, check=False)
            (output/'shutdown-request.log').write_text(proc.stdout)
            deadline = time.monotonic()+60
            while running(name) and time.monotonic() < deadline:
                time.sleep(1)
        if running(name):
            stop = ['docker', 'stop']
            if name != 'al-isaac-slam':
                stop += ['-s', 'SIGINT']
            command(stop+['-t', '120' if name == 'al-isaac-slam' else '30', name], timeout=150)
        value = state(name)
        if value is not None:
            (output/(name+'.log')).write_text(command(['docker', 'logs', name], check=False).stdout)
            # Do not export host paths, usernames or container inspection secrets.
            records[name] = {'state': value['State'], 'image': value['Image']}
            records[name]['state'].pop('Error', None)
    (output/'containers.json').write_text(json.dumps(records, indent=2))
    bridge = REPO/'isaac/runtime/bridge'
    for filename in ('ready.json', 'result.json'):
        if (bridge/filename).is_file():
            shutil.copy2(bridge/filename, output/('bridge-'+filename))
    return records


def execute(args):
    assert re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', args.batch), 'Unsafe batch name'
    assert 0 <= args.seed < 2**32-1, 'Seed out of range'
    assert len(set(args.modes)) == len(args.modes), 'Repeated modes'
    assert sha(MAP) == FROZEN_SHA, 'Pristine map differs'
    assert shutil.disk_usage(WORK).free > MAP.stat().st_size*len(args.modes)+1024**3, 'Insufficient disk'
    assert not any(running(name) for name in OWN), 'Own services are busy; refusing to interfere'
    target = WORK/'experiments'/args.batch
    assert not target.exists(), 'Batch already exists; never overwrite evidence'
    # Validate original scenario/config through the existing ROS workspace before mutation.
    scenario_name = getattr(args, 'scenario', 'shelves_south_wall')
    assert scenario_name in ('shelves_south_wall', 'shelves_loop_a')
    scenario = '/repo/navigation/src/al_eval/config/scenarios/'+scenario_name+'.yaml'
    noise_scale = getattr(args, 'noise_scale', 1.)
    ros(['python3', '-c', "from al_eval.scenario import load_scenario; print(load_scenario('"+scenario+"'))"])
    target.mkdir(parents=True)
    maps = WORK/'maps/episodes'/args.batch
    maps.mkdir(parents=True)
    manifest = {'status': 'running', 'map_sha256': FROZEN_SHA,
        'landmarks_sha256': sha(MAP.with_suffix('.landmarks.yaml')),
        'scenario': scenario, 'scenario_sha256': sha(REPO/scenario.removeprefix('/repo/')),
        'git_head': command(['git', 'rev-parse', 'HEAD']).stdout.strip(),
        'git_status': command(['git', 'status', '--short']).stdout,
        'seed': args.seed, 'noise_scale': noise_scale, 'lidar_seeds': [args.seed, args.seed+1], 'mppi_rng': 'not controlled',
        'known_spawn': [-2., -.5, .16, 0.], 'episodes': []}
    tracked = command(['git', 'ls-files', '-m', '-o', '--exclude-standard']).stdout.splitlines()
    manifest['uncommitted_sha256'] = {p: sha(REPO/p) for p in tracked if (REPO/p).is_file()}
    (target/'manifest.json').write_text(json.dumps(manifest, indent=2))
    (target/'versions.log').write_text(ros(['bash', '-lc', 'ros2 pkg xml nav2_mppi_controller; ros2 pkg xml rtabmap_slam']).stdout)
    try:
        for index, mode in enumerate(args.modes):
            name = mode+'_01'
            output = target/name
            output.mkdir()
            database = maps/(name+'.db')
            shutil.copyfile(MAP, database)
            assert sha(database) == FROZEN_SHA
            env = dict(os.environ, AL_ISAAC_SEED=str(args.seed), AL_ISAAC_NAV_MODE=mode,
                AL_ISAAC_NOISE_SCALE=str(noise_scale),
                AL_ISAAC_RUN_DIR='/workspace/experiments/'+args.batch+'/'+name,
                AL_ISAAC_SLAM_MODE='localization', AL_ISAAC_DB='/workspace/maps/episodes/'+args.batch+'/'+name+'.db',
                AL_ISAAC_LANDMARKS='/workspace/maps/isaac-20261109-global.landmarks.yaml')
            entry = {'mode': mode, 'initial_db_sha256': sha(database), 'status': 'running'}
            manifest['episodes'].append(entry)
            (target/'manifest.json').write_text(json.dumps(manifest, indent=2))
            print('EPISODE_START', name, flush=True)
            try:
                compose(env, 'up', '-d', '--force-recreate', 'fiducial', 'slam', 'recorder')
                started = time.monotonic()
                (output/'start-bridge.log').write_text(command(['bash', 'isaac/start_bridge.sh', '--scene', 'own-world'], env=env).stdout)
                deadline = time.monotonic()+240
                while not (REPO/'isaac/runtime/bridge/ready.json').exists():
                    assert running('al-isaac-astribot'), 'Simulator exited during startup'
                    assert time.monotonic() < deadline, 'Simulator readiness timeout'
                    time.sleep(1)
                ready = json.loads((REPO/'isaac/runtime/bridge/ready.json').read_text())
                assert ready['seeds'] == {'front': args.seed, 'back': args.seed+1}, ready
                entry['startup_wall_s'] = time.monotonic()-started
                compose(env, 'up', '-d', '--force-recreate', 'baseline')
                previous = state('al-isaac-episode')
                if previous is not None:
                    assert not previous['State']['Running']
                    command(['docker', 'rm', 'al-isaac-episode'])
                import shlex
                episode_args = ['python3', '/repo/isaac/episode.py', '--output', env['AL_ISAAC_RUN_DIR'],
                    '--scenario', scenario, '--mode', mode, '--seed', str(args.seed)]
                if getattr(args, 'complete_scenario', False):
                    episode_args.append('--complete-scenario')
                # Own process container: a host timeout/cancel cannot leave a
                # docker-exec child sending goals during the next episode.
                result = command(['docker', 'compose', '-f', 'isaac/compose.yaml', 'run',
                    '--name', 'al-isaac-episode', '--no-deps', '-T', 'navigation', 'bash', '-lc',
                    ROS+'exec '+shlex.join(episode_args)], env=env, timeout=1000, check=False)
                (output/'episode.log').write_text(result.stdout)
                assert result.returncode == 0, 'Episode failed; see episode.log'
                entry['status'] = 'passed'
            except BaseException:
                entry['status'], entry['error'] = 'failed', traceback.format_exc()
                raise
            finally:
                records = cleanup(output)
                clean = all(not r['state']['OOMKilled'] and r['state']['ExitCode'] == 0 for r in records.values())
                entry['clean_shutdown'] = clean
                entry['pristine_after_sha256'] = sha(MAP)
                (target/'manifest.json').write_text(json.dumps(manifest, indent=2))
                assert entry['pristine_after_sha256'] == FROZEN_SHA, 'Pristine changed'
                if entry['status'] == 'passed':
                    assert clean, 'Unclean shutdown; inspect containers.json'
            print('EPISODE_FINISHED', name, flush=True)
        manifest['status'] = 'passed'
    except BaseException:
        manifest['status'], manifest['error'] = 'failed', traceback.format_exc()
        raise
    finally:
        (target/'manifest.json').write_text(json.dumps(manifest, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--batch', required=True)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--scenario', choices=['shelves_south_wall', 'shelves_loop_a'], default='shelves_south_wall')
    parser.add_argument('--noise-scale', type=float, choices=[1., 4., 6.], default=1.)
    parser.add_argument('--complete-scenario', action='store_true')
    parser.add_argument('--modes', nargs='+', choices=['baseline', 'active', 'control'], default=['baseline', 'active', 'control'])
    args = parser.parse_args()
    os.chdir(REPO)
    lock = (WORK/'experiment.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    execute(args)


if __name__ == '__main__':
    main()
