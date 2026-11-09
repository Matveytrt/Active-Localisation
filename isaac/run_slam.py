"""Run original algorithms with explicit Isaac paths and common base limits."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_prefix

parser = argparse.ArgumentParser()
parser.add_argument('component', choices=('slam', 'fiducial', 'baseline'))
args = parser.parse_args()
repo = Path(__file__).resolve().parents[1]
output = Path('/workspace/slam-config')
output.mkdir(exist_ok=True)
mode = os.environ.get('AL_ISAAC_SLAM_MODE', 'mapping')
assert mode in ('mapping', 'localization')
remaps = []
if args.component == 'slam':
    source = repo/'navigation/src/rtabmap_landmarks/config/rtabmap_lidar_landmarks_params.yaml'
    config = yaml.safe_load(source.read_text())
    launch = config['launch_config']['ros__parameters']
    params = config['/**']['ros__parameters']
    params.update({k: str(v) for k, v in launch[mode+'_params'].items()})
    # Bound working/cache memory on the 16 GiB host. Nodes still go to the
    # database; do not discard raw sensor data or replace the 3-D map with GT.
    params.update({'Rtabmap/MemoryThr': '200', 'map_filter_radius': .15,
                   'map_filter_angle': 10., 'map_cache_loaded_on_init': False})
    db = Path(os.environ['AL_ISAAC_DB']).resolve()
    assert db.is_absolute() and db.is_relative_to('/workspace/maps'), 'Use an own map under /workspace/maps'
    db.parent.mkdir(parents=True, exist_ok=True)
    if mode == 'localization':
        assert db.is_file(), 'Localization needs an existing own map'
    params['database_path'] = str(db)
    config = {'/**': {'ros__parameters': params}}
    for old, new in (('odom', launch['odom_topic']), ('scan_cloud', launch['lidar_topic']),
                     ('landmark_detections', launch['landmark_detections_topic']), ('map', launch['rtabmap_map_topic']),
                     ('initialpose', '/initialpose'), ('imu', 'imu_not_used')):
        remaps.extend(['-r', old+':='+new])
    package, executable, node_name = 'rtabmap_slam', 'rtabmap', 'rtabmap'
elif args.component == 'fiducial':
    source = repo/'navigation/src/tf_to_fiducial/config/tf_to_fiducial_params.yaml'
    config = yaml.safe_load(source.read_text())
    config['tf_to_fiducial_node']['ros__parameters']['operation_mode'] = mode
    package, executable, node_name = 'tf_to_fiducial', 'tf_to_fiducial_node', 'tf_to_fiducial_node'
else:
    source = repo/'navigation/src/nav2_planner_custom/config/nav2_params_v2.yaml'
    config = yaml.safe_load(source.read_text())
    controller = config['controller_server']['ros__parameters']['FollowPath']
    controller.update(vx_max=.6, vx_min=-.6, vy_max=.6, wz_max=.8)
    smoother = config['velocity_smoother']['ros__parameters']
    smoother.update(max_velocity=[.6, .6, .8], min_velocity=[-.6, -.6, -.8])
    config_path = output/'baseline.yaml'
    config_path.write_text(yaml.safe_dump(config))
    (output/'baseline-provenance.json').write_text(json.dumps({'source': str(source),
        'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'overrides': 'MPPI and smoother velocities match the Isaac planar base limits: .6/.6/.8'}, indent=2))
    os.execvp('ros2', ['ros2', 'launch', 'nav2_planner_custom', 'navigation_launch.py', 'params_file:='+str(config_path)])
config_path = output/(args.component+'-'+mode+'.yaml')
config_path.write_text(yaml.safe_dump(config))
path = Path(get_package_prefix(package))/'lib'/package/executable
print(f'{args.component}: source={source}; mode={mode}; config={config_path}', flush=True)
os.execv(str(path), [str(path), '--ros-args', '--params-file', str(config_path), '-r', '__node:='+node_name, *remaps])
