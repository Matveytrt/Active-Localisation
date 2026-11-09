"""Common Isaac execution profile; original algorithm YAMLs remain unchanged."""
import hashlib
import math
from pathlib import Path
import yaml

MODES = ('baseline', 'active', 'control')
SLAM_RESOURCES = {'Rtabmap/MemoryThr': '200', 'map_filter_radius': .15,
                  'map_filter_angle': 10., 'map_cache_loaded_on_init': False}
VARIANCE_KEYS = ('k_xx', 'k_xr', 'k_yy', 'k_yr', 'k_rr', 'k_rt')
BIAS_KEYS = ('scale_x', 'scale_y', 'scale_yaw', 'yaw_drift_per_m')


def noise_parameters(repo, seed, scale):
    validate_seed(seed)
    if isinstance(scale, bool) or not math.isfinite(scale) or not 0 < scale <= 20:
        raise ValueError('Noise scale must be finite in (0,20]')
    source = Path(repo)/'navigation/src/al_odom_noise/config/odom_noise.yaml'
    config = yaml.safe_load(source.read_text())
    params = config['odom_noise_node']['ros__parameters']
    params['seed'] = seed
    for key in VARIANCE_KEYS:
        params[key] *= scale**2
    for key in BIAS_KEYS:
        params[key] *= scale
    return config


def sha256(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as source:
        for chunk in iter(lambda: source.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def validate_seed(seed):
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**32-1:
        raise ValueError('Seed must be an integer in [0, 2**32-2]')
    return seed


def nav_parameters(repo, mode, landmarks):
    if mode not in MODES:
        raise ValueError('Unknown mode: '+str(mode))
    suffix = 'v2' if mode == 'baseline' else 'active'
    source = Path(repo)/f'navigation/src/nav2_planner_custom/config/nav2_params_{suffix}.yaml'
    config = yaml.safe_load(source.read_text())
    config['controller_server']['ros__parameters']['FollowPath'].update(
        vx_max=.6, vx_min=-.6, vy_max=.6, wz_max=.8)
    config['velocity_smoother']['ros__parameters'].update(
        max_velocity=[.6, .6, .8], min_velocity=[-.6, -.6, -.8])
    if mode != 'baseline':
        landmarks = Path(landmarks).resolve()
        data = yaml.safe_load(landmarks.read_text())
        assert data['frame_id'] == 'map' and data['source'] == 'optimized_graph'
        assert {m['id'] for m in data['big_markers']} == {1, 2, 3, 4, 5}
        planner = config['planner_server']['ros__parameters']['GridBased']
        layer = config['global_costmap']['global_costmap']['ros__parameters']['loc_layer']
        planner['landmarks_file'] = layer['landmarks_file'] = str(landmarks)
        if mode == 'control':
            planner['scorer'] = layer['scorer'] = 'null'
            layer['enabled'] = False
    return config, {'mode': mode, 'source': str(source), 'source_sha256': sha256(source),
                    'landmarks': str(landmarks), 'landmarks_sha256': sha256(landmarks)}
