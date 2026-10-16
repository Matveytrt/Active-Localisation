"""
STag detector: one instance. Pass a unique namespace and a config YAML per run.

Config YAML must contain:
  stag_detector_launch: { static_tf_parent_frame, static_tf_child_frame }  # launch only
  /**: { ros__parameters: { ... } }  # node parameters
Use different static_tf_child_frame (and parent if two cameras) per config when running two detectors.
"""

import tempfile
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    path = Path(config_file).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f'config_file not found: {path}')

    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f'Invalid YAML root in {path}')

    launch_cfg = data.get('stag_detector_launch') or {}
    if not isinstance(launch_cfg, dict):
        launch_cfg = {}
    parent = launch_cfg.get('static_tf_parent_frame', 'camera_link')
    child = launch_cfg.get('static_tf_child_frame', 'camera_optical_frame')
    parent = str(parent) if parent is not None else 'camera_link'
    child = str(child) if child is not None else 'camera_optical_frame'

    ros_block = data.get('/**')
    if not isinstance(ros_block, dict):
        raise ValueError(f'Missing /** block in {path}')
    ros_params = ros_block.get('ros__parameters')
    if not isinstance(ros_params, dict):
        raise ValueError(f'Missing /** ros__parameters in {path}')

    # Pass parameters via a YAML file path. A plain Python dict
    # parameters=[{'ros__parameters': ...}] often fails to apply types correctly
    # (e.g. show_detection stays false), so no detection_image publisher is created.
    with tempfile.NamedTemporaryFile(
        mode='w',
        suffix='.yaml',
        prefix='stag_detector_params_',
        delete=False,
        encoding='utf-8',
    ) as tfh:
        yaml.safe_dump(
            {'/**': {'ros__parameters': ros_params}},
            tfh,
            default_flow_style=False,
            allow_unicode=True,
            sort_keys=False,
        )
        params_tmp_path = tfh.name

    camera_optical_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_static_tf',
        namespace=LaunchConfiguration('namespace'),
        arguments=[
            '0', '0', '0',
            '-0.5', '0.5', '-0.5', '0.5',
            parent,
            child,
        ],
        output='screen',
    )

    detector_node = Node(
        package='stag_ros2_enhanced',
        executable='stag_detector_node',
        name='stag_detector',
        namespace=LaunchConfiguration('namespace'),
        parameters=[params_tmp_path],
        output='screen',
        emulate_tty=True,
    )

    return [camera_optical_tf_node, detector_node]


def generate_launch_description():
    pkg_share = FindPackageShare('stag_ros2_enhanced')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'detector_params_HD17.yaml',
        ]),
        description='Path to detector YAML (stag_detector_launch + /** ros__parameters)',
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='stag_detector',
        description='Unique namespace per instance (detector + static_transform_publisher)',
    )

    return LaunchDescription([
        config_file_arg,
        namespace_arg,
        OpaqueFunction(function=_launch_setup),
    ])
