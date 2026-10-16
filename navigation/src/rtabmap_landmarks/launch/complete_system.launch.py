"""Complete system launch: STag detectors + tf_to_fiducial + RTAB-Map + marker visualization.

Симуляция запускается отдельно, в модуле gazebo.

Дефолт ``localization`` читается из rtabmap_lidar_landmarks_params.yaml (launch_config).

  Картирование (новая карта): ros2 launch rtabmap_landmarks complete_system.launch.py localization:=false delete_db_on_start:=true
  Локализация:                ros2 launch rtabmap_landmarks complete_system.launch.py localization:=true

После картирования позы больших маркеров выгружаются из карты в landmarks.yaml:
  ros2 run rtabmap_landmarks export_landmarks_from_db
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(share_dir: str, launch_file: str, **launch_args) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_dir, 'launch', launch_file)),
        **({'launch_arguments': list(launch_args.items())} if launch_args else {}),
    )


def _localization_default_from_yaml(rtabmap_share: str) -> str:
    """Read localization default from rtabmap_lidar_landmarks_params.yaml at launch time."""
    cfg_path = os.path.join(rtabmap_share, 'config', 'rtabmap_lidar_landmarks_params.yaml')
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    return 'true' if cfg['launch_config']['ros__parameters']['localization'] else 'false'


def generate_launch_description():
    stag_share = get_package_share_directory('stag_ros2_enhanced')
    rtabmap_share = get_package_share_directory('rtabmap_landmarks')
    tf_to_fiducial_share = get_package_share_directory('tf_to_fiducial')

    # Детекторы HD15 (малые маркеры) и HD17 (большие маркеры, ландмарки RTAB-Map).
    # GroupAction scopes their launch configurations: otherwise the detectors'
    # `config_file`/`namespace` leak into the includes below (rtabmap would
    # load the detector YAML as its config_file).
    stag_detector = GroupAction([_include(stag_share, 'stag_two_detectors.launch.py')])

    # use_sim_time is NOT threaded into the sub-launches: each reads it from
    # its own YAML (single source of truth).
    tf_to_fiducial = _include(
        tf_to_fiducial_share, 'tf_to_fiducial.launch.py',
        localization=LaunchConfiguration('localization'),
    )
    rtabmap_lidar_landmarks = _include(
        rtabmap_share, 'rtabmap_lidar_landmarks.launch.py',
        localization=LaunchConfiguration('localization'),
        delete_db_on_start=LaunchConfiguration('delete_db_on_start'),
        database_path=LaunchConfiguration('database_path'),
    )

    landmarks_viz = Node(
        package='rtabmap_landmarks',
        executable='landmarks_yaml_markers_node',
        name='landmarks_yaml_markers',
        output='screen',
        parameters=[os.path.join(rtabmap_share, 'config', 'landmarks_yaml_markers_params.yaml')],
        condition=IfCondition(LaunchConfiguration('localization')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'localization', default_value=_localization_default_from_yaml(rtabmap_share),
            description=(
                'false: режим картографирования (SLAM); true: режим локализации по готовой карте. '
                'Дефолт читается из rtabmap_lidar_landmarks_params.yaml → launch_config.localization.'
            ),
        ),
        DeclareLaunchArgument(
            'delete_db_on_start', default_value='',
            description='true: удалить карту перед картированием. Пусто = из YAML.',
        ),
        DeclareLaunchArgument(
            'database_path', default_value='',
            description='Путь к .db карты. Пусто = из YAML.',
        ),
        stag_detector,
        tf_to_fiducial,
        rtabmap_lidar_landmarks,
        landmarks_viz,
    ])
