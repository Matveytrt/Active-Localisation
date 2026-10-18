"""Launch traj_recorder with config/traj_recorder.yaml + CLI overrides.

    ros2 launch al_eval traj_recorder.launch.py run_name:=baseline_01 \
        output_dir:=~/al_runs gt_offset_x:=0.0 gt_offset_y:=0.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory('al_eval'),
                               'config', 'traj_recorder.yaml')

    args = [
        DeclareLaunchArgument('config', default_value=default_cfg),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('output_dir', default_value='~/al_runs'),
        DeclareLaunchArgument('run_name', default_value=''),
        DeclareLaunchArgument('rate_hz', default_value='10.0'),
        DeclareLaunchArgument('gt_topic', default_value='/odom_gt'),
        DeclareLaunchArgument('record_gt', default_value='true'),
        DeclareLaunchArgument('gt_offset_x', default_value='0.0'),
        DeclareLaunchArgument('gt_offset_y', default_value='0.0'),
        DeclareLaunchArgument('gt_offset_yaw', default_value='0.0'),
        DeclareLaunchArgument('overwrite', default_value='false'),
        DeclareLaunchArgument('record_landmarks', default_value='true'),
        DeclareLaunchArgument('landmarks_topic',
                              default_value='/landmark_detections'),
        DeclareLaunchArgument('record_map_odom', default_value='true'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
    ]

    def lc(name, typ):
        return ParameterValue(LaunchConfiguration(name), value_type=typ)

    node = Node(
        package='al_eval',
        executable='traj_recorder',
        name='traj_recorder',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config'),
            {
                'use_sim_time': lc('use_sim_time', bool),
                'output_dir': lc('output_dir', str),
                'run_name': lc('run_name', str),
                'rate_hz': lc('rate_hz', float),
                'gt_topic': lc('gt_topic', str),
                'record_gt': lc('record_gt', bool),
                'gt_offset_x': lc('gt_offset_x', float),
                'gt_offset_y': lc('gt_offset_y', float),
                'gt_offset_yaw': lc('gt_offset_yaw', float),
                'overwrite': lc('overwrite', bool),
                'record_landmarks': lc('record_landmarks', bool),
                'landmarks_topic': lc('landmarks_topic', str),
                'record_map_odom': lc('record_map_odom', bool),
                'odom_frame': lc('odom_frame', str),
            },
        ],
    )
    return LaunchDescription(args + [node])
