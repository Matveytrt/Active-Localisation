"""Launch the active-localization nodes.

    ros2 launch active_localization active_localization.launch.py \
        scorer:=visible_count cost_field:=false

`scorer` overrides scorer.name from the config file ('' = keep).
`cost_field:=true` also starts al_cost_field (route lever).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory('active_localization'),
                               'config', 'active_localization.yaml')
    common = {
        'config_file': LaunchConfiguration('config'),
        'scorer': ParameterValue(LaunchConfiguration('scorer'), value_type=str),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_cfg),
        DeclareLaunchArgument('scorer', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cost_field', default_value='false'),
        Node(package='active_localization', executable='al_navigator',
             name='al_navigator', output='screen', parameters=[common]),
        Node(package='active_localization', executable='al_cost_field',
             name='al_cost_field', output='screen', parameters=[common],
             condition=IfCondition(LaunchConfiguration('cost_field'))),
    ])
