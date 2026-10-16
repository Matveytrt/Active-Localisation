import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('al_odom_noise'), 'config', 'odom_noise.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML with odom_noise_node parameters'),
        DeclareLaunchArgument('seed', default_value='42',
                              description='RNG seed (overrides the YAML value)'),
        Node(
            package='al_odom_noise',
            executable='odom_noise_node',
            name='odom_noise_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int)},
            ],
        ),
    ])
