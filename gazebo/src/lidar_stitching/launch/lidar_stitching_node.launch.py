import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lidar_stitching'),
        'config',
        'lidar_stitching_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='lidar_stitching',
            executable='lidar_stitching_node',
            name='lidar_stitching_node',
            output='screen',
            parameters=[config]
        )
    ])