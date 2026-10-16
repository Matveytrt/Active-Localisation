"""
Launch two stag_detector instances (HD15 + HD17 configs). Same as running
stag_detector.launch.py twice with different namespace and config_file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('stag_ros2_enhanced')
    launch_py = os.path.join(pkg, 'launch', 'stag_detector.launch.py')

    hd15 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_py),
        launch_arguments=[
            ('namespace', 'stag_hd15'),
            ('config_file', os.path.join(pkg, 'config', 'detector_params_HD15.yaml')),
        ],
    )
    hd17 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_py),
        launch_arguments=[
            ('namespace', 'stag_hd17'),
            ('config_file', os.path.join(pkg, 'config', 'detector_params_HD17.yaml')),
        ],
    )

    return LaunchDescription([hd15, hd17])
