#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("rtabmap_landmarks")
    params_file = os.path.join(
        package_share, "config", "landmarks_yaml_markers_params.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="rtabmap_landmarks",
                executable="landmarks_yaml_markers_node",
                name="landmarks_yaml_markers",
                output="screen",
                parameters=[params_file],
            )
        ]
    )
