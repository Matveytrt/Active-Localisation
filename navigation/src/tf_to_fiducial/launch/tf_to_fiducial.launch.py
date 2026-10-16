"""Launch file for the tf_to_fiducial converter node (C++).

Converts marker TF frames into rtabmap_msgs/LandmarkDetections for RTAB-Map.
Operation mode (mapping | localization) is chosen from the `localization` argument,
so the node no longer needs to read RTAB-Map's config file.

use_sim_time is read from the YAML (single source of truth), not injected here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('tf_to_fiducial')
    config_file = os.path.join(package_share_dir, 'config', 'tf_to_fiducial_params.yaml')

    operation_mode = PythonExpression(
        ["'localization' if '", LaunchConfiguration('localization'), "' == 'true' else 'mapping'"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='true: localization mode; false: mapping mode',
        ),
        Node(
            package='tf_to_fiducial',
            executable='tf_to_fiducial_node',
            name='tf_to_fiducial_node',
            output='screen',
            parameters=[
                config_file,
                # use_sim_time is read from the YAML (single source of truth),
                # not injected here. operation_mode is the system-wide
                # mapping/localization switch, derived from the `localization`
                # argument (its default is sourced from RTAB-Map's YAML), so it
                # stays in sync with the rtabmap node.
                {'operation_mode': operation_mode},
            ],
        ),
    ])
