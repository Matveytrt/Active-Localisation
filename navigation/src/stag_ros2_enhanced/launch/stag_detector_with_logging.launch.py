"""
Launch file for STag marker detector with logging
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for STag detector with logger"""
    
    # Get package directory
    pkg_share = FindPackageShare('stag_ros2_enhanced')
    
    # Declare launch arguments
    detector_config_arg = DeclareLaunchArgument(
        'detector_config',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'detector_params_HD15.yaml',
        ]),
        description=(
            'Detector YAML: detector_params_HD15.yaml or detector_params_HD17.yaml '
            '(canonical); detector_params.yaml is legacy only'
        ),
    )
    
    logger_config_arg = DeclareLaunchArgument(
        'logger_config',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'logger_params.yaml'
        ]),
        description='Path to logger parameters YAML file'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='stag_detector',
        description='Namespace for the nodes'
    )
    
    log_file_arg = DeclareLaunchArgument(
        'log_file_path',
        default_value='/home/app/ros2_ws/src/logging_files/output/logs/detections.jsonl',
        description='Path to output JSONL log file'
    )
    
    marker_type_arg = DeclareLaunchArgument(
        'marker_type',
        default_value='',
        description='Marker type to detect (HD17, HC21, HG17, etc.). Overrides YAML config if set.'
    )
    
    # Rosbag recording arguments
    enable_rosbag_arg = DeclareLaunchArgument(
        'enable_rosbag',
        default_value='false',
        description='Enable rosbag recording'
    )
    
    rosbag_output_dir_arg = DeclareLaunchArgument(
        'rosbag_output_dir',
        default_value='/home/app/ros2_ws/src/logging_files/output/bags',
        description='Directory to save rosbag files'
    )
    
    rosbag_name_arg = DeclareLaunchArgument(
        'rosbag_name',
        default_value='stag_experiment',
        description='Base name for rosbag file (without .db3 extension)'
    )
    
    rosbag_topics_arg = DeclareLaunchArgument(
        'rosbag_topics',
        default_value='/camera/camera/color/image_raw',
        description='Space-separated list of topics to record'
    )
    
    # Detector node (C++ executable)
    detector_node = Node(
        package='stag_ros2_enhanced',
        executable='stag_detector_node',
        name='stag_detector',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('detector_config'),
            {'marker_type': LaunchConfiguration('marker_type')}
        ],
        output='screen',
        emulate_tty=True,
    )

    # Logger node (C++ executable)
    logger_node = Node(
        package='stag_ros2_enhanced',
        executable='stag_detection_logger_node',
        name='detection_logger',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('logger_config'),
            {'log_file_path': LaunchConfiguration('log_file_path')}
        ],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('markers', 'stag_detector/markers')
        ]
    )
    
    # Rosbag recording process (conditional)
    rosbag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('enable_rosbag')),
        cmd=[
            'ros2', 'bag', 'record',
            '-o', [LaunchConfiguration('rosbag_output_dir'), '/', LaunchConfiguration('rosbag_name')],
            LaunchConfiguration('rosbag_topics')
        ],
        output='screen',
        shell=True
    )
    
    return LaunchDescription([
        detector_config_arg,
        logger_config_arg,
        namespace_arg,
        log_file_arg,
        marker_type_arg,
        enable_rosbag_arg,
        rosbag_output_dir_arg,
        rosbag_name_arg,
        rosbag_topics_arg,
        detector_node,
        logger_node,
        rosbag_record,
    ])
