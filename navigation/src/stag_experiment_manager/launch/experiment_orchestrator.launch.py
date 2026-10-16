"""
Launch file for STag Experiment Orchestrator

Starts the Orchestrator Node for main experiment management.

Note: You need to manually start the keyboard control node in a separate terminal:
    ros2 run stag_experiment_manager keyboard_control

Usage:
    ros2 launch stag_experiment_manager experiment_orchestrator.launch.py

Optional arguments:
    config_file:=<path>  - Path to experiment configuration YAML
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for experiment orchestrator."""
    
    # Get package share directory
    pkg_share = FindPackageShare('stag_experiment_manager')
    
    # ===== Launch Arguments =====
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'experiment_config.yaml'
        ]),
        description='Path to experiment configuration YAML file'
    )
    
    # ===== Nodes =====
    
    # Orchestrator Node (main management)
    orchestrator_node = Node(
        package='stag_experiment_manager',
        executable='orchestrator',
        name='experiment_orchestrator',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # Arguments
        config_file_arg,
        
        # Nodes
        orchestrator_node,
    ])
