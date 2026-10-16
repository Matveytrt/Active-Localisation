# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def _bt_xml_rewrite(context, bringup_dir: str) -> str:
    """Absolute path of the BT XML named in params_file (bt_navigator.default_nav_to_pose_bt_xml).

    BT-navigator's default_nav_to_pose_bt_xml MUST be an absolute path and Nav2
    YAML has no substitution mechanism, so only the file name is taken from the
    YAML and resolved against nav2_planner_custom's installed behavior_trees/.
    """
    params_path = LaunchConfiguration('params_file').perform(context)
    with open(params_path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh)
    bt_name = os.path.basename(
        data['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']
    )
    return os.path.join(bringup_dir, 'behavior_trees', bt_name)


def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('nav2_planner_custom')

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    bt_xml = _bt_xml_rewrite(context, bringup_dir)
    print(f'[nav2_planner_custom] default_nav_to_pose_bt_xml: {bt_xml}')
    params_file_rewritten = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'default_nav_to_pose_bt_xml': bt_xml},
        convert_types=True,
    )

    # Remap /tf and /tf_static into the launch namespace; there is no
    # PushNodeRemapping in ros2 launch yet (ros2/launch_ros#56).
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    def _nav2_node(package, executable, name=None, extra_remappings=()):
        return Node(
            package=package,
            executable=executable,
            **({'name': name} if name else {}),
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[params_file_rewritten],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + list(extra_remappings),
        )

    # (package, executable, name-or-None, extra_remappings). `None` means the
    # node name is left to the executable's default (matches the historical
    # behaviour where `controller_server` was launched without an explicit name).
    _NAV2_NODES = [
        ('nav2_controller', 'controller_server', None,
         [('cmd_vel', 'cmd_vel_raw')]),
        ('nav2_smoother', 'smoother_server', 'smoother_server', []),
        ('nav2_planner', 'planner_server', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server', 'behavior_server', []),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', []),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
         [('cmd_vel', 'cmd_vel_raw'), ('cmd_vel_smoothed', 'cmd_vel')]),
    ]

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[_nav2_node(pkg, exe, name, extra) for pkg, exe, name, extra in _NAV2_NODES],
    )

    # lifecycle_manager: all params (use_sim_time / autostart / bond_timeout /
    # node_names) are in the params file under 'lifecycle_manager_navigation'.
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file_rewritten],
        condition=IfCondition(PythonExpression(['not ', use_composition])),
    )

    def _composable(package, plugin, name, extra_remappings=()):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            parameters=[params_file_rewritten],
            remappings=remappings + list(extra_remappings),
        )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            _composable('nav2_controller', 'nav2_controller::ControllerServer',
                        'controller_server', [('cmd_vel', 'cmd_vel_raw')]),
            _composable('nav2_smoother', 'nav2_smoother::SmootherServer',
                        'smoother_server'),
            _composable('nav2_planner', 'nav2_planner::PlannerServer',
                        'planner_server'),
            _composable('nav2_behaviors', 'behavior_server::BehaviorServer',
                        'behavior_server'),
            _composable('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator',
                        'bt_navigator'),
            _composable('nav2_waypoint_follower',
                        'nav2_waypoint_follower::WaypointFollower',
                        'waypoint_follower'),
            _composable('nav2_velocity_smoother',
                        'nav2_velocity_smoother::VelocitySmoother',
                        'velocity_smoother',
                        [('cmd_vel', 'cmd_vel_raw'), ('cmd_vel_smoothed', 'cmd_vel')]),
            _composable('nav2_lifecycle_manager',
                        'nav2_lifecycle_manager::LifecycleManager',
                        'lifecycle_manager_navigation'),
        ],
    )

    return [load_nodes, lifecycle_manager_nav, load_composable_nodes]


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_planner_custom')

    # NOTE on use_sim_time: intentionally NOT declared as a top-level launch
    # arg. Every node reads `use_sim_time` from its OWN YAML config (true in
    # nav2_params_v2.yaml for the simulation). This keeps the "YAML is the
    # single source of truth" invariant — no hidden top-level knob silently
    # rewrites per-node parameters.
    launch_args = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in (
            ('namespace', '', 'Top-level namespace'),
            ('params_file',
             os.path.join(bringup_dir, 'config', 'nav2_params_v2.yaml'),
             'Full path to the ROS2 parameters file to use for all launched nodes'),
            ('use_composition', 'False', 'Use composed bringup if True'),
            ('container_name', 'nav2_container',
             'the name of container that nodes will load in if use composition'),
            ('use_respawn', 'False',
             'Whether to respawn if a node crashes. Applied when composition is disabled.'),
            ('log_level', 'info', 'log level'),
        )
    ]

    return LaunchDescription(
        [SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')]
        + launch_args
        + [OpaqueFunction(function=launch_setup)]
    )
