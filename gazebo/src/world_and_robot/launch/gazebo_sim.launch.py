import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    package_name = "world_and_robot"

    pkg_share = get_package_share_directory(package_name)
    pkg_share_parent = os.path.dirname(pkg_share)
    world_path = os.path.join(
        pkg_share, "worlds", "astribot_hd15_hd17_walls.world"
    )

    # Add custom models (shelf_6) to Gazebo search path
    custom_model_path = os.path.join(pkg_share, "gazebo_objects")
    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    # Важно: SDFormat может преобразовывать package://world_and_robot/... в
    # model://world_and_robot/...; тогда Gazebo должен видеть parent для папки
    # world_and_robot в GAZEBO_MODEL_PATH.
    model_paths = [custom_model_path, pkg_share_parent]
    if existing_model_path:
        model_paths.append(existing_model_path)
    gazebo_model_path = os.pathsep.join(model_paths)

    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_path,
        description="Path to Gazebo world file (.world format)",
    )

    # RViz config for simulation (can be overridden)
    default_rviz_config = os.path.join(pkg_share, "rviz", "gazebo_sim.rviz")
    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Path to RViz config file",
    )
    declare_gazebo_gui_arg = DeclareLaunchArgument(
        "gazebo_gui",
        default_value="true",
        description="Start Gazebo client GUI (set false for headless test)",
    )
    declare_software_rendering_arg = DeclareLaunchArgument(
        "software_rendering",
        default_value="true",
        description=(
            "Force software OpenGL for Gazebo/RViz stability. "
            "Set false for faster hardware rendering."
        ),
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "rsp.launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": "true",
            "rviz_config": LaunchConfiguration("rviz_config"),
        }.items(),
    )

    joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "joystick.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "params_file": os.path.join(pkg_share, "config", "gazebo_ros_params.yaml"),
            "verbose": "true",
            "pause": "false",
            "gui": LaunchConfiguration("gazebo_gui"),
        }.items(),
    )

    lidar_stitching_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("lidar_stitching"),
                    "launch",
                    "lidar_stitching_node.launch.py",
                )
            ]
        )
    )
    delayed_lidar_stitching = TimerAction(
        period=8.0,
        actions=[lidar_stitching_launch],
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "astribot", "-x", "-2.0", "-y", "-0.5", "-z", "0.16"],
        output="screen",
    )
    delayed_spawn_entity = TimerAction(
        period=6.0,
        actions=[spawn_entity],
    )
    delayed_joy = TimerAction(
        period=2.0,
        actions=[joy],
    )

    return LaunchDescription(
        [
            declare_world_arg,
            declare_rviz_config_arg,
            declare_gazebo_gui_arg,
            declare_software_rendering_arg,
            # Optional software rendering profile. Useful for stability on
            # problematic GPU drivers, but it makes Gazebo GUI much slower.
            SetEnvironmentVariable(
                "LIBGL_ALWAYS_SOFTWARE", "1",
                condition=IfCondition(LaunchConfiguration("software_rendering")),
            ),
            SetEnvironmentVariable(
                "MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe",
                condition=IfCondition(LaunchConfiguration("software_rendering")),
            ),
            SetEnvironmentVariable(
                "LIBGL_DRI3_DISABLE", "1",
                condition=IfCondition(LaunchConfiguration("software_rendering")),
            ),
            SetEnvironmentVariable(
                "OGRE_RTT_MODE", "Copy",
                condition=IfCondition(LaunchConfiguration("software_rendering")),
            ),
            SetEnvironmentVariable(
                "QT_OPENGL", "software",
                condition=IfCondition(LaunchConfiguration("software_rendering")),
            ),
            SetEnvironmentVariable(
                "QT_QUICK_BACKEND", "software",
                condition=IfCondition(LaunchConfiguration("software_rendering")),
            ),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
            rsp,
            delayed_joy,
            gazebo,
            delayed_spawn_entity,
            delayed_lidar_stitching,
        ]
    )


