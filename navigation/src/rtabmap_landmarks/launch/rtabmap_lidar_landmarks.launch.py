"""
RTAB-Map launch: lidar + landmarks (fiducial TF).

All algorithm parameters are loaded from the YAML specified by the
'config_file' launch argument. If not specified, the default
rtabmap_lidar_landmarks_params.yaml is used.

This launch file is responsible only for:
  - topic remappings (topics from launch_config)
  - database_path / localization / delete_db_on_start
  - mode-dependent parameters (launch_config.mapping_params / localization_params)

use_sim_time is read from the YAML '/**' section (single source of truth),
not injected here.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(value: str, default: bool) -> bool:
    if not value or value.strip() == "":
        return default
    v = value.strip().lower()
    if v in ("true", "1", "yes", "on"):
        return True
    if v in ("false", "0", "no", "off"):
        return False
    return default


def launch_setup(context: LaunchContext, *args, **kwargs):
    pkg = get_package_share_directory("rtabmap_landmarks")

    default_config = os.path.join(
        pkg,
        "config",
        "rtabmap_lidar_landmarks_params.yaml",
    )

    config_file = LaunchConfiguration("config_file").perform(context).strip() or default_config
    print(f"[rtabmap] Using parameter file: {config_file}")

    with open(config_file) as f:
        cfg = yaml.safe_load(f)
    lc = cfg.get("launch_config", {}).get("ros__parameters", {})

    def arg_or(name: str, default) -> str:
        v = LaunchConfiguration(name).perform(context).strip()
        return v if v else str(default)

    odom_topic = lc.get("odom_topic", "/odom")
    lidar_topic = lc.get("lidar_topic", "/lidar/points_joined")
    landmark_topic = lc.get("landmark_detections_topic", "/landmark_detections")
    map_topic = lc.get("rtabmap_map_topic", "/map")
    db_path = arg_or("database_path", lc.get("database_path", "~/.ros/rtabmap.db"))

    localization = _parse_bool(
        LaunchConfiguration("localization").perform(context),
        bool(lc.get("localization", False)),
    )
    delete_db = _parse_bool(
        LaunchConfiguration("delete_db_on_start").perform(context),
        bool(lc.get("delete_db_on_start", False)),
    )

    # Memory / occupancy grid settings differ between mapping and localization
    # runs of the same node, so the YAML keeps them in two sections and the
    # active one is chosen here from the resolved mode.
    mode_params = lc.get("localization_params" if localization else "mapping_params", {})
    runtime_params = {"database_path": db_path}
    runtime_params.update({k: str(v) for k, v in mode_params.items()})

    arguments = ["-d"] if (not localization and delete_db) else []
    print(
        f"[rtabmap] mode={'localization' if localization else 'mapping'} "
        f"database={db_path} delete_db={'yes' if arguments else 'no'}"
    )

    remappings = [
        ("map", map_topic),
        ("initialpose", "/initialpose"),
        ("odom", odom_topic),
        ("imu", "imu_not_used"),
        ("scan_cloud", lidar_topic),
        ("landmark_detections", landmark_topic),
    ]

    return [
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[config_file, runtime_params],
            remappings=remappings,
            arguments=arguments,
        )
    ]


def generate_launch_description():
    pkg = get_package_share_directory("rtabmap_landmarks")
    default_config = os.path.join(pkg, "config", "rtabmap_lidar_landmarks_params.yaml")

    args = [
        ("config_file", default_config, "RTAB-Map parameter YAML file."),
        ("localization", "", "Localization mode. Empty = use YAML."),
        ("delete_db_on_start", "", "Delete database on startup (mapping only). Empty = use YAML."),
        ("database_path", "", "Override database path."),
    ]

    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, desc in args]
        + [OpaqueFunction(function=launch_setup)]
    )
