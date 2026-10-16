import math
import os
import subprocess
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def _render_urdf_or_xacro(file_path: str) -> str:
    """Render .xacro (via xacro CLI) or read plain URDF text."""
    if file_path.endswith(".xacro"):
        proc = subprocess.run(
            ["xacro", file_path],
            check=True,
            capture_output=True,
            text=True,
        )
        return proc.stdout
    return open(file_path, encoding="utf-8").read()


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matrix_to_rpy(m):
    pitch = math.atan2(-m[2][0], math.hypot(m[0][0], m[1][0]))
    if abs(math.cos(pitch)) < 1e-9:
        return math.atan2(-m[1][2], m[1][1]), pitch, 0.0
    return math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0])


def _axis_angle_to_matrix(axis, angle):
    norm = math.sqrt(sum(v * v for v in axis))
    x, y, z = (v / norm for v in axis)
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def _bake_joint_position(joint, value):
    """Rotate the joint origin by its position so the pose survives conversion to fixed."""
    if isinstance(value, dict):
        axis, angle = value["axis"], float(value["angle"])
    else:
        axis_el = joint.find("axis")
        axis = [float(v) for v in (axis_el.get("xyz") if axis_el is not None else "1 0 0").split()]
        angle = float(value)
    origin = joint.find("origin")
    if origin is None:
        origin = ET.SubElement(joint, "origin")
    rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    rotated = _matmul(_rpy_to_matrix(*rpy), _axis_angle_to_matrix(axis, angle))
    origin.set("rpy", " ".join(f"{v:.6f}" for v in _matrix_to_rpy(rotated)))


def _load_robot_description_with_all_joints_fixed(file_path: str, pose_file: str) -> str:
    """Render robot description and convert all non-fixed joints to fixed for stable simulation.

    Joint positions from pose_file are baked into the joint origins before the conversion.
    """
    xml_text = _render_urdf_or_xacro(file_path)
    root = ET.fromstring(xml_text)
    with open(pose_file, encoding="utf-8") as f:
        joint_positions = (yaml.safe_load(f) or {}).get("joint_positions") or {}
    removable_tags = (
        "axis",
        "limit",
        "dynamics",
        "safety_controller",
        "calibration",
        "mimic",
    )
    for joint in root.findall("joint"):
        joint_type = (joint.get("type") or "").strip().lower()
        if joint_type != "fixed":
            if joint.get("name") in joint_positions:
                _bake_joint_position(joint, joint_positions[joint.get("name")])
            joint.set("type", "fixed")
            for tag in removable_tags:
                for child in joint.findall(tag):
                    joint.remove(child)
    return ET.tostring(root, encoding="unicode")


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    # Пути к файлам
    pkg_path = os.path.join(get_package_share_directory("world_and_robot"))
    urdf_file = os.path.join(
        pkg_path, "urdf_astribot", "astribot_full.xacro"
    )
    default_rviz_config = os.path.join(pkg_path, "rviz", "model_rviz.rviz")

    # Описание робота: все подвижные joints делаем fixed для стабильной статичной модели.
    pose_file = os.path.join(pkg_path, "config", "astribot_pose.yaml")
    robot_description = _load_robot_description_with_all_joints_fixed(urdf_file, pose_file)
    robot_description_param = {
        "robot_description": robot_description,
        "use_sim_time": use_sim_time,
        "ignore_timestamp": False,
    }

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    # В robot_description все суставы уже fixed (поза из config/astribot_pose.yaml),
    # поэтому TF робота от /joint_states не зависят. Узел лишь держит топик /joint_states
    # (нулевые позиции подвижных суставов исходного URDF) для его подписчиков.
    joint_state_keeper_sim_node = Node(
        package="world_and_robot",
        executable="astribot_joint_state_keeper.py",
        name="astribot_joint_state_keeper",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {
                "robot_urdf_path": os.path.join(
                    pkg_path,
                    "urdf_astribot",
                    "astribot_whole_body_with_head.urdf",
                )
            },
            {"publish_hz": 30.0},
            {"joint_states_topic": "/joint_states"},
        ],
        condition=IfCondition(use_sim_time),
    )

    # Без симуляции /joint_states публикует joint_state_publisher_gui (слайдеры суставов)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=UnlessCondition(use_sim_time),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Path to RViz config file",
            ),
            node_robot_state_publisher,
            joint_state_keeper_sim_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
