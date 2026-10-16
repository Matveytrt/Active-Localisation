#!/usr/bin/env python3
"""Publish Astribot joint states at a fixed rate to keep TF tree populated."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class AstribotJointStateKeeper(Node):
    def __init__(self) -> None:
        super().__init__("astribot_joint_state_keeper")

        self.declare_parameter(
            "robot_urdf_path",
            "/home/app/ros2_ws/install/world_and_robot/share/world_and_robot/urdf_astribot/astribot_whole_body_with_head.urdf",
        )
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("joint_states_topic", "/joint_states")

        urdf_path = Path(str(self.get_parameter("robot_urdf_path").value)).expanduser()
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        topic_name = str(self.get_parameter("joint_states_topic").value)

        self._joint_names = self._load_non_fixed_joints(urdf_path)
        if not self._joint_names:
            self.get_logger().warn(
                f"No non-fixed joints found in {urdf_path}; publishing empty JointState."
            )
        else:
            self.get_logger().info(
                f"Loaded {len(self._joint_names)} joints from {urdf_path}"
            )

        self._positions = [0.0] * len(self._joint_names)
        self._publisher = self.create_publisher(JointState, topic_name, 10)
        self._timer = self.create_timer(1.0 / publish_hz, self._on_timer)

    def _load_non_fixed_joints(self, urdf_path: Path) -> list[str]:
        if not urdf_path.is_file():
            self.get_logger().error(f"URDF file not found: {urdf_path}")
            return []
        try:
            root = ET.parse(urdf_path).getroot()
        except ET.ParseError as exc:
            self.get_logger().error(f"Failed to parse URDF {urdf_path}: {exc}")
            return []

        joint_names: list[str] = []
        for joint in root.findall("joint"):
            joint_type = joint.get("type", "fixed").strip().lower()
            if joint_type == "fixed":
                continue
            name = joint.get("name")
            if name:
                joint_names.append(name)
        return joint_names

    def _on_timer(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = self._positions
        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AstribotJointStateKeeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
