#!/usr/bin/env python3

from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


_PARAM_TYPES = {
    "landmarks_yaml_path": Parameter.Type.STRING,
    "marker_topic": Parameter.Type.STRING,
    "publish_rate_hz": Parameter.Type.DOUBLE,
    "arrow_length": Parameter.Type.DOUBLE,
    "arrow_shaft_diameter": Parameter.Type.DOUBLE,
    "arrow_head_diameter": Parameter.Type.DOUBLE,
    "sphere_diameter": Parameter.Type.DOUBLE,
    "namespace": Parameter.Type.STRING,
    "alpha": Parameter.Type.DOUBLE,
}


class LandmarksYamlMarkersNode(Node):
    """Publishes RViz markers for the big markers from landmarks.yaml.

    All params are declared type-only — YAML is the single source of truth
    (see `config/landmarks_yaml_markers_params.yaml`).
    """

    def __init__(self):
        super().__init__("landmarks_yaml_markers")

        for name, ptype in _PARAM_TYPES.items():
            self.declare_parameter(name, ptype)
        p = lambda name: self.get_parameter(name).value  # noqa: E731

        self._yaml_path = Path(str(p("landmarks_yaml_path"))).expanduser()
        self._marker_topic = str(p("marker_topic"))
        publish_rate_hz = float(p("publish_rate_hz"))
        self._arrow_len = float(p("arrow_length"))
        self._arrow_diam = float(p("arrow_shaft_diameter"))
        self._arrow_head_diam = float(p("arrow_head_diameter"))
        self._sphere_diam = float(p("sphere_diameter"))
        self._ns = str(p("namespace"))
        self._alpha = float(p("alpha"))
        self._yaml_cache: dict = {}

        # RViz needs TRANSIENT_LOCAL so late-joining clients get the last
        # published marker snapshot.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._pub = self.create_publisher(MarkerArray, self._marker_topic, qos)

        self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish_markers)

        self.get_logger().info(
            f"Publishing landmarks markers from {self._yaml_path} to {self._marker_topic}"
        )

    @staticmethod
    def _big_markers_to_entries(big_markers: list) -> list:
        """Normalize unified-file ``big_markers`` to flat drawable entries."""
        return [
            {
                "id": int(bm.get("id", 0)),
                "pose": {
                    "position": bm.get("position", {}),
                    "orientation": bm.get("orientation", {}),
                },
            }
            for bm in big_markers
        ]

    def _append_dataset_markers(
        self, msg: MarkerArray, frame_id: str, landmarks: list, namespace: str, color: tuple,
    ) -> None:
        stamp = self.get_clock().now().to_msg()

        def _make(ns_suffix: str, mtype: int, rviz_id: int) -> Marker:
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = stamp
            m.ns = f"{namespace}_{ns_suffix}"
            m.id = rviz_id
            m.type = mtype
            m.action = Marker.ADD
            m.color.r, m.color.g, m.color.b = float(color[0]), float(color[1]), float(color[2])
            m.color.a = self._alpha
            return m

        for slot, entry in enumerate(landmarks):
            marker_id = int(entry.get("id", 0))
            pose = entry.get("pose", {})
            pos = pose.get("position", {})
            ori = pose.get("orientation", {})
            px = float(pos.get("x", 0.0))
            py = float(pos.get("y", 0.0))
            pz = float(pos.get("z", 0.0))

            arrow = _make("arrow", Marker.ARROW, slot)
            arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = px, py, pz
            arrow.pose.orientation.x = float(ori.get("x", 0.0))
            arrow.pose.orientation.y = float(ori.get("y", 0.0))
            arrow.pose.orientation.z = float(ori.get("z", 0.0))
            arrow.pose.orientation.w = float(ori.get("w", 1.0))
            arrow.scale.x = self._arrow_len
            arrow.scale.y = self._arrow_diam
            arrow.scale.z = self._arrow_head_diam
            msg.markers.append(arrow)

            sphere = _make("pos", Marker.SPHERE, slot)
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = px, py, pz
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = self._sphere_diam
            msg.markers.append(sphere)

            label = _make("label", Marker.TEXT_VIEW_FACING, slot)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = px, py, pz + 0.18
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12
            label.color.r = label.color.g = label.color.b = 1.0
            label.text = str(marker_id)
            msg.markers.append(label)

    def _load_yaml(self, path: Path) -> dict:
        """Parsed YAML, re-read only when the file's mtime/size changes."""
        st = path.stat()
        key = (st.st_mtime_ns, st.st_size)
        cached = self._yaml_cache.get(path)
        if cached is not None and cached[0] == key:
            return cached[1]
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        self._yaml_cache[path] = (key, data)
        return data

    def _publish_markers(self):
        if not self._yaml_path.is_file():
            self.get_logger().warn(
                f"Landmarks YAML not found: {self._yaml_path}", throttle_duration_sec=30.0,
            )
            return
        data = self._load_yaml(self._yaml_path)
        frame_id = str(data.get("frame_id", "map"))

        msg = MarkerArray()

        clear = Marker()
        clear.header.frame_id = frame_id
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        msg.markers.append(clear)

        # Big markers live under 'big_markers' in the unified file; fall back to
        # the legacy flat 'landmarks_sorted_by_id'.
        if isinstance(data.get("big_markers"), list):
            big_entries = self._big_markers_to_entries(data["big_markers"])
        else:
            big_entries = data.get("landmarks_sorted_by_id") or []
        self._append_dataset_markers(
            msg=msg,
            frame_id=frame_id,
            landmarks=big_entries,
            namespace=self._ns,
            color=(0.95, 0.45, 0.10),
        )

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LandmarksYamlMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
