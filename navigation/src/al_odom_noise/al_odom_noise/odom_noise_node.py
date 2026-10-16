"""ROS 2 node: /odom_gt (ideal planar_move odometry) -> noisy /odom + TF.

Thin rclpy wrapper around :class:`al_odom_noise.noise_model.OdomNoiseModel`.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

from al_odom_noise.noise_model import OdomNoiseModel, OdomNoiseParams

SMALL_VAR = 1e-9  # variance for z / roll / pitch (planar robot)
BACKWARD_JUMP_S = 1.0  # time going back by more than this -> model reset


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


IDX_XYYAW = (0, 1, 5)  # x, y, yaw rows/cols in the 6x6 (x y z roll pitch yaw)


def cov6(cov3):
    """6x6 row-major covariance from a 3x3 (x, y, yaw) block.

    The (x, y, yaw) block, including cross terms, goes to indices
    0,1,5 / 6,7,11 / 30,31,35; z, roll, pitch get SMALL_VAR.
    """
    c = [0.0] * 36
    c[14] = SMALL_VAR
    c[21] = SMALL_VAR
    c[28] = SMALL_VAR
    for i, r in enumerate(IDX_XYYAW):
        for j, k in enumerate(IDX_XYYAW):
            c[6 * r + k] = float(cov3[i][j])
    return c


def diag3(v):
    return [[v[0], 0.0, 0.0], [0.0, v[1], 0.0], [0.0, 0.0, v[2]]]


class OdomNoiseNode(Node):

    def __init__(self):
        super().__init__('odom_noise_node')
        self.input_topic = self.declare_parameter('input_topic', '/odom_gt').value
        self.output_topic = self.declare_parameter('output_topic', '/odom').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)
        seed = int(self.declare_parameter('seed', 42).value)

        # dynamic_typing: accept e.g. "a1: 0" (int) from YAML/CLI for a double
        # parameter; the value is cast to the dataclass field type below.
        defaults = OdomNoiseParams()
        dyn = ParameterDescriptor(dynamic_typing=True)
        values = {}
        for name in OdomNoiseParams.names():
            default = getattr(defaults, name)
            raw = self.declare_parameter(name, default, dyn).value
            if isinstance(default, bool) and isinstance(raw, str):
                values[name] = raw.strip().lower() in ('true', '1', 'yes')
            else:
                values[name] = type(default)(raw)
        params = OdomNoiseParams.from_dict(values)
        self.model = OdomNoiseModel(params, seed=seed)
        self._last_t = None

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Odometry, self.output_topic, qos)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.sub = self.create_subscription(Odometry, self.input_topic, self._on_odom, qos)

        self.get_logger().info(
            '%s -> %s (%s->%s, tf=%s), seed=%d, params=%s'
            % (self.input_topic, self.output_topic, self.odom_frame, self.base_frame,
               self.publish_tf, seed, params))

    def _on_odom(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self._last_t is not None and t < self._last_t - BACKWARD_JUMP_S:
            self.get_logger().warn('time jumped back (%.3f -> %.3f s), resetting model'
                                   % (self._last_t, t))
            self.model.reset()
        self._last_t = t

        p = msg.pose.pose
        s = self.model.update(p.position.x, p.position.y, yaw_from_quat(p.orientation), t)
        qz = math.sin(0.5 * s.yaw)
        qw = math.cos(0.5 * s.yaw)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = s.x
        out.pose.pose.position.y = s.y
        out.pose.pose.position.z = p.position.z
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.pose.covariance = cov6(s.pose_cov)
        out.twist.twist.linear.x = s.vx
        out.twist.twist.linear.y = s.vy
        out.twist.twist.angular.z = s.wz
        out.twist.covariance = cov6(diag3(s.twist_var))
        self.pub.publish(out)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = s.x
            tf.transform.translation.y = s.y
            tf.transform.translation.z = p.position.z
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
