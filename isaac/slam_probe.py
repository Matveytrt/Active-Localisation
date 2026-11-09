"""Shared external ROS observations; GT is recorded/evaluated, never drives SLAM."""
import csv
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rtabmap_msgs.msg import LandmarkDetections, Info
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException


def planar(message):
    p, q = message.position, message.orientation
    return np.array([p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def stamp(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


def matrix(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0., 0., 1.]])


class Probe(Node):
    def __init__(self, name, output):
        super().__init__(name)
        self.gt = self.odom = self.grid = self.info = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.edges, self.raw_ids, self.landmark_ids = set(), set(), set()
        self.landmarks, self.landmark_messages = {}, []
        self.marker_tf = {}
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom_gt', self.on_gt, 50)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 50)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'grid', m), latched)
        self.create_subscription(Info, '/info', lambda m: setattr(self, 'info', m), 10)
        self.create_subscription(LandmarkDetections, '/landmark_detections', self.on_landmarks, 50)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)
        self.file = (output/'trajectory.csv').open('w')
        self.csv = csv.writer(self.file)
        self.csv.writerow(['sim_s', 'gt_x', 'gt_y', 'gt_yaw', 'odom_x', 'odom_y', 'odom_yaw', 'map_x', 'map_y', 'map_yaw'])
        self.last_recorded = -math.inf

    def on_tf(self, msg):
        for tf in msg.transforms:
            self.edges.add((tf.header.frame_id, tf.child_frame_id))
            if tf.child_frame_id.startswith('stag_hd17_'):
                self.raw_ids.add(int(tf.child_frame_id.split('_')[-1]))
                self.marker_tf[tf.child_frame_id] = tf

    def on_landmarks(self, msg):
        for landmark in msg.landmarks:
            self.landmark_ids.add(landmark.id)
            self.landmarks[landmark.id] = landmark
        self.landmark_messages.append((stamp(self.gt) if self.gt is not None else 0., [m.id for m in msg.landmarks]))

    def on_gt(self, msg):
        self.gt = msg
        if self.odom is not None and stamp(msg)-self.last_recorded >= .2:
            estimate = self.estimate()
            self.csv.writerow([stamp(msg), *planar(msg.pose.pose), *planar(self.odom.pose.pose),
                               *(estimate if estimate is not None else [float('nan')]*3)])
            self.last_recorded = stamp(msg)

    def estimate(self, allow_odom=False):
        try:
            tf = self.buffer.lookup_transform('map', 'base_link', Time())
            p, q = tf.transform.translation, tf.transform.rotation
            return np.array([p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])
        except TransformException:
            return planar(self.odom.pose.pose) if allow_odom and self.odom is not None else None

    def send(self, velocity):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = map(float, velocity)
        self.publisher.publish(msg)

    def wait(self, predicate, timeout=30):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('SLAM probe condition')
            rclpy.spin_once(self, timeout_sec=.02)

    def hold(self, duration):
        start, next_send = stamp(self.gt), 0.
        deadline = time.monotonic()+max(30., duration*5)
        while stamp(self.gt)-start < duration:
            if time.monotonic() > deadline:
                raise TimeoutError('simulation clock stopped')
            if time.monotonic() >= next_send:
                self.send([0, 0, 0])
                next_send = time.monotonic()+.05
            rclpy.spin_once(self, timeout_sec=.01)

    def move(self, goal):
        # Only noisy odometry / estimated map TF enters the driver. GT is never
        # read here; it remains an independent evaluation channel.
        goal = np.array(goal)
        deadline, next_send = time.monotonic()+120, 0.
        while True:
            if time.monotonic() > deadline:
                raise TimeoutError('mapping waypoint '+str(goal))
            current = self.estimate(allow_odom=True)
            delta = goal-current
            delta[2] = angle(delta[2])
            if np.linalg.norm(delta[:2]) < .04 and abs(delta[2]) < .025:
                break
            if time.monotonic() >= next_send:
                c, s = math.cos(current[2]), math.sin(current[2])
                body = np.array([[c, s], [-s, c]]) @ delta[:2]
                self.send([*np.clip(body*.9, -.3, .3), float(np.clip(delta[2], -.4, .4))])
                next_send = time.monotonic()+.04
            rclpy.spin_once(self, timeout_sec=.01)
        self.hold(1.)
        return self.estimate().tolist()

    def close(self):
        self.file.close()
        self.destroy_node()
