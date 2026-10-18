#!/usr/bin/env python3
"""Start-up nudge for scripts/run_experiment.sh (T-002 finding, D-2: identical in both modes).

RTAB-Map in localization mode localizes at once but publishes no map->odom until a
second localization, i.e. until the robot moves (FACTS 17.10, "waiting for another one
... RGBD/MaxOdomCacheSize>0"); without map->odom Nav2 never activates.  This node
drives the robot `--dist` metres along its heading and back to the start pose using a
P-controller on ground-truth odometry (`--odom`, default /odom_gt, frame == Gazebo
world), then waits for TF map->odom.  Repeats up to `--tries` times.

Exit code 0: map->odom available; 1: not available; 2: no odometry.
Runs inside the navigation container:  python3 al_nudge.py [--odom /odom_gt] [--dist 0.6]
"""
import argparse, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Nudge(Node):
    def __init__(self, odom_topic):
        super().__init__('al_nudge', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pose = None
        self.create_subscription(Odometry, odom_topic, self._odom, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)

    def _odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def spin_for(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.04)

    def goto(self, gx, gy, gyaw, vmax=0.25, tmo=60.0):
        t0 = time.time()
        while time.time() - t0 < tmo:
            rclpy.spin_once(self, timeout_sec=0.03)
            x, y, th = self.pose
            dx, dy, dth = gx - x, gy - y, wrap(gyaw - th)
            d = math.hypot(dx, dy)
            if d < 0.03 and abs(dth) < 0.03:
                self.stop()
                return True
            v = min(vmax, max(0.05 if d > 0.03 else 0.0, d))
            vx, vy = (dx / d * v, dy / d * v) if d > 1e-6 else (0.0, 0.0)
            t = Twist()
            t.linear.x = math.cos(th) * vx + math.sin(th) * vy
            t.linear.y = -math.sin(th) * vx + math.cos(th) * vy
            t.angular.z = max(-0.4, min(0.4, 1.5 * dth))
            self.pub.publish(t)
        self.stop()
        return False

    def has_map(self):
        try:
            self.tfb.lookup_transform('map', 'odom', Time())
            return True
        except Exception:
            return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--odom', default='/odom_gt')
    ap.add_argument('--dist', type=float, default=0.6)
    ap.add_argument('--tries', type=int, default=3)
    ap.add_argument('--wait', type=float, default=15.0, help='wall s to wait for map->odom after each nudge')
    a = ap.parse_args()
    rclpy.init()
    n = Nudge(a.odom)
    t0 = time.time()
    while n.pose is None and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.pose is None:
        print('al_nudge: no odometry on %s' % a.odom, flush=True)
        return 2
    x0, y0, th0 = n.pose
    print('al_nudge: start (%.3f, %.3f, %.1f deg)' % (x0, y0, math.degrees(th0)), flush=True)
    ok = False
    for k in range(1, a.tries + 1):
        n.goto(x0 + a.dist * math.cos(th0), y0 + a.dist * math.sin(th0), th0)
        n.goto(x0, y0, th0)
        t1 = time.time()
        while time.time() - t1 < a.wait and not (ok := n.has_map()):
            n.spin_for(0.5)
        x, y, th = n.pose
        print('al_nudge: try %d -> map->odom %s, back at (%.3f, %.3f, %.1f deg)'
              % (k, 'OK' if ok else 'missing', x, y, math.degrees(th)), flush=True)
        if ok:
            break
    n.stop()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
