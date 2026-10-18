"""scenario_runner: drive Nav2 through a scenario (ordered goal poses).

    ros2 run al_eval scenario_runner --scenario <yaml> --out <dir>/result.yaml \
        [--run-name baseline_01] [--mode baseline] [--seed 42] \
        --ros-args -p use_sim_time:=true

Sequence:

1. Readiness (with timeouts): the NavigateToPose action server
   (``--action``, default ``/navigate_to_pose``) is available, then TF
   ``<frame_id> -> <base_frame>`` (``map -> base_link``) can be looked up.
   Readiness is checked here, not in the shell orchestrator, so that the
   check uses the same DDS participant / TF buffer that will send goals.
2. Wait ``settle_s`` (wall clock) while spinning.
3. Send goals one by one. Each goal has a wall-clock timeout
   ``goal_timeout_s``; on timeout the goal is cancelled and recorded as
   ``timeout``. By default the runner continues with the next goal after a
   failure (``--stop-on-failure`` to stop; remaining goals become
   ``skipped``).
4. Write ``result.yaml`` (see :class:`al_eval.scenario.RunResult`) and exit:
   0 all goals succeeded, 3 any goal failed / rejected / timed out,
   4 not ready (action server or TF), 2 bad input, 130 interrupted.

Per goal it records: terminal status, wall and sim duration, sim-time start
and end stamps (``start_sim_s`` / ``end_sim_s``, the evaluation window of
``eval/crop_run.py``), GT path length
(integrated xy distance of ``--distance-topic``, default ``/odom_gt``), final
xy / yaw error of the *estimated* pose (TF) w.r.t. the goal and the Nav2
``number_of_recoveries`` from the last feedback.

[unverified] Everything ROS-side here (Humble rclpy/nav2_msgs API usage,
SIGINT behaviour, timing) is written from the API docs and not run: there is
no ROS in the environment where this file was written. ENG: check on the
testbed.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Callable, Optional, Sequence

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from al_eval import scenario as sc
from al_eval.tum import quat_to_yaw, yaw_to_quat

try:  # Humble has it; keep a fallback for older rclpy
    from rclpy.executors import ExternalShutdownException
except ImportError:  # pragma: no cover
    class ExternalShutdownException(Exception):  # type: ignore[no-redef]
        pass

EXIT_INTERRUPTED = 130

_STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'canceled',
}


class ScenarioRunner(Node):

    def __init__(self, action_name: str, frame_id: str, base_frame: str,
                 distance_topic: str) -> None:
        super().__init__('scenario_runner')
        self.frame_id = frame_id
        self.base_frame = base_frame
        self.client = ActionClient(self, NavigateToPose, action_name)
        self.action_name = action_name
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_length = 0.0
        self._last_xy = None
        self.distance_topic = distance_topic
        if distance_topic:
            # BEST_EFFORT matches both reliable and best-effort publishers.
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Odometry, distance_topic, self._on_odom,
                                     qos)
        self.last_feedback = None
        self.active_handle = None  # goal handle of the goal in progress

    # ------------------------------------------------------------ helpers
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if self._last_xy is not None:
            self.path_length += math.hypot(p.x - self._last_xy[0],
                                           p.y - self._last_xy[1])
        self._last_xy = (p.x, p.y)

    def _on_feedback(self, fb_msg) -> None:
        self.last_feedback = fb_msg.feedback

    def spin_until(self, done: Callable[[], bool], timeout_s: float) -> bool:
        """Spin until ``done()`` or ``timeout_s`` wall seconds; True if done."""
        deadline = time.monotonic() + timeout_s
        while not done():
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def spin_for(self, seconds: float) -> None:
        self.spin_until(lambda: False, seconds)

    def sim_now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def lookup_pose(self):
        """(x, y, yaw) of base_frame in frame_id, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(self.frame_id,
                                                 self.base_frame, Time())
        except TransformException:
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return t.x, t.y, quat_to_yaw(r.x, r.y, r.z, r.w)

    # ------------------------------------------------------------ readiness
    def wait_ready(self, server_timeout_s: float, tf_timeout_s: float) -> str:
        """'' if ready, otherwise an error message."""
        log = self.get_logger()
        log.info('waiting for action server %s (timeout %.0f s)'
                 % (self.action_name, server_timeout_s))
        if not self.client.wait_for_server(timeout_sec=server_timeout_s):
            return 'action server %s not available after %.0f s' % (
                self.action_name, server_timeout_s)
        log.info('waiting for TF %s->%s (timeout %.0f s)'
                 % (self.frame_id, self.base_frame, tf_timeout_s))
        ok = self.spin_until(
            lambda: self.tf_buffer.can_transform(self.frame_id,
                                                 self.base_frame, Time()),
            tf_timeout_s)
        if not ok:
            return 'TF %s->%s not available after %.0f s' % (
                self.frame_id, self.base_frame, tf_timeout_s)
        log.info('ready')
        return ''

    # ------------------------------------------------------------ one goal
    def run_goal(self, res: sc.GoalResult, timeout_s: float) -> None:
        log = self.get_logger()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = res.x
        goal.pose.pose.position.y = res.y
        qx, qy, qz, qw = yaw_to_quat(res.yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.last_feedback = None
        path0 = self.path_length
        t0_wall = time.monotonic()
        t0_sim = self.sim_now_s()

        def finish(status: str, message: str = '', t_end=None) -> None:
            # t_end = (wall, sim) end time; default: now
            end_wall, end_sim = t_end or (time.monotonic(), self.sim_now_s())
            res.status = status
            res.message = message
            res.duration_wall_s = end_wall - t0_wall
            res.duration_sim_s = end_sim - t0_sim
            res.start_sim_s = t0_sim
            res.end_sim_s = end_sim
            res.path_length_m = (self.path_length - path0
                                 if self.distance_topic else None)
            if self.last_feedback is not None:
                res.recoveries = int(self.last_feedback.number_of_recoveries)
            pose = self.lookup_pose()
            if pose is not None:
                res.set_final_pose(*pose)
            log.info('goal %d (%.2f, %.2f, %.2f): %s in %.1f s%s' % (
                res.index, res.x, res.y, res.yaw, status,
                res.duration_wall_s, (' - ' + message) if message else ''))

        log.info('goal %d -> (%.2f, %.2f, yaw %.2f) in %s, timeout %.0f s'
                 % (res.index, res.x, res.y, res.yaw, self.frame_id,
                    timeout_s))
        send_future = self.client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        # Acceptance counts against the same per-goal timeout.
        if not self.spin_until(send_future.done, timeout_s):
            finish('timeout', 'no goal response from the action server')
            return
        handle = send_future.result()
        if handle is None or not handle.accepted:
            finish('rejected', 'goal rejected by the action server')
            return
        self.active_handle = handle

        result_future = handle.get_result_async()
        remaining = timeout_s - (time.monotonic() - t0_wall)
        if not self.spin_until(result_future.done, max(0.0, remaining)):
            # durations end at the timeout, not after the cancel round trip
            t_end = (time.monotonic(), self.sim_now_s())
            cancel_future = handle.cancel_goal_async()
            self.spin_until(cancel_future.done, 10.0)
            # Give the server a moment to report the terminal state.
            self.spin_until(result_future.done, 5.0)
            self.active_handle = None
            finish('timeout', 'goal timed out after %.0f s (cancel sent)'
                   % timeout_s, t_end)
            return
        self.active_handle = None
        wrapped = result_future.result()
        status = _STATUS_NAMES.get(wrapped.status, 'error')
        msg = '' if status != 'error' else 'goal status %d' % wrapped.status
        finish(status, msg)

    def cancel_active(self) -> None:
        """Best effort: cancel the goal in progress (used on Ctrl+C).

        [unverified] After SIGINT the rclpy context may already be shut
        down, in which case the cancel request cannot be sent; the robot then
        keeps driving until Nav2 itself is stopped by the orchestrator.
        """
        handle, self.active_handle = self.active_handle, None
        if handle is None:
            return
        try:
            fut = handle.cancel_goal_async()
            self.spin_until(fut.done, 3.0)
        except Exception:  # noqa: BLE001
            pass


# ------------------------------------------------------------------- main
def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog='scenario_runner',
        description='Send the goals of a scenario YAML to Nav2 '
                    'NavigateToPose and write result.yaml.')
    ap.add_argument('--scenario', required=True, help='scenario YAML')
    ap.add_argument('--out', default='',
                    help='result.yaml path (default: do not write)')
    ap.add_argument('--run-name', default='')
    ap.add_argument('--mode', default='', choices=('',) + sc.MODES)
    ap.add_argument('--seed', type=int, default=None,
                    help='odometry-noise seed of this run (recorded only)')
    ap.add_argument('--action', default='/navigate_to_pose')
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--distance-topic', default='/odom_gt',
                    help="nav_msgs/Odometry for path length ('' = off)")
    ap.add_argument('--server-timeout', type=float, default=120.0,
                    help='wait for the action server [s]')
    ap.add_argument('--tf-timeout', type=float, default=120.0,
                    help='wait for TF frame_id->base_frame [s]')
    ap.add_argument('--stop-on-failure', action='store_true',
                    help='stop after the first failed goal')
    ap.add_argument('--allowed-frames', nargs='+',
                    default=list(sc.DEFAULT_ALLOWED_FRAMES))
    return ap


def main(args: Optional[Sequence[str]] = None) -> None:
    argv = list(sys.argv if args is None else args)
    cli = rclpy.utilities.remove_ros_args(args=argv)[1:]
    a = build_arg_parser().parse_args(cli)
    try:
        scenario = sc.load_scenario(a.scenario,
                                    allowed_frames=a.allowed_frames)
    except sc.ScenarioError as e:
        print('[scenario_runner] ERROR: %s' % e, file=sys.stderr)
        sys.exit(sc.EXIT_BAD_INPUT)

    result = sc.RunResult.for_scenario(scenario, run_name=a.run_name,
                                       mode=a.mode, seed=a.seed,
                                       start_wall=sc.now_iso())
    rclpy.init(args=argv)
    node = None
    try:
        node = ScenarioRunner(a.action, scenario.frame_id, a.base_frame,
                              a.distance_topic)
        err = node.wait_ready(a.server_timeout, a.tf_timeout)
        if err:
            node.get_logger().error(err)
            result.error = err
            result.exit_code_override = sc.EXIT_NOT_READY
        else:
            if scenario.settle_s > 0.0:
                node.get_logger().info('settling %.1f s' % scenario.settle_s)
                node.spin_for(scenario.settle_s)
            for g in result.goals:
                node.run_goal(g, scenario.goal_timeout_s)
                if not g.succeeded and a.stop_on_failure:
                    break
    except (KeyboardInterrupt, ExternalShutdownException):
        result.error = 'interrupted'
        result.exit_code_override = EXIT_INTERRUPTED
        if node is not None:
            node.cancel_active()
    finally:
        result.end_wall = sc.now_iso()
        if a.out:
            sc.write_result(a.out, result)
            print('[scenario_runner] wrote %s' % a.out)
        print('[scenario_runner] %d/%d goals succeeded, exit %d' % (
            result.n_succeeded, len(result.goals), result.exit_code))
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(result.exit_code)


if __name__ == '__main__':
    main()
