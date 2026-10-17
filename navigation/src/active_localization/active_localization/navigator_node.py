"""al_navigator — integration option A (see _team/knowledge/ARCH.md).

Serves nav2_msgs/NavigateToPose on `al/navigate_to_pose` (same type as
bt_navigator, so the scenario runner only switches the action name by mode)
and runs a minimal navigate loop itself:

    every 1/replan_hz:  ComputePathToPose (planner_server, same Theta*)
                        -> plan_yaws(scorer)           (yaw lever)
                        -> FollowPath (controller_server, same MPPI)
    until FollowPath succeeds / the goal is cancelled / too many failures.

Known asymmetry vs baseline: no BT recovery ladder here (spin/backup/clear).
Option B (C++ planner wrapper, BT unchanged) removes it — ADR-001.

For the controller to actually follow our headings MPPI must use path
orientations and stop forcing "face forward" — see
config/nav2_active_overrides.yaml.

[unverified] Not run in ROS yet (no ROS in the LEAD environment): action
names, TF availability and controller pre-emption by a new FollowPath goal
are to be checked by ENG.
"""

import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from .config import load_config
from .geometry import quaternion_from_yaw, yaw_from_quaternion
from .yaw_planner import plan_yaws


def _wait(future, timeout_s):
    """Block the calling (worker) thread until the future is done."""
    t0 = time.monotonic()
    while not future.done():
        if timeout_s is not None and time.monotonic() - t0 > timeout_s:
            return False
        time.sleep(0.01)
    return True


class ALNavigator(Node):
    def __init__(self):
        super().__init__('al_navigator')
        self.declare_parameter('config_file', '')
        self.declare_parameter('scorer', '')       # override scorer.name
        cfg_path = self.get_parameter('config_file').value
        if not cfg_path:
            raise RuntimeError('parameter config_file is required')
        self.cfg = load_config(cfg_path, scorer_override=self.get_parameter('scorer').value)
        nav = self.cfg.navigator
        self.planner_id = nav.get('planner_id', 'GridBased')
        self.controller_id = nav.get('controller_id', 'FollowPath')
        self.replan_period = 1.0 / float(nav.get('replan_hz', 1.0))
        self.global_frame = nav.get('global_frame', 'map')
        self.robot_frame = nav.get('robot_frame', 'base_link')
        self.server_timeout = float(nav.get('server_timeout_s', 10.0))
        self.max_plan_fail = int(nav.get('max_planner_failures', 5))
        self.max_ctrl_fail = int(nav.get('max_controller_failures', 3))

        cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.planner = ActionClient(self, ComputePathToPose,
                                    nav.get('planner_action', 'compute_path_to_pose'),
                                    callback_group=cb)
        self.controller = ActionClient(self, FollowPath,
                                       nav.get('controller_action', 'follow_path'),
                                       callback_group=cb)
        self.plan_pub = self.create_publisher(Path, 'al/oriented_plan', 1)
        self.server = ActionServer(
            self, NavigateToPose, nav.get('action_name', 'al/navigate_to_pose'),
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb)
        self.get_logger().info(
            f'al_navigator ready: scorer={self.cfg.scorer.name}, '
            f'replan={1.0 / self.replan_period:.1f} Hz')

    # ---------------------------------------------------------------- helpers
    def _robot_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame,
                                                rclpy.time.Time())
        except Exception as e:  # noqa: BLE001 — any TF error -> unknown yaw
            self.get_logger().warn(f'TF {self.global_frame}->{self.robot_frame}: {e}',
                                   throttle_duration_sec=5.0)
            return None
        q = t.transform.rotation
        return yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _compute_path(self, goal_pose):
        req = ComputePathToPose.Goal()
        req.goal = goal_pose
        req.planner_id = self.planner_id
        req.use_start = False
        gh_future = self.planner.send_goal_async(req)
        if not _wait(gh_future, self.server_timeout):
            return None
        gh = gh_future.result()
        if not gh.accepted:
            return None
        res_future = gh.get_result_async()
        if not _wait(res_future, self.server_timeout):
            gh.cancel_goal_async()
            return None
        res = res_future.result()
        if res.status != GoalStatus.STATUS_SUCCEEDED or not res.result.path.poses:
            return None
        return res.result.path

    def _orient(self, path, goal_pose):
        xy = np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
        q = goal_pose.pose.orientation
        plan = plan_yaws(xy, self.cfg.scorer, self.cfg.yaw_planner,
                         start_yaw=self._robot_yaw(),
                         goal_yaw=yaw_from_quaternion(q.x, q.y, q.z, q.w))
        out = Path()
        out.header = path.header
        for (x, y), yaw in zip(plan.xy, plan.yaw):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            qx, qy, qz, qw = quaternion_from_yaw(float(yaw))
            ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
            ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
            out.poses.append(ps)
        return out, plan

    def _send_follow(self, path):
        req = FollowPath.Goal()
        req.path = path
        req.controller_id = self.controller_id
        gh_future = self.controller.send_goal_async(req)
        if not _wait(gh_future, self.server_timeout):
            return None
        gh = gh_future.result()
        return gh if gh.accepted else None

    # ---------------------------------------------------------------- execute
    def _execute(self, goal_handle):
        goal_pose = goal_handle.request.pose
        result = NavigateToPose.Result()
        for client, name in ((self.planner, 'planner'), (self.controller, 'controller')):
            if not client.wait_for_server(timeout_sec=self.server_timeout):
                self.get_logger().error(f'{name} action server not available')
                goal_handle.abort()
                return result

        plan_fail = ctrl_fail = 0
        follow_gh = None
        follow_res = None
        t_start = self.get_clock().now()
        next_plan = 0.0
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                if follow_gh is not None:
                    follow_gh.cancel_goal_async()
                goal_handle.canceled()
                return result

            now = time.monotonic()
            if now >= next_plan:
                next_plan = now + self.replan_period
                path = self._compute_path(goal_pose)
                if path is None:
                    plan_fail += 1
                    self.get_logger().warn(f'planning failed ({plan_fail}/{self.max_plan_fail})')
                    if plan_fail >= self.max_plan_fail:
                        break
                else:
                    plan_fail = 0
                    oriented, plan = self._orient(path, goal_pose)
                    self.plan_pub.publish(oriented)
                    # A new FollowPath goal pre-empts the running one; nav2's
                    # SimpleActionServer ends the old goal (aborted/cancelled
                    # [unverified]), so we drop the old result future here and
                    # never count that as a controller failure.
                    gh = self._send_follow(oriented)
                    if gh is not None:
                        follow_gh = gh
                        follow_res = gh.get_result_async()

            if follow_res is not None and follow_res.done():
                status = follow_res.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    goal_handle.succeed()
                    return result
                if status == GoalStatus.STATUS_ABORTED:
                    ctrl_fail += 1
                    self.get_logger().warn(f'controller aborted ({ctrl_fail}/{self.max_ctrl_fail})')
                    if ctrl_fail >= self.max_ctrl_fail:
                        break
                    next_plan = 0.0        # replan immediately
                follow_res = None

            fb = NavigateToPose.Feedback()
            fb.navigation_time = (self.get_clock().now() - t_start).to_msg()
            goal_handle.publish_feedback(fb)
            time.sleep(0.05)

        if follow_gh is not None:
            follow_gh.cancel_goal_async()
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ALNavigator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
