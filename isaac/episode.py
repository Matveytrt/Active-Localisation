"""Isaac readiness/evaluation around the original ScenarioRunner, no Twist publisher."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import traceback

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, Path as NavPath
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from al_eval.scenario_runner_node import ScenarioRunner
from al_eval import scenario as sc
from al_eval.tum import TumWriter, quat_to_yaw
from experiment_config import noise_parameters


class Episode(ScenarioRunner):
    def __init__(self, output):
        super().__init__('/navigate_to_pose', 'map', 'base_link', '/odom_gt')
        self.output, self.gt = output, None
        self.plans = (output/'plans.jsonl').open('w')
        self.feedback = (output/'feedback.jsonl').open('w')
        self.clocks = (output/'clock-wall.jsonl').open('w')
        self.last_clock = -1.
        self.noisy = TumWriter(str(output/'noisy.tum'))
        self.create_subscription(Odometry, '/odom_gt', self.truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.noise, qos_profile_sensor_data)
        self.create_subscription(NavPath, '/plan', self.plan, 10)
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)

    def clock(self, msg):
        stamp = msg.clock.sec+msg.clock.nanosec*1e-9
        if stamp-self.last_clock >= .1:
            self.clocks.write(json.dumps({'sim_s': stamp, 'wall': time.monotonic()})+'\n')
            self.last_clock = stamp

    def truth(self, msg):
        self.gt = msg

    def noise(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.noisy.write_stamp(msg.header.stamp.sec, msg.header.stamp.nanosec,
                               p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def plan(self, msg):
        points = []
        for pose in msg.poses:
            p, q = pose.pose.position, pose.pose.orientation
            points.append([p.x, p.y, quat_to_yaw(q.x, q.y, q.z, q.w)])
        self.plans.write(json.dumps({'sim_s': self.sim_now_s(), 'frame': msg.header.frame_id,
                                    'points': points})+'\n')

    def _on_feedback(self, msg):
        super()._on_feedback(msg)
        fb = msg.feedback
        self.feedback.write(json.dumps({'sim_s': self.sim_now_s(), 'wall': time.monotonic(),
            'recoveries': fb.number_of_recoveries, 'distance_remaining': fb.distance_remaining})+'\n')

    def require(self, condition, timeout=60):
        if not self.spin_until(condition, timeout):
            raise TimeoutError('Episode readiness timed out')

    def parameters(self, name, keys):
        client = self.create_client(GetParameters, '/'+name+'/get_parameters')
        assert client.wait_for_service(timeout_sec=20), name
        request = GetParameters.Request(names=keys)
        future = client.call_async(request)
        self.require(future.done, 20)
        values = {}
        for key, val in zip(keys, future.result().values):
            values[key] = {1: val.bool_value, 2: val.integer_value, 3: val.double_value,
                          4: val.string_value, 9: list(val.string_array_value)}.get(val.type)
        self.destroy_client(client)
        return values

    def ready(self, seed, mode):
        self.require(lambda: self.gt is not None)
        pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.require(lambda: pub.get_subscription_count() > 0)
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id, initial.header.stamp = 'map', self.gt.header.stamp
        initial.pose.pose.position.x, initial.pose.pose.position.y, initial.pose.pose.position.z = -2., -.5, .16
        initial.pose.pose.orientation.w = 1.
        initial.pose.covariance[0] = initial.pose.covariance[7] = initial.pose.covariance[35] = .01
        pub.publish(initial)  # one fixed known scenario spawn, no live GT correction
        error = self.wait_ready(120, 120)
        assert not error, error
        for name in ('controller_server', 'planner_server', 'bt_navigator', 'velocity_smoother'):
            client = self.create_client(GetState, '/'+name+'/get_state')
            assert client.wait_for_service(timeout_sec=30), name
            deadline = time.monotonic()+150
            while True:
                future = client.call_async(GetState.Request())
                self.require(future.done, 10)
                if future.result().current_state.id == 3:
                    break
                assert time.monotonic() < deadline, (name, 'not active')
                self.spin_for(.2)
            self.destroy_client(client)
        self.spin_for(2.)
        expected_noise = noise_parameters(Path('/repo'), seed, float(os.environ.get('AL_ISAAC_NOISE_SCALE', '1')))['odom_noise_node']['ros__parameters']
        noise_keys = ['seed', 'k_xx', 'k_xr', 'k_yy', 'k_yr', 'k_rr', 'k_rt',
                      'scale_x', 'scale_y', 'scale_yaw', 'yaw_drift_per_m']
        checks = {'noise': self.parameters('odom_noise_node', noise_keys),
            'planner': self.parameters('planner_server', ['GridBased.plugin'] +
                ([] if mode == 'baseline' else ['GridBased.scorer', 'GridBased.landmarks_file'])),
            'controller': self.parameters('controller_server', ['FollowPath.vx_max', 'FollowPath.vy_max', 'FollowPath.wz_max']),
            'slam': self.parameters('rtabmap', ['Rtabmap/MemoryThr', 'map_filter_radius', 'map_filter_angle', 'map_cache_loaded_on_init']),
            'tf_owners': [p.node_name for p in self.get_publishers_info_by_topic('/tf')]}
        assert checks['noise']['seed'] == seed, checks
        assert all(checks['noise'][k] == expected_noise[k] for k in noise_keys), checks['noise']
        if mode != 'baseline':
            checks['layer'] = self.parameters('global_costmap/global_costmap',
                ['loc_layer.enabled', 'loc_layer.scorer', 'loc_layer.landmarks_file'])
            assert checks['planner']['GridBased.scorer'] == ('null' if mode == 'control' else 'marker_soft_count'), checks
            assert checks['layer']['loc_layer.enabled'] == (mode == 'active'), checks
        assert checks['tf_owners'].count('rtabmap') == checks['tf_owners'].count('odom_noise_node') == 1, checks
        (self.output/'runtime-parameters.json').write_text(json.dumps(checks, indent=2))
        return {'known_spawn': [-2., -.5, .16, 0.], 'initial_estimate': self.lookup_pose(),
                'initial_gt': [self.gt.pose.pose.position.x, self.gt.pose.pose.position.y]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--mode', required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--complete-scenario', action='store_true',
                        help='Record every planned goal even when a preceding goal fails acceptance')
    args = parser.parse_args()
    output = Path(args.output)
    scenario = sc.load_scenario(args.scenario)
    result = sc.RunResult.for_scenario(scenario, run_name=output.name, mode=args.mode,
                                      seed=args.seed, start_wall=sc.now_iso())
    acceptance = {'status': 'running', 'goals': []}
    rclpy.init(args=['episode', '--ros-args', '-p', 'use_sim_time:=true'])
    node = Episode(output)
    start_wall = time.monotonic()
    try:
        acceptance['initialization'] = node.ready(args.seed, args.mode)
        node.spin_for(scenario.settle_s)
        sim0 = node.sim_now_s()
        wall0 = time.monotonic()
        all_checks = True
        for goal in result.goals:
            node.run_goal(goal, scenario.goal_timeout_s)
            truth = node.gt.pose.pose.position
            gt_error = math.hypot(truth.x-goal.x, truth.y-goal.y)
            acceptance['goals'].append({'index': goal.index, 'status': goal.status,
                'gt_xy_error_m': gt_error, 'estimated_xy_error_m': goal.final_xy_error_m,
                'estimated_yaw_error_rad': goal.final_yaw_error_rad})
            passed = goal.succeeded and goal.final_xy_error_m is not None and goal.final_yaw_error_rad is not None
            passed = passed and goal.final_xy_error_m <= .15 and goal.final_yaw_error_rad <= .10 and gt_error <= .30
            acceptance['goals'][-1]['within_acceptance'] = bool(passed)
            all_checks = all_checks and passed
            if not args.complete_scenario:
                assert passed, acceptance
        acceptance['navigation_rtf'] = (node.sim_now_s()-sim0)/(time.monotonic()-wall0)
        acceptance['status'] = 'passed' if all_checks else 'failed'
        if not all_checks:
            acceptance['error'] = 'One or more goals failed action or pose acceptance; all planned goals attempted'
    except BaseException:
        acceptance['status'], acceptance['error'] = 'failed', traceback.format_exc()
        result.error = acceptance['error']
        result.exit_code_override = 1
        node.cancel_active()
        print(acceptance['error'], flush=True)
    finally:
        acceptance['wall_seconds'] = time.monotonic()-start_wall
        result.end_wall = sc.now_iso()
        sc.write_result(str(output/'result.yaml'), result)
        (output/'acceptance.json').write_text(json.dumps(acceptance, indent=2))
        node.noisy.close()
        node.plans.close()
        node.feedback.close()
        node.clocks.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if acceptance['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
