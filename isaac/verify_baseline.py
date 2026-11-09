"""Real Nav2 NavigateToPose acceptance on a fresh localization session."""
import json
import math
from pathlib import Path
import time
import traceback
import numpy as np
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from slam_probe import Probe, angle, matrix, planar, stamp

output = Path('/workspace/baseline-007')
output.mkdir(exist_ok=True)
result = {'status': 'running', 'goals': []}
(output/'result.json').write_text(json.dumps(result, indent=2))
rclpy.init()
node = Probe('isaac_baseline_acceptance', output)
active_handle = None
# This probe never publishes /cmd_vel during Nav2. The inherited publisher
# exists for mapping, but only the Nav2 controller/smoother drives this run.
try:
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    node.wait(lambda: node.gt is not None and node.odom is not None and node.grid is not None, timeout=180)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    node.wait(lambda: initial_pub.get_subscription_count() > 0, timeout=30)
    initial = PoseWithCovarianceStamped()
    initial.header.frame_id, initial.header.stamp = 'map', node.gt.header.stamp
    # Known scenario spawn, not a live GT correction; fixed once at episode start.
    initial.pose.pose.position.x, initial.pose.pose.position.y, initial.pose.pose.position.z = -2., -.5, .16
    initial.pose.pose.orientation.w = 1.
    initial.pose.covariance[0] = initial.pose.covariance[7] = .01
    initial.pose.covariance[35] = .01
    initial_pub.publish(initial)
    node.wait(lambda: node.estimate() is not None and client.server_is_ready(), timeout=180)
    # Action discovery precedes lifecycle activation. Wait for every component
    # that actually plans, follows, and publishes the velocity commands.
    for name in ('controller_server', 'planner_server', 'bt_navigator', 'velocity_smoother'):
        state_client = node.create_client(GetState, '/'+name+'/get_state')
        assert state_client.wait_for_service(timeout_sec=30), name
        deadline = time.monotonic()+120
        while True:
            state = state_client.call_async(GetState.Request())
            node.wait(state.done, timeout=10)
            if state.result().current_state.id == 3:
                break
            assert time.monotonic() < deadline, (name, 'not active')
            until = time.monotonic()+.2
            node.wait(lambda: time.monotonic() >= until, timeout=1)
        node.destroy_client(state_client)
    initialized_at = stamp(node.gt)
    node.wait(lambda: stamp(node.gt)-initialized_at > 1., timeout=20)
    # Initial map/world alignment is an evaluation transform only.
    result['initial_world_from_map'] = (matrix(planar(node.gt.pose.pose)) @ np.linalg.inv(matrix(node.estimate()))).tolist()
    result['initialization'] = 'fixed scenario spawn [-2,-.5,.16, yaw=0], one /initialpose'
    for target in ([-1., -.5, 0.], [1.9, -.5, math.pi/2], [2.65, 2.4, math.pi/2]):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id, goal.pose.header.stamp = 'map', node.gt.header.stamp
        goal.pose.pose.position.x, goal.pose.pose.position.y = target[:2]
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(target[2]/2), math.cos(target[2]/2)
        start = time.monotonic()
        future = client.send_goal_async(goal)
        node.wait(future.done, timeout=15)
        handle = future.result()
        assert handle.accepted, 'Nav2 rejected the goal'
        active_handle = handle
        action_result = handle.get_result_async()
        node.wait(action_result.done, timeout=180)
        status = action_result.result().status
        active_handle = None
        estimate = node.estimate()
        truth = planar(node.gt.pose.pose)
        estimated_error = np.linalg.norm(estimate[:2]-target[:2])
        yaw_error = abs(angle(estimate[2]-target[2]))
        expected_world = np.array(result['initial_world_from_map']) @ [*target[:2], 1.]
        truth_error = np.linalg.norm(truth[:2]-expected_world[:2])
        record = {'goal_map': target, 'action_status': status, 'wall_seconds': time.monotonic()-start,
            'estimated_pose': estimate.tolist(), 'gt': truth.tolist(), 'map_xy_error_m': float(estimated_error),
            'map_yaw_error_rad': yaw_error, 'gt_xy_error_m': float(truth_error)}
        result['goals'].append(record)
        (output/'result.json').write_text(json.dumps(result, indent=2))
        print('NAV2_GOAL_RESULT', json.dumps(record), flush=True)
        assert status == 4, record
        assert estimated_error <= .15 and yaw_error <= .10 and truth_error <= .25, record
    owners = [p.node_name for p in node.get_publishers_info_by_topic('/tf')]
    assert owners.count('rtabmap') == 1 and owners.count('odom_noise_node') == 1, owners
    result['tf_owners'] = owners
    result['status'] = 'passed'
    print('AL_BASELINE_NAV2_PASSED', flush=True)
except BaseException:
    result['status'], result['error'] = 'failed', traceback.format_exc()
    print(result['error'], flush=True)
    raise
finally:
    if active_handle is not None:
        cancel = active_handle.cancel_goal_async()
        node.wait(cancel.done, timeout=5)
    # Do not inject manual velocities into an autonomous-goal acceptance run.
    (output/'result.json').write_text(json.dumps(result, indent=2))
    node.close()
    rclpy.shutdown()
