"""External ROS acceptance probe: real DDS messages, motion and sensor geometry."""
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bridge_core import integrate, yaw_matrix

output = Path('/workspace/bridge-verification')
output.mkdir(exist_ok=True)
result = {"status": "running", "checks": {}}
(output/'result.json').write_text(json.dumps(result, indent=2))


def pose(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    return np.array([p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])


def seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec*1e-9


class Probe(Node):
    def __init__(self):
        super().__init__('isaac_acceptance_probe')
        self.gt = self.noisy = self.image = self.info = self.depth = self.joined = None
        self.raw = {}
        self.status = None
        self.edges, self.static_edges = set(), set()
        self.stamps = {name: [] for name in ('gt', 'rgb', 'depth', 'front', 'back', 'joined')}
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom_gt', self.on_gt, 50)
        self.create_subscription(Odometry, '/odom', lambda msg: setattr(self, 'noisy', msg), 50)
        self.create_subscription(Image, '/torso_rgbd/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/torso_rgbd/camera_info', lambda msg: setattr(self, 'info', msg), 10)
        self.create_subscription(Image, '/torso_rgbd/depth/image_raw', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(String, '/isaac/status', lambda msg: setattr(self, 'status', json.loads(msg.data)), 10)
        for side in ('front', 'back'):
            self.create_subscription(PointCloud2, f'/lidar_{side}/points',
                                     lambda msg, side=side: self.on_cloud(side, msg), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/lidar/points_joined', self.on_joined, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', lambda msg: self.edges.update((t.header.frame_id, t.child_frame_id) for t in msg.transforms), 100)
        static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, '/tf_static', lambda msg: self.static_edges.update((t.header.frame_id, t.child_frame_id) for t in msg.transforms), static_qos)

    def on_gt(self, msg):
        self.gt = msg
        self.stamps['gt'].append(seconds(msg))

    def on_image(self, msg):
        self.image = msg
        self.stamps['rgb'].append(seconds(msg))

    def on_depth(self, msg):
        self.depth = msg
        self.stamps['depth'].append(seconds(msg))

    def on_cloud(self, side, msg):
        self.raw[side] = msg
        self.stamps[side].append(seconds(msg))

    def on_joined(self, msg):
        self.joined = msg
        self.stamps['joined'].append(seconds(msg))

    def send(self, values):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = map(float, values)
        self.publisher.publish(msg)

    def wait(self, predicate, timeout=30, command=None):
        deadline, next_send = time.monotonic()+timeout, 0.
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('Condition timed out')
            if command is not None and time.monotonic() >= next_send:
                self.send(command)
                next_send = time.monotonic()+.05
            rclpy.spin_once(self, timeout_sec=.02)

    def drive(self, values, duration):
        start = seconds(self.gt)
        before = pose(self.gt)
        self.wait(lambda: seconds(self.gt)-start >= duration, timeout=120, command=values)
        elapsed = seconds(self.gt)-start
        self.send([0, 0, 0])
        after = pose(self.gt)
        # Drain the zero command and a few render frames while stationary.
        end = seconds(self.gt)
        self.wait(lambda: seconds(self.gt)-end > .3, timeout=30, command=[0, 0, 0])
        return before, after, elapsed

    def service(self, name):
        client = self.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=10):
            raise TimeoutError(name)
        future = client.call_async(Trigger.Request())
        self.wait(future.done, timeout=20)
        response = future.result()
        self.destroy_client(client)
        assert response.success, response.message

    def reset(self):
        self.send([0, 0, 0])
        self.service('/isaac/reset')
        self.service('/isaac/reset_noise')
        start = seconds(self.gt)
        self.wait(lambda: seconds(self.gt)-start > .3, timeout=30, command=[0, 0, 0])
        assert np.linalg.norm(pose(self.gt)) < 1e-7, pose(self.gt)
        assert np.linalg.norm(pose(self.noisy)) < 1e-7, pose(self.noisy)


def cloud_array(msg):
    offset = {f.name: f.offset for f in msg.fields}
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4']*3,
                      'offsets': [offset[k] for k in ('x', 'y', 'z')], 'itemsize': msg.point_step})
    values = np.frombuffer(bytes(msg.data), dtype=dtype)
    return np.column_stack([values[k] for k in ('x', 'y', 'z')])


def color_centers(msg):
    rgb = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3).astype(float)
    centers = {}
    for index, color in enumerate(('red', 'green', 'blue')):
        other = np.max(np.delete(rgb, index, axis=2), axis=2)
        ys, xs = np.where((rgb[:, :, index] > other*1.5+15) & (rgb[:, :, index] > 50))
        if len(xs) > 10:
            centers[color] = [float(np.median(xs)), float(np.median(ys))]
    return centers


rclpy.init()
node = Probe()
try:
    print('WAITING_FOR_BRIDGE', flush=True)
    node.wait(lambda: node.gt is not None and node.noisy is not None and node.image is not None
              and node.info is not None and node.depth is not None and len(node.raw) == 2
              and node.joined is not None and node.status is not None, timeout=360)
    node.reset()
    print('BRIDGE_MESSAGES_READY', flush=True)
    assert node.image.encoding == 'rgb8' and node.image.header.frame_id == 'torso_rgbd_optical'
    assert (node.image.width, node.image.height) == (640, 480)
    assert node.depth.encoding == '32FC1'
    expected_fx = 640/(2*math.tan(1.089/2))
    assert abs(node.info.k[0]-expected_fx) < .01
    assert abs(node.info.k[4]-expected_fx) < .01
    assert list(node.info.d) == [0.]*5
    centers = color_centers(node.image)
    assert set(centers) == {'red', 'green', 'blue'}, centers
    expected = {'red': [320-expected_fx*.45/2.43, 240],
                'green': [320+expected_fx*.45/2.43, 240], 'blue': [320, 240-expected_fx*.45/2.43]}
    depth = np.frombuffer(bytes(node.depth.data), '<f4').reshape(480, 640)
    for color in expected:
        assert np.max(np.abs(np.array(centers[color])-expected[color])) < 7, (color, centers, expected)
        u, v = np.round(centers[color]).astype(int)
        assert abs(float(np.median(depth[v-3:v+4, u-3:u+4]))-2.42) < .04
    from PIL import Image as PilImage
    PilImage.frombytes('RGB', (640, 480), bytes(node.image.data)).save(output/'torso_calibration.png')
    result['checks']['camera'] = {'fx': node.info.k[0], 'centers': centers, 'expected_centers': expected,
                                  'frame': node.image.header.frame_id, 'depth_target_m': float(depth[240, int(centers['red'][0])])}

    metadata = json.loads(Path('/repo/isaac/runtime/robot/manifest.json').read_text())
    for side in ('front', 'back'):
        msg = node.raw[side]
        assert (msg.width, msg.height, msg.header.frame_id) == (120, 16, 'lidar_'+side)
        points = cloud_array(msg)
        points = points[np.isfinite(points).all(axis=1)]
        assert len(points) > 800
        ext = np.array(metadata['base_to_links']['lidar_'+side])
        world_points = points @ ext[:3, :3].T + ext[:3, 3] + [0, 0, .16]
        error = np.min(np.column_stack([abs(world_points[:, 0]-4), abs(world_points[:, 0]+4),
            abs(world_points[:, 1]-3), abs(world_points[:, 1]+3), abs(world_points[:, 2])]), axis=1)
        assert np.quantile(error, .99) < .04, (side, np.quantile(error, .99))
        result['checks']['lidar_'+side] = {'finite_points': len(points), 'surface_error_p99_m': float(np.quantile(error, .99))}
    assert node.joined.header.frame_id == 'base_link' and node.joined.width*node.joined.height > 1500
    result['checks']['joined'] = {'frame': node.joined.header.frame_id, 'points': node.joined.width*node.joined.height}

    motions = []
    for values, duration in (([.2, 0, 0], 2.), ([0, .15, 0], 2.), ([0, 0, .3], 2.)):
        node.reset()
        before, after, elapsed = node.drive(values, duration)
        expected_pose = integrate(before, values, elapsed)
        assert np.max(np.abs(after-expected_pose)) < .03, (values, after, expected_pose)
        motions.append({'command': values, 'elapsed_sim_s': elapsed, 'actual': after.tolist(), 'expected': expected_pose.tolist()})
        print('MOTION_PASSED', values, after.tolist(), flush=True)
    rotated = color_centers(node.image)
    assert rotated['red'][0] > centers['red'][0]+200, rotated
    result['checks']['camera_moves_with_robot'] = rotated
    before, after, elapsed = node.drive([.2, 0, 0], 1.)
    assert np.max(np.abs(after-integrate(before, [.2, 0, 0], elapsed))) < .03
    motions.append({'rotated_body_forward': (after-before).tolist()})
    result['checks']['motion'] = motions
    # Native sensor transforms must also follow the moving base in the USD scene.
    for side in ('front', 'back'):
        points = cloud_array(node.raw[side])
        points = points[np.isfinite(points).all(axis=1)]
        ext = np.array(metadata['base_to_links']['lidar_'+side])
        current = pose(node.gt)
        body_points = points @ ext[:3, :3].T + ext[:3, 3]
        world_points = body_points @ yaw_matrix(current[2]).T + [current[0], current[1], .16]
        error = np.min(np.column_stack([abs(world_points[:, 0]-4), abs(world_points[:, 0]+4),
            abs(world_points[:, 1]-3), abs(world_points[:, 1]+3), abs(world_points[:, 2])]), axis=1)
        assert np.quantile(error, .99) < .04, (side, 'moving base', np.quantile(error, .99))
        result['checks']['lidar_'+side]['moving_surface_error_p99_m'] = float(np.quantile(error, .99))

    node.reset()
    node.send([.2, 0, 0])
    initial_time = seconds(node.gt)
    node.wait(lambda: seconds(node.gt)-initial_time > 1.3, timeout=60)
    stopped = pose(node.gt)
    assert .002 < stopped[0] < .13, stopped
    initial_time = seconds(node.gt)
    node.wait(lambda: seconds(node.gt)-initial_time > .5, timeout=30)
    assert np.max(np.abs(pose(node.gt)-stopped)) < .001
    result['checks']['watchdog'] = {'stopped_pose': stopped.tolist()}

    node.reset()
    _, after, _ = node.drive([.6, 0, 0], 8.)
    assert 3.64 < after[0] <= 3.675+1e-6, after
    result['checks']['wall_collision'] = {'final_x': float(after[0]), 'maximum_safe_x': 3.675}
    assert np.linalg.norm(pose(node.noisy)-pose(node.gt)) > 1e-5
    result['checks']['noise_separated_from_gt'] = {'error': (pose(node.noisy)-pose(node.gt)).tolist()}
    node.reset()
    result['checks']['reset'] = {'gt': pose(node.gt).tolist(), 'noisy': pose(node.noisy).tolist()}

    assert ('world', 'base_link') not in node.edges
    assert node.edges == {('odom', 'base_link')}, node.edges
    assert ('torso_rgbd', 'torso_rgbd_optical') in node.static_edges
    assert len(node.static_edges) == 38, len(node.static_edges)
    tf_owners = [p.node_name for p in node.get_publishers_info_by_topic('/tf')]
    assert tf_owners == ['odom_noise_node'], tf_owners
    result['checks']['tf'] = {'dynamic': sorted(node.edges), 'static_count': len(node.static_edges), 'owners': tf_owners}
    rates = {}
    for name, stamps in node.stamps.items():
        stamps = np.array(stamps)
        rates[name] = float((len(stamps)-1)/(stamps[-1]-stamps[0]))
        assert np.all(np.diff(stamps) >= -1e-7), name
        if name in ('rgb', 'depth'):
            assert np.all(np.diff(stamps) > 0), (name, 'duplicate/stale frame stamps')
    assert 45 < rates['gt'] < 55, rates
    assert 25 < rates['rgb'] < 35, rates
    assert 25 < rates['depth'] < 35, rates
    assert 8 < rates['front'] < 12 and 8 < rates['back'] < 12, rates
    result['checks']['sim_time_rates_hz'] = rates
    result['checks']['rtf_observed'] = node.status.get('rtf')
    result['checks']['model_alignment_error'] = node.status.get('model_alignment_error')
    assert result['checks']['model_alignment_error'] < 3e-5
    result['status'] = 'passed'
    print('AL_EXTERNAL_BRIDGE_PASSED', json.dumps(result), flush=True)
except BaseException as exc:
    import traceback
    result['status'], result['error'] = 'failed', traceback.format_exc()
    print(result['error'], flush=True)
    raise
finally:
    node.send([0, 0, 0])
    (output/'result.json').write_text(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
