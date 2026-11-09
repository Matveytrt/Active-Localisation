"""External acceptance: source world, real camera->STag, lidar and collisions.

Source poses are used only to evaluate observations; no detector input is
generated from them. All robot movement is through /cmd_vel.
"""
import json
import math
from pathlib import Path
import time
import traceback

import numpy as np
from PIL import Image as PilImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from bridge_core import yaw_matrix
from world_source import audit, collision_boxes

output = Path('/workspace/world-verification')
output.mkdir(exist_ok=True)
result = {'status': 'running', 'checks': {}}
(output/'result.json').write_text(json.dumps(result, indent=2))


def seconds(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


def pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return np.array([p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])


def cloud(msg):
    assert msg.point_step == 16 and [f.name for f in msg.fields] == ['x', 'y', 'z', 'intensity']
    return np.frombuffer(bytes(msg.data), '<f4').reshape(-1, 4)[:, :3]


class Probe(Node):
    def __init__(self):
        super().__init__('isaac_world_acceptance')
        self.gt = self.noisy = self.image = self.status = None
        self.raw, self.detections, self.overlays = {}, {}, {}
        self.tf_edges = set()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom_gt', lambda m: setattr(self, 'gt', m), 50)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'noisy', m), 50)
        self.create_subscription(Image, '/torso_rgbd/image_raw', lambda m: setattr(self, 'image', m), qos_profile_sensor_data)
        self.create_subscription(String, '/isaac/status', lambda m: setattr(self, 'status', json.loads(m.data)), 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        for side in ('front', 'back'):
            self.create_subscription(PointCloud2, '/lidar_'+side+'/points',
                lambda m, side=side: self.raw.update({side: m}), qos_profile_sensor_data)
        for family in ('hd15', 'hd17'):
            self.create_subscription(Image, f'/isaac/stag/stag_{family}/detection_image_{family}',
                lambda m, family=family: self.overlays.update({family: m}), 10)

    def on_tf(self, msg):
        for transform in msg.transforms:
            self.tf_edges.add((transform.header.frame_id, transform.child_frame_id))
            if transform.child_frame_id.startswith(('stag_hd15_', 'stag_hd17_')):
                observations = self.detections.setdefault(transform.child_frame_id, [])
                if not observations or seconds(transform) != seconds(observations[-1]):
                    observations.append(transform)
                    del observations[:-30]

    def send(self, velocity):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = map(float, velocity)
        self.publisher.publish(msg)

    def wait(self, predicate, timeout=30, command=None):
        deadline, next_send = time.monotonic()+timeout, 0.
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError('world probe condition')
            if command is not None and time.monotonic() >= next_send:
                self.send(command)
                next_send = time.monotonic()+.04
            rclpy.spin_once(self, timeout_sec=.01)

    def service(self, name):
        client = self.create_client(Trigger, name)
        assert client.wait_for_service(timeout_sec=10), name
        future = client.call_async(Trigger.Request())
        self.wait(future.done)
        assert future.result().success, future.result().message
        self.destroy_client(client)

    def settle(self, duration=.5):
        start = seconds(self.gt)
        self.wait(lambda: seconds(self.gt)-start >= duration, command=[0, 0, 0])

    def drive(self, command, duration):
        start = seconds(self.gt)
        self.wait(lambda: seconds(self.gt)-start >= duration, timeout=90, command=command)
        self.send([0, 0, 0])
        self.settle()

    def reset(self):
        self.send([0, 0, 0])
        self.service('/isaac/reset')
        self.service('/isaac/reset_noise')
        self.settle()
        assert np.max(np.abs(pose(self.gt)-[-2, -.5, 0])) < 1e-7
        assert np.max(np.abs(pose(self.noisy)-pose(self.gt))) < 1e-7


def save_image(msg, path):
    image = PilImage.frombytes('RGB', (msg.width, msg.height), bytes(msg.data))
    if msg.encoding == 'bgr8':
        b, g, r = image.split()
        image = PilImage.merge('RGB', (r, g, b))
    image.save(path)


manifest = audit(Path('/repo'))
metadata = json.loads(Path('/repo/isaac/runtime/robot/manifest.json').read_text())
rclpy.init()
node = Probe()
try:
    print('WAITING_FOR_OWN_WORLD', flush=True)
    node.wait(lambda: node.gt is not None and node.noisy is not None and node.image is not None
              and node.status is not None and len(node.raw) == 2, timeout=360)
    assert node.status['scene'] == 'own-world'
    usd = node.status['world_validation']
    assert usd['collisions'] == 35 and usd['marker_faces'] == 24 and usd['shelf_mesh_parts'] == 12
    assert usd['shelf_triangles_total'] == 426288 and usd['max_transform_error'] < 1e-6
    result['checks']['usd_source_geometry'] = usd
    assert (node.image.width, node.image.height, node.image.encoding, node.image.header.frame_id) == (640, 480, 'rgb8', 'torso_rgbd_optical')
    node.reset()
    result['checks']['spawn_and_reset'] = pose(node.gt).tolist()
    save_image(node.image, output/'spawn.png')
    # Move from the actual source spawn to a safe front-row observation pose.
    node.drive([.2, 0, 0], 1.)
    yaw_error = math.pi/2-pose(node.gt)[2]
    node.drive([0, 0, .4], yaw_error/.4)
    node.drive([.2, 0, 0], .5/.2)
    result['checks']['observation_pose'] = pose(node.gt).tolist()
    # Pose accuracy is tested on preselected, centered targets, separately for
    # each family. Off-axis IPPE has a second planar orientation solution; keep
    # that diagnostic instead of claiming all off-axis normals are reliable.
    node.drive([.15, 0, 0], (.15-pose(node.gt)[1])/.15)
    frontal_checks = []
    for family, marker_id, target_x in (('hd15', 2, -1.65), ('hd17', 0, -1.5)):
        delta_x = target_x-pose(node.gt)[0]
        node.drive([0, -.15 if delta_x > 0 else .15, 0], abs(delta_x)/.15)
        node.detections.clear()
        target_name = f'stag_{family}_{marker_id}'
        node.wait(lambda: len(node.detections.get(target_name, [])) >= 8, timeout=30)
        frontal_checks.append((target_name, pose(node.gt).copy(), list(node.detections[target_name])))
        save_image(node.image, output/f'frontal_{family}.png')
        if family in node.overlays:
            save_image(node.overlays[family], output/f'detections_{family}.png')
    node.detections.clear()
    node.wait(lambda: any(k.startswith('stag_hd15_') and len(v) >= 5 for k, v in node.detections.items())
              and any(k.startswith('stag_hd17_') and len(v) >= 5 for k, v in node.detections.items()), timeout=30)
    node.settle(.5)
    save_image(node.image, output/'torso_markers.png')
    for family, overlay in node.overlays.items():
        save_image(overlay, output/f'detections_{family}.png')
    current = pose(node.gt)
    world_base = np.eye(4)
    world_base[:3, :3] = yaw_matrix(current[2])
    world_base[:3, 3] = [current[0], current[1], .16]
    world_camera = world_base @ np.array(metadata['base_to_links']['torso_rgbd'])
    accepted = []
    for name, observation_pose, samples in frontal_checks:
        sample_world_base = np.eye(4)
        sample_world_base[:3, :3] = yaw_matrix(observation_pose[2])
        sample_world_base[:3, 3] = [*observation_pose[:2], .16]
        sample_world_camera = sample_world_base @ np.array(metadata['base_to_links']['torso_rgbd'])
        family, marker_id = name[len('stag_'):].split('_')
        marker = next((m for m in manifest['markers'] if m['family'].lower() == family and m['id'] == int(marker_id)), None)
        assert marker is not None, ('unexpected decoded ID', name)
        assert len(samples) >= 8
        actual_marker = np.array(marker['transform'])
        face_offset = np.eye(4)
        face_offset[2, 3] = marker['size'][2]/2+.00001
        actual_marker = actual_marker @ face_offset
        expected = np.linalg.inv(sample_world_camera) @ actual_marker
        errors, angles, observations = [], [], []
        for sample in samples:
            assert sample.header.frame_id == 'torso_rgbd'
            p, q = sample.transform.translation, sample.transform.rotation
            translation = np.array([p.x, p.y, p.z])
            rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            errors.append(float(np.linalg.norm(translation-expected[:3, 3])))
            angles.append(float(math.degrees(math.acos(np.clip(rotation[:, 2] @ expected[:3, 2], -1, 1)))))
            observations.append({'stamp': seconds(sample), 'translation': translation.tolist(), 'quaternion_xyzw': [q.x, q.y, q.z, q.w]})
        tolerance = max(.04, np.linalg.norm(expected[:3, 3])*.05)
        accepted.append({'name': name, 'samples': observations, 'expected_translation': expected[:3, 3].tolist(),
            'robot_pose': observation_pose.tolist(),
            'median_error_m': float(np.median(errors)), 'translation_tolerance_m': tolerance,
            'median_normal_error_deg': float(np.median(angles))})
        result['checks']['real_stag'] = accepted
        assert np.median(errors) < tolerance, (name, errors, expected[:3, 3])
        assert np.median(angles) < 25, (name, 'face normal', angles)
    assert any(a['name'].startswith('stag_hd15_') for a in accepted)
    assert any(a['name'].startswith('stag_hd17_') for a in accepted)
    result['checks']['real_stag'] = accepted
    print('REAL_STAG_POSES_PASSED', [(a['name'], a['median_error_m']) for a in accepted], flush=True)
    # Measure both physical lidar clouds against the exact source collision surfaces.
    for side in ('front', 'back'):
        message = node.raw[side]
        assert (message.width, message.height, message.header.frame_id) == (120, 16, 'lidar_'+side)
        points = cloud(message)
        points = points[np.isfinite(points).all(axis=1)]
        assert len(points) > 800
        ext = np.array(metadata['base_to_links']['lidar_'+side])
        points = points @ ext[:3, :3].T+ext[:3, 3]
        points = points @ world_base[:3, :3].T+world_base[:3, 3]
        surface_errors = [abs(points[:, 2])]
        for box in collision_boxes(manifest):
            local = (points-box['center']) @ np.array(box['rotation'])
            delta = abs(local)-np.array(box['size'])/2
            error = np.where(np.any(delta > 0, axis=1), np.linalg.norm(np.maximum(delta, 0), axis=1), np.min(-delta, axis=1))
            surface_errors.append(error)
        p99 = float(np.quantile(np.min(np.array(surface_errors), axis=0), .99))
        assert p99 < .04, (side, p99)
        result['checks']['lidar_'+side] = {'finite_points': len(points), 'surface_error_p99_m': p99}
    # Drive into the first shelf, then hold the command: the footprint must stop.
    node.drive([.3, 0, 0], 5.)
    stopped = pose(node.gt)
    assert .54 < stopped[1] <= .57, stopped  # source front face .9 minus yawed half-extent .325
    node.drive([.3, 0, 0], 1.)
    assert np.max(abs(pose(node.gt)-stopped)) < 1e-7
    result['checks']['shelf_collision'] = stopped.tolist()
    node.reset()
    # Rear wall face y=-2.04749; robot half width .375 at yaw zero.
    node.drive([0, -.4, 0], 5.)
    stopped = pose(node.gt)
    assert -1.67249 <= stopped[1] < -1.64, stopped
    result['checks']['wall_collision'] = stopped.tolist()
    node.reset()
    # Negative control: look at the blank south wall, with every STag behind
    # the camera. Fresh marker TFs must disappear, not follow source poses.
    node.drive([0, 0, -.4], (math.pi/2)/.4)
    node.settle(1.)
    node.detections.clear()
    blank_start = seconds(node.gt)
    node.settle(1.)
    assert not any(seconds(s) >= blank_start for samples in node.detections.values() for s in samples)
    save_image(node.image, output/'negative_blank_wall.png')
    result['checks']['negative_control_no_fresh_marker_tf'] = True
    tf_owners = sorted(p.node_name for p in node.get_publishers_info_by_topic('/tf'))
    assert tf_owners == ['odom_noise_node', 'stag_hd15', 'stag_hd17'], tf_owners
    assert {(a, b) for a, b in node.tf_edges if b == 'base_link'} == {('odom', 'base_link')}
    assert all(a == 'torso_rgbd' for a, b in node.tf_edges if b.startswith('stag_'))
    result['checks']['tf'] = {'owners': tf_owners, 'observed_edges': sorted(node.tf_edges)}
    node.reset()
    result['checks']['rtf_observed'] = node.status['rtf']
    result['checks']['source_sha256'] = manifest['source_sha256']
    result['status'] = 'passed'
    print('AL_EXTERNAL_WORLD_PASSED', flush=True)
except BaseException:
    result['status'], result['error'] = 'failed', traceback.format_exc()
    print(result['error'], flush=True)
    raise
finally:
    node.send([0, 0, 0])
    (output/'result.json').write_text(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
