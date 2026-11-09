"""Own Astribot: planar base, RTX torso RGB/depth, PhysX 3-D lidars, ROS Humble.

The imported fixed model is visual geometry. A swept rectangular footprint guards
kinematic movement against the same static boxes that PhysX uses for lidar rays.
"""
import argparse
import array
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time
import traceback
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser()
parser.add_argument("--max-wall-seconds", type=float, default=900)
parser.add_argument("--scene", choices=("room", "own-world"), default="room")
parser.add_argument("--spawn", type=float, nargs=3, metavar=('X', 'Y', 'YAW'))
parser.add_argument("--seed", type=int, default=42)
args, _ = parser.parse_known_args()
if not 0 <= args.seed < 2**32-1:
    parser.error('seed must be in [0, 2**32-2]')
repo = Path(__file__).resolve().parents[1]
out = repo / "isaac/runtime/bridge"
out.mkdir(parents=True, exist_ok=True)
result_path = out / "result.json"
result_path.unlink(missing_ok=True)
(out / "ready.json").unlink(missing_ok=True)
sys.argv.extend(["--/renderer/enabled=rtx", "--/renderer/active=rtx"])
from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "active_gpu": 0, "multi_gpu": False,
                     "width": 640, "height": 480})
node = None
try:
    import numpy as np
    import omni.usd
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import FixedCuboid, VisualCuboid
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.sensors.camera import Camera
    from PIL import Image as PilImage
    from bridge_core import PlanarBase, ROOM_BOXES, beam_directions, sensor_specs, yaw_matrix, quaternion_xyzw
    from prepare_robot import rpy_matrix
    from physx_lidars import PhysXLidars

    context = omni.usd.get_context()
    deadline = time.monotonic() + 180
    while "rtx" not in context.get_attached_hydra_engine_names():
        if time.monotonic() > deadline:
            raise TimeoutError("RTX startup exceeded 180 seconds")
        app.update()
        time.sleep(.05)
    print("AL_BRIDGE_RTX_READY", flush=True)
    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.sensors.physx")
    app.update()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage

    package = repo / "gazebo/src/world_and_robot"
    specs = sensor_specs(package)
    model_dir = repo / "isaac/runtime/robot"
    metadata = json.loads((model_dir / "manifest.json").read_text())
    for name, path in (("urdf", package / "urdf_astribot/astribot_whole_body_with_head.urdf"),
                       ("pose", package / "config/astribot_pose.yaml")):
        assert hashlib.sha256(path.read_bytes()).hexdigest() == metadata[name+"_sha256"], "Re-import the changed robot first"
    frames = {name: np.array(value) for name, value in metadata["base_to_links"].items()}
    stage = context.get_stage()
    root = UsdGeom.Xform.Define(stage, "/World/Robot").GetPrim()
    model = UsdGeom.Xform.Define(stage, "/World/Robot/Model").GetPrim()
    model.GetReferences().AddReference(str(model_dir / "asset/astribot.usd"), "/astribot")
    app.update()
    # Author overrides in our scene, leaving the imported asset untouched.
    for _ in range(3):
        for prim in list(stage.Traverse()):
            if str(prim.GetPath()).startswith("/World/Robot/") and prim.IsInstance():
                prim.SetInstanceable(False)
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith("/World/Robot/"):
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if prim.IsA(UsdPhysics.Joint):
            prim.SetActive(False)
    root_op = UsdGeom.Xformable(root).AddTransformOp()
    scene_manifest = None
    boxes, spawn = ROOM_BOXES, (0., 0., 0.)
    if args.scene == 'own-world':
        from world_usd import build
        from world_source import collision_boxes
        scene_path, scene_manifest = build(repo)
        environment = UsdGeom.Xform.Define(stage, '/World/Environment').GetPrim()
        environment.GetReferences().AddReference(str(scene_path), '/Environment')
        boxes = collision_boxes(scene_manifest)
        spawn = (*scene_manifest['spawn_xyz'][:2], 0.)
    if args.spawn is not None:
        spawn = tuple(args.spawn)
    base = PlanarBase(boxes, spawn=spawn)
    from bridge_core import intersects_box
    assert not any(intersects_box(base.pose, base.half_extent, box) for box in boxes), 'Spawn intersects a source collision'
    base_z = .16  # Same base_link height above floor as the Gazebo spawn.

    def update_robot():
        matrix = np.eye(4)
        matrix[:3, :3] = yaw_matrix(base.pose[2])
        matrix[:3, 3] = [base.pose[0], base.pose[1], base_z]
        root_op.Set(Gf.Matrix4d(*matrix.T.flatten().tolist()))

    update_robot()
    world = World(stage_units_in_meters=1., physics_dt=1/60, rendering_dt=1/60)
    if args.scene == 'room':
        world.scene.add(FixedCuboid("/World/Floor", name="Floor", position=np.array([0., 0., -.1]),
        scale=np.array([20., 20., .2]), color=np.array([.18, .2, .24])))
    for box in (ROOM_BOXES if args.scene == 'room' else []):
        world.scene.add(FixedCuboid("/World/"+box["name"], name=box["name"],
            position=np.array(box["center"]), scale=np.array(box["size"]), color=np.array([.55, .57, .6])))
    # Visual calibration targets are not physical obstacles or STag landmarks.
    cz = base_z + frames["torso_rgbd"][2, 3]
    targets = (("Red", [2.5, .45, cz], [1., .02, .02]),
                              ("Green", [2.5, -.45, cz], [.02, 1., .02]),
                              ("Blue", [2.5, 0., cz+.45], [.02, .02, 1.]))
    for name, xyz, color in (targets if args.scene == 'room' else []):
        world.scene.add(VisualCuboid("/World/Target"+name, name="Target"+name,
            position=np.array(xyz), scale=np.array([.02, .22, .22]), color=np.array(color)))
    UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(350)
    cam_spec = specs["camera"]
    camera = Camera("/World/Robot/TorsoCamera", resolution=(cam_spec["width"], cam_spec["height"]))
    optical_rotation = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    camera_body = frames["torso_rgbd"]
    q = quaternion_xyzw(camera_body[:3, :3])
    camera.set_local_pose(translation=camera_body[:3, 3], orientation=q[[3, 0, 1, 2]], camera_axes="world")
    lidars = PhysXLidars(specs, frames)
    world.reset()
    update_robot()
    camera.initialize()
    camera.set_focal_length(camera.get_horizontal_aperture() / (2*np.tan(cam_spec["hfov"]/2)))
    camera.set_clipping_range(cam_spec["near"], cam_spec["far"])
    camera.add_distance_to_image_plane_to_frame()
    deadline = time.monotonic() + 180
    while time.monotonic() < deadline:
        lidars.refill()
        world.step(render=True)
        rgba, depth = camera.get_rgba(), camera.get_depth()
        if (rgba is not None and rgba.shape == (480, 640, 4) and np.std(rgba[:, :, :3]) > 5
                and depth is not None and depth.shape == (480, 640) and lidars.ready()):
            break
        time.sleep(.02)
    else:
        raise TimeoutError("RGB/depth warm-up exceeded 180 seconds")
    PilImage.fromarray(rgba).save(out / "torso_initial.png")
    from omni.kit.viewport.utility import get_active_viewport
    viewport = get_active_viewport()
    if viewport:
        viewport.updates_enabled = False  # Keep the actual torso render product active.
    beams = {side: beam_directions(specs[side]) for side in ("front", "back")}
    sensor_seeds = {'front': args.seed, 'back': args.seed+1}
    rngs = {side: np.random.default_rng(seed) for side, seed in sensor_seeds.items()}
    fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(("x", "y", "z", "intensity"))]

    def stamp(value):
        ns = round(float(value)*1e9)
        return Time(sec=ns//1_000_000_000, nanosec=ns % 1_000_000_000)

    def transform(parent, child, matrix):
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, matrix[:3, 3])
        quat = quaternion_xyzw(matrix[:3, :3])
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, quat)
        return msg

    rclpy.init(args=[])

    class Bridge(Node):
        def __init__(self):
            super().__init__("isaac_astribot_bridge")
            self.sim_time = world.current_time
            self.episode = 0
            self.stopping = False
            self.counts = {"odom": 0, "rgb": 0, "depth": 0, "front": 0, "back": 0}
            self.max_model_alignment_error = 0.
            self.clock_pub = self.create_publisher(Clock, "/clock", 1)
            self.odom_pub = self.create_publisher(Odometry, "/odom_gt", 50)
            self.pose_pub = self.create_publisher(PoseStamped, "/isaac/usd_pose", 10)
            self.image_pub = self.create_publisher(Image, "/torso_rgbd/image_raw", qos_profile_sensor_data)
            # The original STag detector requests reliable CameraInfo (images
            # remain SensorData/best effort), so calibration must offer it.
            self.info_pub = self.create_publisher(CameraInfo, "/torso_rgbd/camera_info", 10)
            self.depth_pub = self.create_publisher(Image, "/torso_rgbd/depth/image_raw", qos_profile_sensor_data)
            self.cloud_pub = {side: self.create_publisher(PointCloud2, f"/lidar_{side}/points", qos_profile_sensor_data)
                              for side in ("front", "back")}
            self.status_pub = self.create_publisher(String, "/isaac/status", 1)
            static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE)
            self.static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
            self.create_subscription(Twist, "/cmd_vel", self.command, 10)
            self.create_service(Trigger, "/isaac/reset", self.reset_episode)
            self.create_service(Trigger, "/isaac/shutdown", self.shutdown_bridge)
            transforms = []
            for joint in ET.parse(model_dir / "astribot_fixed.urdf").findall("joint"):
                matrix = np.eye(4)
                origin = joint.find("origin")
                if origin is not None:
                    matrix[:3, :3] = rpy_matrix(*map(float, origin.get("rpy", "0 0 0").split()))
                    matrix[:3, 3] = list(map(float, origin.get("xyz", "0 0 0").split()))
                transforms.append(transform(joint.find("parent").get("link"), joint.find("child").get("link"), matrix))
            matrix = np.eye(4)
            matrix[:3, :3] = optical_rotation
            transforms.append(transform("torso_rgbd", "torso_rgbd_optical", matrix))
            self.static_pub.publish(TFMessage(transforms=transforms))
            self.info = CameraInfo()
            self.info.width, self.info.height = cam_spec["width"], cam_spec["height"]
            self.info.distortion_model, self.info.d = "plumb_bob", [0.]*5
            intrinsic = camera.get_intrinsics_matrix()
            self.info.k = intrinsic.flatten().astype(float).tolist()
            self.info.r = np.eye(3).flatten().tolist()
            projection = np.zeros((3, 4))
            projection[:, :3] = intrinsic
            self.info.p = projection.flatten().tolist()

        def command(self, msg):
            base.accept([msg.linear.x, msg.linear.y, msg.angular.z], self.sim_time, time.monotonic())

        def reset_episode(self, _request, response):
            base.reset()
            update_robot()
            self.episode += 1
            for side, seed in sensor_seeds.items():
                rngs[side] = np.random.default_rng(seed)
            response.success, response.message = True, f"episode={self.episode}; clock remains monotonic; reset noise separately"
            return response

        def shutdown_bridge(self, _request, response):
            self.stopping = True
            response.success, response.message = True, "shutdown requested"
            return response

        def publish_odom(self):
            # Read USD back independently, rather than publishing commanded pose.
            matrix = UsdGeom.XformCache().GetLocalToWorldTransform(root)
            position, quat = matrix.ExtractTranslation(), matrix.ExtractRotationQuat()
            model_sensor = stage.GetPrimAtPath('/World/Robot/Model/torso_rgbd')
            actual = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(model_sensor)).T
            expected = np.asarray(matrix).T @ frames['torso_rgbd']
            error = float(np.max(np.abs(actual-expected)))
            self.max_model_alignment_error = max(self.max_model_alignment_error, error)
            if error > 3e-5:
                raise RuntimeError(f'Imported robot no longer follows base transform: {error}')
            msg = Odometry()
            msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp(self.sim_time), "world", "base_link"
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, position)
            msg.pose.pose.orientation.w = float(quat.GetReal())
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z = map(float, quat.GetImaginary())
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = map(float, base.actual)
            self.odom_pub.publish(msg)
            pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
            self.pose_pub.publish(pose)
            self.counts["odom"] += 1

        def publish_camera(self):
            rgba, depth = camera.get_rgba(), camera.get_depth()
            if rgba is None or rgba.shape != (480, 640, 4) or depth is None or depth.shape != (480, 640):
                return
            frame_time = float(camera.get_current_frame().get("rendering_time", self.sim_time))
            header_time = stamp(frame_time if frame_time > 0 else self.sim_time)
            msg = Image()
            msg.header.stamp, msg.header.frame_id = header_time, "torso_rgbd_optical"
            msg.width, msg.height, msg.encoding, msg.step = 640, 480, "rgb8", 640*3
            # The ROS generated setter validates plain bytes one byte at a
            # time in Python. Its public typed-array fast path keeps the same
            # wire data without millions of redundant Python checks per frame.
            msg.data = array.array('B', rgba[:, :, :3].astype(np.uint8).tobytes())
            self.image_pub.publish(msg)
            self.info.header = msg.header
            self.info_pub.publish(self.info)
            msg = Image(header=msg.header, width=640, height=480, encoding="32FC1", step=640*4)
            msg.data = array.array('B', np.asarray(depth, dtype="<f4").tobytes())
            self.depth_pub.publish(msg)
            self.counts["rgb"] += 1
            self.counts["depth"] += 1

        def publish_lidar(self, side):
            spec = specs[side]
            distances = lidars.depths(side)
            if len(distances) != len(beams[side]):
                raise RuntimeError(f'Incomplete native lidar batch: {side} {len(distances)}')
            points = np.full((len(beams[side]), 4), np.nan, dtype="<f4")
            points[:, 3] = 0
            valid = np.isfinite(distances) & (distances >= spec['min']) & (distances < spec['max']-1e-5)
            distances[valid] += rngs[side].normal(0, spec['noise'], int(valid.sum()))
            valid &= (distances >= spec['min']) & (distances <= spec['max'])
            points[valid, :3] = beams[side][valid] * distances[valid, None]
            points[valid, 3] = 1
            msg = PointCloud2()
            msg.header.stamp, msg.header.frame_id = stamp(self.sim_time), "lidar_"+side
            msg.width, msg.height = int(spec["horizontal"][0]), int(spec["vertical"][0])
            msg.fields, msg.point_step, msg.row_step = fields, 16, msg.width*16
            msg.is_dense, msg.is_bigendian, msg.data = False, False, array.array('B', points.tobytes())
            self.cloud_pub[side].publish(msg)
            self.counts[side] += 1

    node = Bridge()
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, "stopping", True))
    signal.signal(signal.SIGINT, lambda *_: setattr(node, "stopping", True))
    started, start_sim = time.monotonic(), world.current_time
    timing = {name: 0. for name in ('spin', 'base', 'physics_render', 'odom', 'camera', 'lidar')}
    steps = 0
    due = {"odom": start_sim, "camera": start_sim, "lidar": start_sim, "status": start_sim}
    (out / "ready.json").write_text(json.dumps({"status": "ready", "specs": specs,
        "seeds": sensor_seeds,
        "scene": args.scene, "spawn": list(spawn),
        "world_validation": scene_manifest.get('usd_validation') if scene_manifest else None,
        "clock_policy": "monotonic across episode resets", "base_height": base_z,
        "lidar_backend": "Native PhysX Generic Sensor batches", "collision_boxes": boxes}, indent=2))
    print("AL_ASTRIBOT_BRIDGE_READY", flush=True)
    while app.is_running() and not node.stopping:
        wall = time.monotonic()
        if args.max_wall_seconds and wall-started > args.max_wall_seconds:
            break
        node.sim_time = world.current_time
        measured = time.perf_counter()
        rclpy.spin_once(node, timeout_sec=0)
        timing['spin'] += time.perf_counter()-measured
        measured = time.perf_counter()
        base.step(1/60, node.sim_time, wall)
        update_robot()
        lidars.refill()
        timing['base'] += time.perf_counter()-measured
        measured = time.perf_counter()
        world.step(render=True)
        timing['physics_render'] += time.perf_counter()-measured
        steps += 1
        node.sim_time = world.current_time
        node.clock_pub.publish(Clock(clock=stamp(node.sim_time)))
        for key, rate in (("odom", 50.), ("camera", cam_spec["hz"]), ("lidar", specs["front"]["hz"]), ("status", 2.)):
            if node.sim_time + 1e-8 < due[key]:
                continue
            due[key] += 1/rate
            measured = time.perf_counter()
            if key == "odom":
                node.publish_odom()
            elif key == "camera":
                node.publish_camera()
            elif key == "lidar":
                for side in ("front", "back"):
                    node.publish_lidar(side)
            else:
                node.status_pub.publish(String(data=json.dumps({"ready": True, "sim_time": node.sim_time,
                    "seeds": sensor_seeds,
                    "scene": args.scene, "spawn": list(spawn),
                    "world_validation": scene_manifest.get('usd_validation') if scene_manifest else None,
                    "episode": node.episode, "blocked": base.blocked, "pose": base.pose.tolist(), "counts": node.counts,
                    "rtf": (world.current_time-start_sim)/(time.monotonic()-started),
                    "timing_ms_per_step": {name: value*1000/steps for name, value in timing.items()},
                    "model_alignment_error": node.max_model_alignment_error})))
            if key in timing:
                timing[key] += time.perf_counter()-measured
        time.sleep(max(0., 1/60 - (time.monotonic()-wall)))
    result_path.write_text(json.dumps({"status": "finished", "wall_seconds": time.monotonic()-started,
        "seeds": sensor_seeds,
        "scene": args.scene, "spawn": list(spawn),
        "world_source_sha256": scene_manifest['source_sha256'] if scene_manifest else None,
        "sim_seconds": world.current_time-start_sim, "counts": node.counts,
        "timing_ms_per_step": {name: value*1000/max(steps, 1) for name, value in timing.items()},
        "model_alignment_error": node.max_model_alignment_error}, indent=2))
except BaseException as exc:
    print("AL_BRIDGE_FAILED " + traceback.format_exc(), flush=True)
    result_path.write_text(json.dumps({"status": "failed", "error": repr(exc)}, indent=2))
    raise
finally:
    if node is not None:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    app.close()
