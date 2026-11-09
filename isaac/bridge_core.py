"""Simulator-independent motion, collision and sensor calibration primitives."""
import math
from dataclasses import dataclass
import xml.etree.ElementTree as ET

import numpy as np


def yaw_matrix(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quaternion_xyzw(rotation):
    m = np.asarray(rotation)
    trace = np.trace(m)
    if trace > 0:
        s = 2 * math.sqrt(trace + 1)
        q = [(m[2, 1]-m[1, 2])/s, (m[0, 2]-m[2, 0])/s,
             (m[1, 0]-m[0, 1])/s, s/4]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i+1) % 3, (i+2) % 3
        s = 2 * math.sqrt(1 + m[i, i] - m[j, j] - m[k, k])
        q = [0.0]*4
        q[i], q[j], q[k], q[3] = s/4, (m[j, i]+m[i, j])/s, (m[k, i]+m[i, k])/s, (m[k, j]-m[j, k])/s
    return np.array(q) / np.linalg.norm(q)


def integrate(pose, twist, dt):
    """Exact SE(2) body-velocity integration (x forward, y left, yaw CCW)."""
    x, y, yaw = pose
    vx, vy, wz = twist
    angle = wz * dt
    if abs(wz) < 1e-9:
        dx, dy = vx*dt, vy*dt
    else:
        a, b = math.sin(angle)/wz, (1-math.cos(angle))/wz
        dx, dy = a*vx-b*vy, b*vx+a*vy
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x+c*dx-s*dy, y+s*dx+c*dy,
                     math.atan2(math.sin(yaw+angle), math.cos(yaw+angle))])


def intersects_box(pose, half_extent, box):
    """SAT: robot footprint vs the XY projection of an oriented 3-D box."""
    x, y, yaw = pose
    center, size = box["center"], box["size"]
    delta = np.array([x-center[0], y-center[1]])
    axes = yaw_matrix(yaw)[:2, :2]
    projected = np.asarray(box.get('rotation', np.eye(3)))[:2, :] * (np.asarray(size)/2)
    directions = [axes[:, 0], axes[:, 1]]
    for generator in projected.T:
        length = np.linalg.norm(generator)
        if length > 1e-9:
            directions.append(np.array([-generator[1], generator[0]])/length)
    for direction in directions:
        r1 = np.sum(np.abs(direction @ axes) * half_extent)
        r2 = np.sum(np.abs(direction @ projected))
        if abs(delta @ direction) > r1+r2+1e-9:
            return False
    return True


@dataclass
class PlanarBase:
    boxes: list
    spawn: tuple = (0., 0., 0.)
    watchdog_sim: float = 0.5
    watchdog_wall: float = 2.0

    def __post_init__(self):
        self.half_extent = np.array([0.325, 0.375])
        self.reset()

    def reset(self):
        self.pose = np.array(self.spawn, dtype=float)
        self.command = np.zeros(3)
        self.actual = np.zeros(3)
        self.last_sim = self.last_wall = -math.inf
        self.blocked = False

    def accept(self, command, sim_time, wall_time):
        value = np.asarray(command, dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            self.command[:] = 0
            return False
        value[:2] = np.clip(value[:2], -0.6, 0.6)
        value[2] = np.clip(value[2], -0.8, 0.8)
        self.command = value.copy()
        self.last_sim, self.last_wall = sim_time, wall_time
        return True

    def step(self, dt, sim_time, wall_time):
        twist = self.command.copy()
        if sim_time-self.last_sim > self.watchdog_sim or wall_time-self.last_wall > self.watchdog_wall:
            twist[:] = 0
        # Substeps prevent fast commands from tunnelling through thin obstacles.
        n = max(1, math.ceil(np.linalg.norm(twist[:2])*dt/0.01), math.ceil(abs(twist[2])*dt/0.01))
        self.blocked = False
        for _ in range(n):
            candidate = integrate(self.pose, twist, dt/n)
            if any(intersects_box(candidate, self.half_extent, box) for box in self.boxes):
                self.blocked = True
                break
            self.pose = candidate
        self.actual = np.zeros(3) if self.blocked else twist
        return self.pose


def sensor_specs(package):
    result = {}
    for side in ("front", "back"):
        sensor = ET.parse(package / f"urdf_astribot/lidar_{side}.xacro").find(".//sensor")
        ray = sensor.find("ray")
        scan = ray.find("scan")
        result[side] = {
            "hz": float(sensor.findtext("update_rate")),
            "horizontal": [float(scan.findtext("horizontal/"+k)) for k in ("samples", "min_angle", "max_angle")],
            "vertical": [float(scan.findtext("vertical/"+k)) for k in ("samples", "min_angle", "max_angle")],
            "min": float(ray.findtext("range/min")), "max": float(ray.findtext("range/max")),
            "noise": float(ray.findtext("noise/stddev")),
        }
    camera = ET.parse(package / "urdf_astribot/torso_rgbd.xacro").find(".//sensor")
    result["camera"] = {"hz": float(camera.findtext("update_rate")),
        "width": int(camera.findtext("camera/image/width")),
        "height": int(camera.findtext("camera/image/height")),
        "hfov": float(camera.findtext("camera/horizontal_fov")),
        "near": float(camera.findtext("camera/clip/near")), "far": float(camera.findtext("camera/clip/far"))}
    return result


def beam_directions(spec):
    h, v = spec["horizontal"], spec["vertical"]
    azimuth, elevation = np.meshgrid(np.linspace(h[1], h[2], int(h[0])), np.linspace(v[1], v[2], int(v[0])))
    return np.stack([np.cos(elevation)*np.cos(azimuth), np.cos(elevation)*np.sin(azimuth),
                     np.sin(elevation)], axis=-1).reshape(-1, 3)


def physx_generic_angles(spec):
    """PhysX Generic zenith is positive down; Gazebo elevation is positive up."""
    beams = beam_directions(spec)
    return np.stack([np.arctan2(beams[:, 1], beams[:, 0]), -np.arcsin(beams[:, 2])])


ROOM_BOXES = [
    {"name": "EastWall", "center": [4.1, 0, 1.5], "size": [0.2, 6.4, 3.]},
    {"name": "WestWall", "center": [-4.1, 0, 1.5], "size": [0.2, 6.4, 3.]},
    {"name": "NorthWall", "center": [0, 3.1, 1.5], "size": [8.4, 0.2, 3.]},
    {"name": "SouthWall", "center": [0, -3.1, 1.5], "size": [8.4, 0.2, 3.]},
]
