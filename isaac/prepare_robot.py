"""Bake the project's Gazebo robot pose into a simulator-independent fixed URDF.

The rotation convention matches world_and_robot/launch/rsp.launch.py.
Only generated files under isaac/runtime are changed.
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def bake_origin(joint, value):
    origin = joint.find("origin")
    if origin is None:
        origin = ET.SubElement(joint, "origin")
    if isinstance(value, dict):
        axis, angle = value["axis"], float(value["angle"])
    else:
        axis_element = joint.find("axis")
        axis = [float(v) for v in (axis_element.get("xyz", "1 0 0")
                                  if axis_element is not None else "1 0 0").split()]
        angle = float(value)
    x, y, z = np.array(axis) / np.linalg.norm(axis)
    c, s, t = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    rotation = np.array([[t*x*x+c, t*x*y-s*z, t*x*z+s*y],
                         [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
                         [t*x*z-s*y, t*y*z+s*x, t*z*z+c]])
    rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    m = rpy_matrix(*rpy) @ rotation
    pitch = math.atan2(-m[2, 0], math.hypot(m[0, 0], m[1, 0]))
    if abs(math.cos(pitch)) < 1e-9:
        roll, yaw = math.atan2(-m[1, 2], m[1, 1]), 0.0
    else:
        roll, yaw = math.atan2(m[2, 1], m[2, 2]), math.atan2(m[1, 0], m[0, 0])
    origin.set("rpy", " ".join(f"{v:.6f}" for v in (roll, pitch, yaw)))


def prepare(repo=None, output=None):
    repo = Path(repo or Path(__file__).resolve().parents[1])
    source = repo / "gazebo/src/world_and_robot"
    urdf = source / "urdf_astribot/astribot_whole_body_with_head.urdf"
    pose = source / "config/astribot_pose.yaml"
    root = ET.parse(urdf).getroot()
    root.set("name", "astribot")
    positions = yaml.safe_load(pose.read_text())["joint_positions"]
    for gazebo in root.findall("gazebo"):
        root.remove(gazebo)
    for joint in root.findall("joint"):
        if joint.get("type") != "fixed":
            if joint.get("name") in positions:
                bake_origin(joint, positions[joint.get("name")])
            joint.set("type", "fixed")
            for tag in ("axis", "limit", "dynamics", "safety_controller", "calibration", "mimic"):
                for child in joint.findall(tag):
                    joint.remove(child)
    meshes = set()
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        prefix = "package://world_and_robot/"
        if not filename.startswith(prefix):
            raise ValueError(f"Unexpected mesh URI: {filename}")
        relative = filename[len(prefix):]
        if not (source / relative).is_file():
            raise FileNotFoundError(source / relative)
        meshes.add(relative)
        mesh.set("filename", "/work/gazebo/src/world_and_robot/" + relative)
    transforms = {"base_link": np.eye(4)}
    remaining = list(root.findall("joint"))
    while remaining:
        previous_count = len(remaining)
        for joint in remaining[:]:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in transforms:
                continue
            transform = np.eye(4)
            origin = joint.find("origin")
            if origin is not None:
                transform[:3, :3] = rpy_matrix(*map(float, origin.get("rpy", "0 0 0").split()))
                transform[:3, 3] = list(map(float, origin.get("xyz", "0 0 0").split()))
            transforms[child] = transforms[parent] @ transform
            remaining.remove(joint)
        if len(remaining) == previous_count:
            raise ValueError("URDF contains disconnected links or a cycle")
    assert len(transforms) == len(root.findall("link"))
    output = Path(output or repo / "isaac/runtime/robot")
    output.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(output / "astribot_fixed.urdf", encoding="utf-8", xml_declaration=True)
    metadata = {"links": len(transforms), "joints": len(root.findall("joint")),
                "meshes": len(meshes), "pose": positions,
                "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
                "pose_sha256": hashlib.sha256(pose.read_bytes()).hexdigest(),
                "base_to_links": {k: v.tolist() for k, v in transforms.items()}}
    (output / "manifest.json").write_text(json.dumps(metadata, indent=2))
    return output / "astribot_fixed.urdf", metadata


if __name__ == "__main__":
    path, metadata = prepare()
    print(json.dumps({"urdf": str(path), "links": metadata["links"], "meshes": metadata["meshes"]}))
