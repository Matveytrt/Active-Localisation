"""Ensure Isaac's baked joints match the original Gazebo pose implementation."""
import ast
import math
import subprocess
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from prepare_robot import prepare


def test_fixed_pose_matches_gazebo(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    package = repo / "gazebo/src/world_and_robot"
    source = package / "launch/rsp.launch.py"
    # Load only pure geometry helpers; ROS launch imports are not needed by this test.
    helpers = [n for n in ast.parse(source.read_text()).body
               if isinstance(n, ast.FunctionDef) and n.name.startswith("_")]
    namespace = {"math": math, "subprocess": subprocess, "ET": ET, "yaml": yaml}
    exec(compile(ast.Module(body=helpers, type_ignores=[]), str(source), "exec"), namespace)
    reference = ET.fromstring(namespace["_load_robot_description_with_all_joints_fixed"](
        str(package / "urdf_astribot/astribot_whole_body_with_head.urdf"),
        str(package / "config/astribot_pose.yaml")))
    path, metadata = prepare(repo, tmp_path)
    actual = ET.parse(path).getroot()
    expected_joints = {j.get("name"): j for j in reference.findall("joint")}
    assert metadata["links"] == len(reference.findall("link"))
    assert metadata["meshes"] == len({m.get("filename") for m in reference.iter("mesh")})
    for joint in actual.findall("joint"):
        expected = expected_joints[joint.get("name")]
        assert joint.get("type") == expected.get("type") == "fixed"
        for attribute in ("xyz", "rpy"):
            a, b = joint.find("origin"), expected.find("origin")
            va = a.get(attribute, "0 0 0") if a is not None else "0 0 0"
            vb = b.get(attribute, "0 0 0") if b is not None else "0 0 0"
            assert np.allclose(list(map(float, va.split())), list(map(float, vb.split())), atol=1e-6)
    for frame in ("torso_rgbd", "lidar_front", "lidar_back"):
        transform = np.array(metadata["base_to_links"][frame])
        assert np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3))
    assert not actual.findall("gazebo")
