#!/usr/bin/env python3
"""
Export big-marker poses from an RTAB-Map .db into landmarks.yaml (frame_id: map).

Source of poses, in order of preference:
  1. Optimized graph saved by RTAB-Map on shutdown after mapping
     (Admin.opt_ids / Admin.opt_poses). Landmarks have negative ids there.
  2. Fallback: landmark links (Link.type == 8) composed with the raw node
     pose (Node.pose, odometry frame): T_map_landmark = T_node * T_node_landmark.
     For each landmark the link with the largest from_id (newest node) is used.

Usage:
  ros2 run rtabmap_landmarks export_landmarks_from_db \
      --db /home/app/ros2_ws/maps/astribot_sim.db \
      --out /home/app/ros2_ws/src/rtabmap_landmarks/config/landmarks.yaml
"""

import argparse
import math
import os
import sqlite3
import struct
import zlib
from datetime import datetime

import yaml


DEFAULT_DB = "/home/app/ros2_ws/maps/astribot_sim.db"
DEFAULT_OUT = "/home/app/ros2_ws/src/rtabmap_landmarks/config/landmarks.yaml"
LINK_TYPE_LANDMARK = 8


def read_mat34(values):
    """3x4 row-major float transform (12 values) to a 4x4 matrix (list of lists)."""
    return [
        [values[0], values[1], values[2], values[3]],
        [values[4], values[5], values[6], values[7]],
        [values[8], values[9], values[10], values[11]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matmul44(a, b):
    """Multiply two 4x4 matrices."""
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rot_to_quat_xyzw(r):
    """Convert 3x3 rotation matrix to quaternion (x, y, z, w)."""
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2][1] - r[1][2]) / s
        y = (r[0][2] - r[2][0]) / s
        z = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        w = (r[2][1] - r[1][2]) / s
        x = 0.25 * s
        y = (r[0][1] + r[1][0]) / s
        z = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        w = (r[0][2] - r[2][0]) / s
        x = (r[0][1] + r[1][0]) / s
        y = 0.25 * s
        z = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        w = (r[1][0] - r[0][1]) / s
        x = (r[0][2] + r[2][0]) / s
        y = (r[1][2] + r[2][1]) / s
        z = 0.25 * s
    return x, y, z, w


def decompress_mat(blob):
    """Decode an RTAB-Map compressData2() blob: zlib payload + rows, cols, type (int32)."""
    return zlib.decompress(bytes(blob[:-12]))


def optimized_landmark_poses(cur):
    """{marker_id: 4x4} from Admin.opt_ids/opt_poses, or {} if the graph was not saved."""
    row = cur.execute("SELECT opt_ids, opt_poses FROM Admin").fetchone()
    if not row or row[0] is None or row[1] is None:
        return {}
    ids_raw = decompress_mat(row[0])
    poses_raw = decompress_mat(row[1])
    ids = struct.unpack(f"<{len(ids_raw) // 4}i", ids_raw)
    floats = struct.unpack(f"<{len(poses_raw) // 4}f", poses_raw)
    if len(floats) != 12 * len(ids):
        raise ValueError(
            f"Admin.opt_poses has {len(floats)} values for {len(ids)} ids (expected 12 per id)"
        )
    # RTAB-Map stores landmark ids as negative graph ids (marker 3 -> -3).
    return {
        -node_id: read_mat34(floats[12 * k:12 * (k + 1)])
        for k, node_id in enumerate(ids)
        if node_id < 0
    }


def link_landmark_poses(cur):
    """{marker_id: 4x4} from landmark links and raw node poses (odometry frame)."""
    best_by_to_id = {}
    for from_id, to_id, transform_blob in cur.execute(
        "SELECT from_id, to_id, transform FROM Link WHERE type=?", (LINK_TYPE_LANDMARK,)
    ):
        if transform_blob is None:
            continue
        prev = best_by_to_id.get(int(to_id))
        if prev is None or int(from_id) > prev[0]:
            best_by_to_id[int(to_id)] = (int(from_id), transform_blob)

    poses = {}
    for to_id, (from_id, transform_blob) in best_by_to_id.items():
        node_row = cur.execute("SELECT pose FROM Node WHERE id=?", (from_id,)).fetchone()
        if not node_row or node_row[0] is None:
            continue
        t_node = read_mat34(struct.unpack("<12f", node_row[0]))
        t_node_landmark = read_mat34(struct.unpack("<12f", transform_blob))
        poses[abs(to_id)] = matmul44(t_node, t_node_landmark)
    return poses


def build_big_markers(poses):
    big_markers = []
    for order, marker_id in enumerate(sorted(poses), start=1):
        t = poses[marker_id]
        qx, qy, qz, qw = rot_to_quat_xyzw([row[:3] for row in t[:3]])
        big_markers.append({
            "id": int(marker_id),
            "order": order,
            "position": {"x": float(t[0][3]), "y": float(t[1][3]), "z": float(t[2][3])},
            "orientation": {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)},
        })
    return big_markers


def export_landmarks(db_path, output_yaml_path, family):
    if not os.path.isfile(db_path):
        raise FileNotFoundError(f"DB file not found: {db_path}")

    con = sqlite3.connect(db_path)
    try:
        cur = con.cursor()
        poses = optimized_landmark_poses(cur)
        source = "optimized_graph"
        if not poses:
            print(
                "WARNING: no optimized graph in Admin (RTAB-Map was not shut down cleanly "
                "after mapping?) — falling back to landmark links over raw odometry poses."
            )
            poses = link_landmark_poses(cur)
            source = "odometry_links"
    finally:
        con.close()

    data = {
        "frame_id": "map",
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "source_db": db_path,
        "source": source,
        "big_marker_family": family,
        "big_markers": build_big_markers(poses),
    }

    out_dir = os.path.dirname(output_yaml_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(output_yaml_path, "w", encoding="utf-8") as f:
        f.write("# Сгенерировано export_landmarks_from_db — позы больших маркеров из карты RTAB-Map.\n")
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)

    ids = ", ".join(str(m["id"]) for m in data["big_markers"]) or "none"
    print(f"Exported {len(data['big_markers'])} landmark(s) [{ids}] from {source} to: {output_yaml_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--db", default=DEFAULT_DB, help=f"RTAB-Map database (default: {DEFAULT_DB})")
    parser.add_argument("--out", default=DEFAULT_OUT, help=f"Output YAML (default: {DEFAULT_OUT})")
    parser.add_argument("--family", default="HD17", help="STag family of the big markers (default: HD17)")
    args = parser.parse_args()
    export_landmarks(args.db, args.out, args.family)


if __name__ == "__main__":
    main()
