#!/usr/bin/env python3
"""
Generate a Gazebo SDF world with the same shelf layout as my_world.world.

Placement rules:
- 6 shelves (2 rows x 3 columns), same positions as generate_gazebo_world.py.
- HD17 markers: exactly 1 marker on one corner of each shelf (6 total).
- HD15 markers: exactly 3 markers on level 4 of each shelf, symmetric around
  shelf center along X with equal spacing (18 total).

The script writes a complete SDF world with inline marker models, so it stays
compatible with extract_markers.py (model names start with "stag_").
"""

import argparse
from pathlib import Path
from typing import List


LEVEL_HEIGHTS = [0.146, 0.506, 0.867, 1.227, 1.556, 1.886]


def load_marker_material_names(materials_path: Path) -> List[str]:
    names: List[str] = []
    if not materials_path.is_file():
        return names

    with materials_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line.startswith("material "):
                parts = line.split()
                if len(parts) >= 2:
                    names.append(parts[1])
    return names


def append_marker_model(
    lines: List[str],
    *,
    model_name: str,
    x: float,
    y: float,
    z: float,
    size_xyz: str,
    script_uri_model: str,
    marker_material_name: str,
) -> None:
    lines.append(f'    <model name="{model_name}">')
    lines.append("      <static>true</static>")
    lines.append(f"      <pose>{x:.6f} {y:.6f} {z:.6f} 1.5708 0 0</pose>")
    lines.append('      <link name="marker_link">')
    lines.append('        <collision name="collision">')
    lines.append(
        f"          <geometry><box><size>{size_xyz}</size></box></geometry>"
    )
    lines.append("        </collision>")
    lines.append('        <visual name="visual">')
    lines.append(
        f"          <geometry><box><size>{size_xyz}</size></box></geometry>"
    )
    lines.append("          <material>")
    lines.append("            <script>")
    lines.append(
        f"              <uri>model://{script_uri_model}/materials/scripts</uri>"
    )
    lines.append(
        f"              <uri>model://{script_uri_model}/materials/textures</uri>"
    )
    lines.append(f"              <name>{marker_material_name}</name>")
    lines.append("            </script>")
    lines.append("            <ambient>1 1 1 1</ambient>")
    lines.append("            <diffuse>1 1 1 1</diffuse>")
    lines.append("          </material>")
    lines.append("          <pose>0 0 0.001 0 0 0</pose>")
    lines.append("        </visual>")
    lines.append("      </link>")
    lines.append("    </model>")


def generate_world_xml(hd15_materials: List[str], hd17_materials: List[str]) -> str:
    if len(hd15_materials) < 18:
        raise ValueError(
            f"Need at least 18 HD15 materials, found {len(hd15_materials)}."
        )
    if len(hd17_materials) < 6:
        raise ValueError(
            f"Need at least 6 HD17 materials, found {len(hd17_materials)}."
        )

    # Shelf layout with requested spacing:
    # - X step: 1.7 m
    # - Y step: 2.5 m
    shelf_positions = [
        (-1.5, 1.5),
        (0.2, 1.5),
        (1.9, 1.5),
        (-1.5, 4.0),
        (0.2, 4.0),
        (1.9, 4.0),
    ]

    # Marker placement offsets relative to shelf pose.
    front_y_offset = -0.6
    hd17_corner_x_offset = -0.5  # one selected shelf corner (front-left)
    hd17_x_shift = 0.5
    hd17_z = 0.5 * (LEVEL_HEIGHTS[3] + LEVEL_HEIGHTS[4])  # between 4th and 5th
    hd15_level4_z = LEVEL_HEIGHTS[3]
    # Shift the HD15 triplet by -0.9 m along X relative to each shelf while
    # keeping equal spacing (symmetry around the shifted local center).
    hd15_center_x_offset = -0.65
    hd15_spacing = 0.5
    hd15_x_offsets = (
        hd15_center_x_offset - hd15_spacing,
        hd15_center_x_offset,
        hd15_center_x_offset + hd15_spacing,
    )

    lines: List[str] = []
    lines.append('<?xml version="1.0"?>')
    lines.append('<sdf version="1.6">')
    lines.append('  <world name="default">')
    lines.append("    <include>")
    lines.append("      <uri>model://ground_plane</uri>")
    lines.append("    </include>")
    lines.append("    <include>")
    lines.append("      <uri>model://sun</uri>")
    lines.append("    </include>")

    hd17_idx = 0
    hd15_idx = 0

    for shelf_idx, (sx, sy) in enumerate(shelf_positions, start=1):
        row = 1 if shelf_idx <= 3 else 2
        col = shelf_idx if row == 1 else shelf_idx - 3
        shelf_name = f"shelf_6_r{row}_c{col}"

        lines.append("    <include>")
        lines.append(f"      <name>{shelf_name}</name>")
        lines.append("      <uri>model://shelf_6</uri>")
        lines.append(f"      <pose>{sx} {sy} 0 0 0 0</pose>")
        lines.append("    </include>")

        # 1 x HD17 marker for this shelf.
        append_marker_model(
            lines,
            model_name=f"stag_hd17_s{shelf_idx}",
            x=sx + hd17_corner_x_offset + hd17_x_shift,
            y=sy + front_y_offset,
            z=hd17_z,
            size_xyz="0.1 0.1 0.01",
            script_uri_model="STag_marker_HD17",
            marker_material_name=hd17_materials[hd17_idx],
        )
        hd17_idx += 1

        # 3 x HD15 markers on level 4, equal distance from shelf center.
        for local_idx, x_offset in enumerate(hd15_x_offsets, start=1):
            append_marker_model(
                lines,
                model_name=f"stag_hd15_s{shelf_idx}_m{local_idx}",
                x=sx + x_offset,
                y=sy + front_y_offset,
                z=hd15_level4_z,
                size_xyz="0.08 0.08 0.01",
                script_uri_model="STag_marker_HD15",
                marker_material_name=hd15_materials[hd15_idx],
            )
            hd15_idx += 1

    lines.append("  </world>")
    lines.append("</sdf>")
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate Gazebo world: shelves + 6 HD17 + 18 HD15 markers."
        )
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help=(
            "Output world file path. If omitted, prints XML to stdout. "
            "Example: ../worlds/generated_world_hd17_hd15.world"
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    script_path = Path(__file__).resolve()
    pkg_root = script_path.parents[1]  # .../src/world_and_robot

    hd15_materials_path = (
        pkg_root
        / "gazebo_objects"
        / "STag_marker_HD15"
        / "materials"
        / "scripts"
        / "markers.material"
    )
    hd17_materials_path = (
        pkg_root
        / "gazebo_objects"
        / "STag_marker_HD17"
        / "materials"
        / "scripts"
        / "markers.material"
    )

    hd15_materials = load_marker_material_names(hd15_materials_path)
    hd17_materials = load_marker_material_names(hd17_materials_path)

    xml = generate_world_xml(
        hd15_materials=hd15_materials,
        hd17_materials=hd17_materials,
    )

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(xml, encoding="utf-8")
        print(f"World saved to: {out_path}")
    else:
        print(xml)


if __name__ == "__main__":
    main()
