#!/usr/bin/env python3
"""
Generate an SDF world for Gazebo using shelf_6 and STag_marker models
from the world_and_robot package.

The world layout follows the example of `worlds/my_world.world`:
- Shelves (`shelf_6`) are placed in two rows.
- Each shelf can have up to 6 marker levels at fixed heights.

Usage examples:
  - Reproduce my_world.world layout (6 shelves, 5 markers per level) to a new file:
      ./generate_gazebo_world.py --shelves 6 --marker-levels 5 \
          --output ../worlds/generated_world.world

  - 8 shelves (2 rows x 4 columns), 5 marker levels per shelf:
      ./generate_gazebo_world.py --shelves 8 --marker-levels 5
"""

import argparse
import math
from pathlib import Path
from typing import List


# Fixed heights for 6 shelf levels (taken from my_world.world)
LEVEL_HEIGHTS = [0.146, 0.506, 0.867, 1.227, 1.556, 1.886]


def load_marker_material_names(materials_path: Path) -> List[str]:
    """
    Parse markers.material and return a list of material names
    like 'STagMarker/marker_00219', 'STagMarker/marker_00261', ...
    """
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


def generate_world_xml(
    num_shelves: int,
    markers_per_level: int,
    marker_materials: List[str],
) -> str:
    """
    Generate SDF XML string.

    - num_shelves: total number of shelf_6 instances (0..6).
    - markers_per_level: how many markers are placed on each of the 6 levels
      of every shelf (0..5).
    - marker_materials: list of material names; reused cyclically if needed.
    """
    # Hard limits: up to 6 shelves and up to 5 markers per level
    markers_per_level = max(0, min(markers_per_level, 5))

    lines: List[str] = []
    lines.append('<?xml version="1.0"?>')
    lines.append('<sdf version="1.6">')
    lines.append('  <world name="default">')
    # Ground plane and sun, as in example world
    lines.append("    <include>")
    lines.append("      <uri>model://ground_plane</uri>")
    lines.append("    </include>")
    lines.append("    <include>")
    lines.append("      <uri>model://sun</uri>")
    lines.append("    </include>")

    # Layout parameters (taken from my_world.world)
    # Row Y positions:
    row_ys = [1.5, 4.5]
    # First column X and step:
    base_x = -1.5
    step_x = 3.0

    shelves_per_row = math.ceil(num_shelves / 2) if num_shelves > 0 else 0

    material_index = 0
    num_materials = len(marker_materials)

    for idx in range(num_shelves):
        row = idx // shelves_per_row  # 0 or 1
        col = idx % shelves_per_row
        if row >= 2:
            # If more than 2 rows would be needed, just wrap to second row
            row = 1

        shelf_name = f"shelf_6_r{row+1}_c{col+1}"
        x = base_x + step_x * col
        y = row_ys[row]
        z = 0.0

        # Shelf include (same as in my_world.world)
        lines.append("    <include>")
        lines.append(f"      <name>{shelf_name}</name>")
        lines.append("      <uri>model://shelf_6</uri>")
        lines.append(f"      <pose>{x} {y} {z} 0 0 0</pose>")
        lines.append("    </include>")

        # Markers in front of shelf on all 6 levels.
        # Base marker offsets relative to shelf pose (taken from example):
        # Marker X offset: shelf_x - 0.1
        # Marker Y offset: shelf_y - 0.6
        marker_base_x = x - 0.1
        marker_base_y = y - 0.6

        # On each of the 6 levels place `markers_per_level` markers.
        # Логика размещения:
        # - первый маркер на базовой точке (marker_base_x, marker_base_y)
        # - остальные смещены только влево по X и равномерно занимают 1.0 м
        for level_idx, z_level in enumerate(LEVEL_HEIGHTS):
            level = level_idx + 1
            for marker_idx in range(markers_per_level):
                # Шаг по X зависит от числа маркеров на уровне.
                # При ширине стеллажа ≈ 1 м распределяем маркеры равномерно:
                # step = 1.0 / (markers_per_level - 1), если маркеров > 1.
                if markers_per_level > 1:
                    step = 1.0 / float(markers_per_level - 1)
                    offset = step * marker_idx
                else:
                    offset = 0.0
                marker_x = marker_base_x - offset
                marker_y = marker_base_y

                model_name = (
                    f"stag_r{row+1}c{col+1}_l{level}_m{marker_idx+1}"
                )

                # Choose material; if none, just skip setting a script
                material_line_block: List[str] = []
                if num_materials > 0:
                    mat_name = marker_materials[material_index % num_materials]
                    material_index += 1
                    material_line_block.extend(
                        [
                            "          <material>",
                            "            <script>",
                            "              <uri>model://STag_marker/materials/scripts</uri>",
                            "              <uri>model://STag_marker/materials/textures</uri>",
                            f"              <name>{mat_name}</name>",
                            "            </script>",
                            "            <ambient>1 1 1 1</ambient>",
                            "            <diffuse>1 1 1 1</diffuse>",
                            "          </material>",
                        ]
                    )
                else:
                    material_line_block.extend(
                        [
                            "          <material>",
                            "            <ambient>1 1 1 1</ambient>",
                            "            <diffuse>1 1 1 1</diffuse>",
                            "          </material>",
                        ]
                    )

                lines.append(f"    <model name=\"{model_name}\">")
                lines.append("      <static>true</static>")
                # 1.5708 rad roll to put the marker upright, as in the example
                lines.append(
                    f"      <pose>{marker_x} {marker_y} {z_level} 1.5708 0 0</pose>"
                )
                lines.append("      <link name=\"marker_link\">")
                lines.append("        <collision name=\"collision\">")
                lines.append(
                    "          <geometry><box><size>0.1 0.1 0.01</size></box></geometry>"
                )
                lines.append("        </collision>")
                lines.append("        <visual name=\"visual\">")
                lines.append(
                    "          <geometry><box><size>0.1 0.1 0.01</size></box></geometry>"
                )
                lines.extend(material_line_block)
                lines.append("          <pose>0 0 0.001 0 0 0</pose>")
                lines.append("        </visual>")
                lines.append("      </link>")
                lines.append("    </model>")

    lines.append("  </world>")
    lines.append("</sdf>")
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a Gazebo SDF world with shelf_6 and STag markers "
            "for the world_and_robot package."
        )
    )
    parser.add_argument(
        "--shelves",
        type=int,
        default=6,
        help=(
            "Number of shelf_6 objects (0..6). "
            "Default: 6, as in my_world.world."
        ),
    )
    parser.add_argument(
        "--marker-levels",
        type=int,
        default=5,
        help=(
            "Number of STag markers placed on each of the 6 shelf levels (0..5). "
            "Например, 5 => 5 маркеров на КАЖДОМ из 6 уровней полок каждого shelf_6. "
            "Не больше 5 маркеров на уровень."
        ),
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help=(
            "Output world file path. If not set, prints SDF to stdout. "
            "Example: ../worlds/my_world.world"
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    # Clamp arguments to required limits: shelves <= 6, marker_levels <= 5
    num_shelves = max(0, min(args.shelves, 6))
    markers_per_level = max(0, min(args.marker_levels, 5))

    # Locate markers.material relative to this script
    script_path = Path(__file__).resolve()
    pkg_root = script_path.parents[1]  # .../src/world_and_robot
    materials_path = (
        pkg_root
        / "gazebo_objects"
        / "STag_marker"
        / "materials"
        / "scripts"
        / "markers.material"
    )

    marker_materials = load_marker_material_names(materials_path)

    xml = generate_world_xml(
        num_shelves=num_shelves,
        markers_per_level=markers_per_level,
        marker_materials=marker_materials,
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


