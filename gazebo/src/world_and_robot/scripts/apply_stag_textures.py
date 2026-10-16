#!/usr/bin/env python3
"""
Generate markers.material file from PNG textures in the textures directory.

Usage (from src/world_and_robot):
  python3 scripts/apply_stag_textures.py \
      --textures gazebo_objects/STag_marker/materials/textures \
      --materials gazebo_objects/STag_marker/materials/scripts/markers.material

How it works:
- Scans all *.png files in the textures directory (sorted by name).
- Generates markers.material with one material per PNG.
- Material name format: STagMarker/marker_<basename>

The generate_gazebo_world.py script reads this markers.material 
to assign materials to markers in the generated world.
"""

import argparse
import pathlib
import shutil
import sys


def build_materials(entries, material_namespace):
    """Generate Ogre material definitions for each PNG texture."""
    lines = []
    for marker_id, texture_name in entries:
        mat_name = f"{material_namespace}/marker_{marker_id}"
        lines.append(f"material {mat_name} {{\n")
        lines.append("  technique { pass {\n")
        lines.append("    ambient 1 1 1 1\n")
        lines.append("    diffuse 1 1 1 1\n")
        lines.append("    specular 0 0 0 1\n")
        lines.append("    lighting off\n")
        lines.append(f"    texture_unit {{ texture {texture_name} }}\n")
        lines.append("  } }\n")
        lines.append("}\n")
    return "".join(lines)


def main():
    ap = argparse.ArgumentParser(
        description="Generate markers.material from PNG textures"
    )
    ap.add_argument(
        "--textures",
        required=True,
        type=pathlib.Path,
        help="Directory with PNG textures"
    )
    ap.add_argument(
        "--materials",
        required=True,
        type=pathlib.Path,
        help="Output markers.material path"
    )
    ap.add_argument(
        "--namespace",
        default="STagMarker",
        help=(
            "Material namespace/prefix, e.g. STagMarkerHD15 or STagMarkerHD17. "
            "Default: STagMarker"
        ),
    )
    ap.add_argument(
        "--texture-prefix",
        default="",
        help=(
            "Optional prefix for texture file names used in materials. "
            "When set, prefixed copies are created in --textures to avoid "
            "name collisions between models (e.g. hd15_ or hd17_)."
        ),
    )
    args = ap.parse_args()

    # Find all PNG files
    pngs = sorted(args.textures.glob("*.png"))
    if not pngs:
        sys.exit("No PNG files found in textures directory.")

    # If prefix is set, use non-prefixed files as source and create prefixed
    # copies for deterministic, collision-free texture names.
    if args.texture_prefix:
        source_pngs = [
            png for png in pngs if not png.name.startswith(args.texture_prefix)
        ]
        if not source_pngs:
            source_pngs = pngs
        entries = []
        for png in source_pngs:
            prefixed_name = f"{args.texture_prefix}{png.name}"
            prefixed_path = args.textures / prefixed_name
            if png != prefixed_path:
                shutil.copy2(png, prefixed_path)
            entries.append((png.stem, prefixed_name))
    else:
        entries = [(png.stem, png.name) for png in pngs]

    # Generate and write materials
    materials_content = build_materials(entries, args.namespace)
    args.materials.parent.mkdir(parents=True, exist_ok=True)
    args.materials.write_text(materials_content, encoding="utf-8")
    
    print(f"[OK] Generated {len(entries)} materials -> {args.materials}")
    print("Materials:")
    for marker_id, texture_name in entries:
        print(f"  - {args.namespace}/marker_{marker_id} -> {texture_name}")


if __name__ == "__main__":
    main()
