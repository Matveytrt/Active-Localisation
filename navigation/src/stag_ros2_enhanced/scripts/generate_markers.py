#!/usr/bin/env python3
"""
Standalone Marker Generator

Generate STag markers (HC, HG, HO types) from HD marker sets.
Can be run without ROS2 environment.

Usage:
    python3 generate_markers.py --input-dir markers/HD17 --output-dir output/HC17 --input-set HD17 --output-set HC17
"""

import sys
import os

# Add package to path for standalone execution
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from stag_ros2_enhanced.marker_generator_cli import main

if __name__ == "__main__":
    main()
