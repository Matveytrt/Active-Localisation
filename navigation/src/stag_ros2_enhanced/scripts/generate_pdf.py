#!/usr/bin/env python3
"""
Standalone PDF Generator

Generate printable PDF files with STag markers.
Can be run without ROS2 environment.

Usage:
    python3 generate_pdf.py --input markers/HD17 --marker-size 50
"""

import sys
import os

# Add package to path for standalone execution
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from stag_ros2_enhanced.pdf_generator_cli import main

if __name__ == "__main__":
    main()
