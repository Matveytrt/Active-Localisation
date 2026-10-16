#!/usr/bin/env python3
"""
Standalone Batch PDF Generator

Automatically generate PDF files for multiple marker folders and sizes.
Can be run without ROS2 environment.

Usage:
    python3 pdf_batch_generation.py --folders markers/HD13 markers/HD17 --sizes 40 80 100
"""

import sys
import os

# Add package to path for standalone execution
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from stag_ros2_enhanced.batch_generator_cli import main

if __name__ == "__main__":
    main()
