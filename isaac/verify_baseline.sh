#!/usr/bin/env bash
set -euo pipefail
docker exec al-isaac-navigation bash -lc \
  'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && python3 /repo/isaac/verify_baseline.py'
