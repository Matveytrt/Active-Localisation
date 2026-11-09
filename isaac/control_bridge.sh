#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 1 || ( $1 != reset && $1 != shutdown ) ]]; then
  echo 'Usage: bash isaac/control_bridge.sh reset|shutdown' >&2
  exit 2
fi
docker exec al-isaac-navigation bash -lc \
  'source /opt/ros/humble/setup.bash && python3 /repo/isaac/bridge_control.py "$1"' -- "$1"
