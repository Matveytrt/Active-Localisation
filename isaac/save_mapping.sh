#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# != 1 || $1 != /workspace/maps/*.db ]]; then
  echo 'Usage: bash isaac/save_mapping.sh /workspace/maps/OWN.db' >&2
  exit 2
fi
docker exec al-isaac-navigation test -f "$1"
docker exec al-isaac-navigation bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  ros2 run nav2_map_server map_saver_cli -f "${1%.db}" --ros-args \
    -p use_sim_time:=true -p save_map_timeout:=20.0
' -- "$1"
docker compose -f isaac/compose.yaml --profile slam stop -t 120 slam fiducial
docker exec al-isaac-navigation bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  ros2 run rtabmap_landmarks export_landmarks_from_db --db "$1" \
    --out "${1%.db}.landmarks.yaml" --family HD17
' -- "$1"
