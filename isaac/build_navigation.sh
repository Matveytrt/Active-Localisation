#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p isaac/runtime/navigation isaac/runtime/evidence
docker compose -f isaac/compose.yaml build navigation
docker compose -f isaac/compose.yaml up -d navigation
docker compose -f isaac/compose.yaml exec -T navigation bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  colcon build --base-paths /repo/navigation/src /repo/gazebo/src/lidar_stitching \
    --executor sequential --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
  source /workspace/install/setup.bash
  ros2 pkg prefix al_nav2_plugins
  ros2 pkg prefix al_eval
  ros2 pkg prefix al_odom_noise
  ros2 pkg prefix active_localization
  ros2 pkg prefix lidar_stitching
'
