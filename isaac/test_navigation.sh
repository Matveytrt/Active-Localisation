#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
docker compose -f isaac/compose.yaml exec -T navigation bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  colcon test --base-paths /repo/navigation/src /repo/gazebo/src/lidar_stitching \
    --packages-select al_nav2_plugins --ctest-args -R ^test_ --return-code-on-test-failure
  colcon test-result --verbose
  python3 -m pytest -p no:cacheprovider \
    /repo/navigation/src/active_localization/test \
    /repo/navigation/src/al_odom_noise/test \
    /repo/navigation/src/al_eval/test /repo/isaac/test_robot_pose.py \
    /repo/isaac/test_bridge_core.py /repo/isaac/test_world_source.py -q
'
