#!/usr/bin/env bash
# Builds and tests al_nav2_plugins in the running `navigation` container, in an OVERLAY
# workspace (~/dev_ws) so the baseline workspace (~/ros2_ws, mounted from ENG's checkout)
# is never touched. Run it in WSL (needs docker). Usage: tools/build_and_test.sh
set -euo pipefail
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
C="${NAV_CONTAINER:-navigation}"

docker exec "$C" mkdir -p /home/app/dev_ws/src
docker exec "$C" rm -rf /home/app/dev_ws/src/al_nav2_plugins
docker cp "$PKG_DIR" "$C":/home/app/dev_ws/src/al_nav2_plugins
# Low parallelism: experiments may be running on the same machine (LEAD-29).
docker exec "$C" bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  cd ~/dev_ws
  export MAKEFLAGS=-j2
  colcon build --packages-select al_nav2_plugins --parallel-workers 1 \
    --event-handlers console_direct+
  colcon test --packages-select al_nav2_plugins --parallel-workers 1 \
    --event-handlers console_direct+ || true
  colcon test-result --verbose
'
