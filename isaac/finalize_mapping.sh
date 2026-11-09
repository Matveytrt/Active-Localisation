#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# != 2 || $1 != /workspace/maps/*.db || $2 != /workspace/maps/*.db || $1 == "$2" ]]; then
  echo 'Usage: bash isaac/finalize_mapping.sh /workspace/maps/ONLINE.db /workspace/maps/GLOBAL.db' >&2
  exit 2
fi
for container in al-isaac-astribot al-isaac-slam; do
  if [[ $(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true) == true ]]; then
    echo 'Stop the own simulator and SLAM before offline consolidation' >&2
    exit 1
  fi
done
if docker exec al-isaac-navigation test -e "$2"; then
  echo 'Destination already exists; choose a new own map path' >&2
  exit 1
fi
docker exec al-isaac-navigation python3 -c \
  'import shutil,sys;shutil.copy2(sys.argv[1],sys.argv[2])' "$1" "$2"
docker exec al-isaac-navigation bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  cmake -S /repo/isaac/tools -B /workspace/finalize-build -DCMAKE_BUILD_TYPE=Release
  cmake --build /workspace/finalize-build -j2
  /workspace/finalize-build/finalize_map "$1"
  ros2 run rtabmap_landmarks export_landmarks_from_db --db "$1" --out "${1%.db}.landmarks.yaml"
' -- "$2"
