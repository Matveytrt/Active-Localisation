#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# != 2 || ( $1 != mapping && $1 != localization ) || $2 != /workspace/maps/*.db ]]; then
  echo 'Usage: bash isaac/start_slam.sh mapping|localization /workspace/maps/UNIQUE.db' >&2
  exit 2
fi
export AL_ISAAC_SLAM_MODE=$1 AL_ISAAC_DB=$2
if [[ $1 == mapping ]] && docker exec al-isaac-navigation test -e "$2"; then
  echo 'Mapping database already exists; choose a new path' >&2
  exit 1
fi
docker compose -f isaac/compose.yaml --profile slam up -d --force-recreate fiducial slam
