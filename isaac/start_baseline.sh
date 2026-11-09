#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $(docker inspect -f '{{.State.Running}}' al-isaac-slam) != true ]]; then
  echo 'Start own localization before baseline Nav2' >&2
  exit 1
fi
if ! docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' al-isaac-slam | grep -qx 'AL_ISAAC_SLAM_MODE=localization'; then
  echo 'Baseline requires localization mode on an own DB copy' >&2
  exit 1
fi
docker compose -f isaac/compose.yaml --profile nav up -d --force-recreate baseline
