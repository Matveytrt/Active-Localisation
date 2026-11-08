#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$(pwd)
mkdir -p isaac/runtime/robot isaac/runtime/cache/ov isaac/runtime/cache/gl isaac/runtime/logs
docker run -d --name al-isaac-robot-import --user 0:0 --runtime nvidia --gpus all --network host --shm-size 2g \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ALLOW_ROOT=1 \
  -v "$ROOT:/work" -w /work \
  -v "$ROOT/isaac/runtime/cache/ov:/root/.cache/ov" \
  -v "$ROOT/isaac/runtime/cache/gl:/root/.cache/nvidia/GLCache" \
  -v "$ROOT/isaac/runtime/logs:/root/.nvidia-omniverse/logs" \
  --entrypoint /isaac-sim/python.sh \
  nvcr.io/nvidia/isaac-sim@sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54 \
  /work/isaac/import_robot.py --allow-root
