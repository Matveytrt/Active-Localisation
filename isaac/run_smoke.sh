#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$(pwd)
IMAGE=nvcr.io/nvidia/isaac-sim@sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54
mkdir -p isaac/runtime/cache/ov isaac/runtime/cache/gl isaac/runtime/logs
docker run -d --name al-isaac-runtime-smoke --user 0:0 --gpus all --network host --shm-size 2g \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ALLOW_ROOT=1 \
  -e ROS_DISTRO=humble -e ROS_DOMAIN_ID=71 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib \
  -v "$ROOT:/work" -w /work \
  -v "$ROOT/isaac/runtime/cache/ov:/root/.cache/ov" \
  -v "$ROOT/isaac/runtime/cache/gl:/root/.cache/nvidia/GLCache" \
  -v "$ROOT/isaac/runtime/logs:/root/.nvidia-omniverse/logs" \
  --entrypoint /isaac-sim/python.sh "$IMAGE" /work/isaac/smoke_runtime.py --allow-root
