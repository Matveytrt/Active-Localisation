#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$(pwd)
bridge_args=("$@")
AL_ISAAC_SEED=${AL_ISAAC_SEED:-42}
for ((i=0; i<${#bridge_args[@]}; i++)); do
  if [[ ${bridge_args[i]} == --seed ]]; then
    AL_ISAAC_SEED=${bridge_args[i+1]:-invalid}
  elif [[ ${bridge_args[i]} == --seed=* ]]; then
    AL_ISAAC_SEED=${bridge_args[i]#--seed=}
  fi
done
[[ $AL_ISAAC_SEED =~ ^[0-9]+$ && ${#AL_ISAAC_SEED} -le 10 ]] || { echo 'Invalid seed' >&2; exit 2; }
AL_ISAAC_SEED=$((10#$AL_ISAAC_SEED))
((AL_ISAAC_SEED < 4294967295)) || { echo 'Invalid seed' >&2; exit 2; }
export AL_ISAAC_SEED
mkdir -p isaac/runtime/bridge isaac/runtime/evidence isaac/runtime/cache/ov isaac/runtime/cache/gl
if docker container inspect al-isaac-astribot >/dev/null 2>&1; then
  if [[ $(docker inspect -f '{{.State.Running}}' al-isaac-astribot) == true ]]; then
    echo 'al-isaac-astribot is already running' >&2
    exit 1
  fi
  docker logs al-isaac-astribot > isaac/runtime/evidence/bridge-previous.log 2>&1
  docker rm al-isaac-astribot
fi
rm -f isaac/runtime/bridge/ready.json
docker compose -f isaac/compose.yaml up -d navigation
docker compose -f isaac/compose.yaml up -d --force-recreate odom-noise lidar-stitching
if [[ " $* " == *own-world* ]]; then
  docker compose -f isaac/compose.yaml --profile world up -d --force-recreate stag-hd15 stag-hd17
else
  docker compose -f isaac/compose.yaml --profile world stop stag-hd15 stag-hd17
fi
docker run -d --name al-isaac-astribot --user 0:0 --runtime nvidia --gpus all \
  --network host --shm-size 2g -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ALLOW_ROOT=1 \
  -e ROS_DISTRO=humble -e ROS_DOMAIN_ID=71 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/work/isaac/fastdds.xml \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib \
  -v "$ROOT:/work" -w /work \
  -v "$ROOT/isaac/runtime/cache/ov:/root/.cache/ov" \
  -v "$ROOT/isaac/runtime/cache/gl:/root/.cache/nvidia/GLCache" \
  --entrypoint /isaac-sim/python.sh \
  nvcr.io/nvidia/isaac-sim@sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54 \
  /work/isaac/astribot_bridge.py --allow-root --seed "$AL_ISAAC_SEED" "$@"
