#!/usr/bin/env bash
# Quick live check of al_nav2_plugins with Nav2 up (T-009e). Prints PASS/FAIL per check
# and a final PASS/FAIL line; exit code 0 = no FAIL.
#
#   tools/check_live.sh [--expect active|control|baseline] [--goal x,y[,yaw]]
#                       [--scenario FILE] [--cell x,y,min,max]... [--step S]
#
# Works inside the `navigation` container and from the host (then it copies check_live.py
# into the container and runs itself there via `docker exec`; NAV_CONTAINER overrides the name).
#
# What it checks (node names: /planner_server, /global_costmap/global_costmap):
#   1. plugin classes loaded: GridBased.plugin, GridBased.inner_plugin, loc_layer in the
#      costmap `plugins`, loc_layer.plugin; active/control vs baseline as --expect says
#   2. loc_layer / GridBased parameters (scorer, c_max, marker ids) and spec v2 values
#      (k_a 20, r_min 0.66, k_r 40, k_yaw_rate 2.0): a stale params file gives WARN
#   3. global costmap: layer cost statistics, cost at 2 default "marker seen from 1.88 m" cells
#      and at every --cell given
#   4. one ComputePathToPose sample (--goal or --scenario): last pose = goal, pose spacing,
#      start yaw pinned, yaw != path tangent (active) / yaw ~ tangent (control)
# Modes: active = al_nav2_plugins with scorer marker_soft_count; control = scorer "null" and
# loc_layer disabled; baseline = plain Theta*. Without --goal/--scenario the plan sample is skipped.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" 2>/dev/null && pwd || true)"
C="${NAV_CONTAINER:-navigation}"
PLANNER_NODE="${PLANNER_NODE:-/planner_server}"
COSTMAP_NODE="${COSTMAP_NODE:-/global_costmap/global_costmap}"
PLANNER_ID="${PLANNER_ID:-GridBased}"
LAYER="${LAYER:-loc_layer}"

if [[ ${1:-} == -h || ${1:-} == --help ]]; then sed -n '2,24p' "${BASH_SOURCE[0]}"; exit 0; fi

# --- ROS environment (a plain `docker exec bash` has none of the ~/.bashrc lines)
set +u
[[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash"
[[ -f "$HOME/dev_ws/install/setup.bash" ]] && source "$HOME/dev_ws/install/setup.bash"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  if command -v docker >/dev/null 2>&1 && [[ -f "$HERE/check_live.py" ]]; then
    docker cp "$HERE/check_live.py" "$C":/tmp/al_check_live.py >/dev/null || exit 2
    AL_CHECK_PY=/tmp/al_check_live.py exec docker exec -i -e AL_CHECK_PY=/tmp/al_check_live.py \
      "$C" bash -s -- "$@" < "$HERE/check_live.sh"
  fi
  echo "FAIL ros2 not found and no docker; run this inside the navigation container" >&2
  exit 2
fi
PY="${AL_CHECK_PY:-$HERE/check_live.py}"
[[ -f "$PY" ]] || { echo "FAIL check_live.py not found at $PY" >&2; exit 2; }

EXPECT=active
ARGS=("$@")
for ((i = 0; i < ${#ARGS[@]}; i++)); do
  [[ ${ARGS[$i]} == --expect ]] && EXPECT="${ARGS[$((i + 1))]:-active}"
done
[[ $EXPECT == active || $EXPECT == control || $EXPECT == baseline ]] \
  || { echo "FAIL --expect must be active, control or baseline" >&2; exit 2; }

FAILS=0
WARNS=0
pass() { echo "PASS $*"; }
info() { echo "INFO $*"; }
fail() { echo "FAIL $*"; FAILS=$((FAILS + 1)); }
warn() { echo "WARN $*"; WARNS=$((WARNS + 1)); }

# pget NODE PARAM -> prints the value ("String value is: x" -> x); non-zero if unavailable.
pget() {
  local out
  out=$(timeout 15 ros2 param get "$1" "$2" 2>&1 </dev/null) || return 1
  [[ $out == *"is: "* || $out == *"are: "* ]] || return 1
  printf '%s\n' "$out" | head -n 1 | sed -E 's/^[^:]*(is|are): //'
}
# parr NODE PARAM -> "1,2,3" from "Integer values are: array('q', [1, 2, 3])" / "['a', 'b']".
parr() { pget "$1" "$2" | sed -E 's/.*\[(.*)\].*/\1/' | tr -d " '"; }
# num_eq VALUE EXPECTED TOL
num_eq() { awk -v a="$1" -v b="$2" -v t="$3" 'BEGIN{d=a-b; if(d<0)d=-d; exit !(d<=t)}'; }

echo "== al_nav2_plugins live check (expect: $EXPECT)"
for n in "$PLANNER_NODE" "$COSTMAP_NODE"; do
  if ros2 node list 2>/dev/null </dev/null | grep -qx "$n"; then pass "node $n is up"
  else fail "node $n not found (is Nav2 running? ros2 node list)"; fi
done

# --- 1. plugin classes
PLUGIN=$(pget "$PLANNER_NODE" "$PLANNER_ID.plugin" || true)
INNER=$(pget "$PLANNER_NODE" "$PLANNER_ID.inner_plugin" || true)
LPLUGINS=$(pget "$COSTMAP_NODE" plugins || true)
LCLASS=$(pget "$COSTMAP_NODE" "$LAYER.plugin" || true)
info "planner plugin: ${PLUGIN:-?}  inner: ${INNER:-?}"
info "global_costmap plugins: ${LPLUGINS:-?}  $LAYER.plugin: ${LCLASS:-?}"
if [[ $EXPECT == baseline ]]; then
  [[ $PLUGIN == nav2_theta_star_planner/ThetaStarPlanner ]] \
    && pass "baseline: plain Theta* planner" || fail "baseline expected Theta*, got '${PLUGIN:-?}'"
  [[ $LPLUGINS == *"$LAYER"* ]] \
    && fail "baseline expected no $LAYER in the costmap plugins" || pass "baseline: no $LAYER"
else
  [[ $PLUGIN == al_nav2_plugins/LocalizationAwarePlanner ]] \
    && pass "planner class is al_nav2_plugins/LocalizationAwarePlanner" \
    || fail "planner class is '${PLUGIN:-?}' (params file not the active one?)"
  [[ $INNER == nav2_theta_star_planner/ThetaStarPlanner ]] \
    && pass "inner planner is Theta*" || fail "inner planner is '${INNER:-?}'"
  [[ $LPLUGINS == *"$LAYER"* ]] && pass "$LAYER is in the global_costmap plugins" \
    || fail "$LAYER missing in the global_costmap plugins"
  [[ $LCLASS == al_nav2_plugins/LocalizabilityLayer ]] \
    && pass "layer class is al_nav2_plugins/LocalizabilityLayer" || fail "layer class '${LCLASS:-?}'"
fi

# --- 2. parameters
CMAX=0
LMFILE=''
LMIDS=''
LMNORM='0,-1'
if [[ $EXPECT != baseline ]]; then
  P_SCORER=$(pget "$PLANNER_NODE" "$PLANNER_ID.scorer" || true)
  L_SCORER=$(pget "$COSTMAP_NODE" "$LAYER.scorer" || true)
  L_ENABLED=$(pget "$COSTMAP_NODE" "$LAYER.enabled" || true)
  CMAX=$(pget "$COSTMAP_NODE" "$LAYER.c_max" || echo 0)
  LMFILE=$(pget "$COSTMAP_NODE" "$LAYER.landmarks_file" || true)
  LMIDS=$(parr "$COSTMAP_NODE" "$LAYER.marker_ids" || true)
  LMNORM=$(parr "$COSTMAP_NODE" "$LAYER.marker_normal" || echo '0,-1')
  [[ -n $LMNORM ]] || LMNORM='0,-1'
  info "planner scorer=${P_SCORER:-?}; $LAYER scorer=${L_SCORER:-?} enabled=${L_ENABLED:-?} c_max=${CMAX}"
  info "landmarks_file=${LMFILE:-?} marker_ids=[${LMIDS}]"
  if [[ $EXPECT == active ]]; then
    [[ $P_SCORER == marker_soft_count && $L_SCORER == marker_soft_count ]] \
      && pass "scorer is marker_soft_count in planner and layer" \
      || fail "active run but scorer is planner='${P_SCORER:-?}' layer='${L_SCORER:-?}'"
    [[ $L_ENABLED == True ]] && pass "$LAYER.enabled" || fail "$LAYER.enabled is '${L_ENABLED:-?}'"
    [[ $CMAX =~ ^[0-9]+$ && $CMAX -gt 0 ]] && pass "c_max=$CMAX (integer, > 0)" || fail "c_max='$CMAX'"
  else
    [[ $P_SCORER == null ]] && pass "control: planner scorer is null" \
      || fail "control expected planner scorer null, got '${P_SCORER:-?}'"
    [[ $L_SCORER == null || $L_ENABLED == False ]] \
      && pass "control: layer scorer null or layer disabled (spec v2: scorer null + enabled false)" \
      || fail "control expected the layer off, got scorer='${L_SCORER:-?}' enabled='${L_ENABLED:-?}'"
  fi
  # spec v2 values (a params file generated before RND-7 still has k_a 10, r_min 0.70, k_r 20, k_yaw_rate 1.0)
  for kv in "k_a 20" "r_min 0.66" "k_r 40"; do
    read -r key want <<< "$kv"
    for where in "$PLANNER_NODE $PLANNER_ID" "$COSTMAP_NODE $LAYER"; do
      read -r node prefix <<< "$where"
      v=$(pget "$node" "$prefix.$key" || true)
      if [[ -z $v ]]; then warn "$prefix.$key not readable"
      elif num_eq "$v" "$want" 1e-9; then pass "spec v2: $prefix.$key = $v"
      else warn "spec v2: $prefix.$key = $v, spec v2 says $want (stale params file?)"; fi
    done
  done
  v=$(pget "$PLANNER_NODE" "$PLANNER_ID.k_yaw_rate" || true)
  if [[ -z $v ]]; then warn "$PLANNER_ID.k_yaw_rate not readable"
  elif num_eq "$v" 2.0 1e-9; then pass "spec v2: $PLANNER_ID.k_yaw_rate = $v"
  else warn "spec v2: $PLANNER_ID.k_yaw_rate = $v, spec v2 says 2.0 (stale params file?)"; fi
fi
STEP=$(pget "$PLANNER_NODE" "$PLANNER_ID.step" || echo 0.20)

# --- 3 + 4. costmap and plan sample (python / rclpy)
PYARGS=(--expect "$EXPECT" --c-max "${CMAX:-0}" --step "$STEP" --landmarks-file "$LMFILE"
        --marker-ids "$LMIDS" --marker-normal "$LMNORM" --planner "$PLANNER_ID")
OUT=$(python3 "$PY" "${PYARGS[@]}" "${ARGS[@]}" </dev/null 2>&1)
RC=$?
printf '%s\n' "$OUT"
FAILS=$((FAILS + $(printf '%s\n' "$OUT" | grep -c '^FAIL')))
WARNS=$((WARNS + $(printf '%s\n' "$OUT" | grep -c '^WARN')))
[[ $RC -ne 0 && $(printf '%s\n' "$OUT" | grep -c '^FAIL') -eq 0 ]] && fail "check_live.py exited with $RC"

echo "== $([[ $FAILS -eq 0 ]] && echo PASS || echo FAIL): $FAILS failed, $WARNS warnings (expect: $EXPECT)"
[[ $FAILS -eq 0 ]]
