#!/usr/bin/env bash
# Run one scenario N times in one mode, each run from a clean simulator start.
#
#   scripts/run_experiment.sh --mode baseline|active --runs N --scenario <yaml>
#                             [--seed0 S] [--out DIR] [--overwrite] [--dry-run]
#
#   --mode M        baseline | active (active also starts ACTIVE_LAUNCH and
#                   uses NAV_LAUNCH_ACTIVE instead of NAV_LAUNCH_BASELINE)
#   --runs N        number of runs (>= 1)
#   --scenario F    scenario YAML (see navigation/src/al_eval/config/scenarios/)
#   --seed0 S       seed of run 1; run i uses S + i - 1 (default 42). Both
#                   modes get the same seed list. --seed is an alias.
#   --out DIR       host directory for run dirs (default ~/al_runs)
#   --overwrite     allow existing <out>/<mode>_<NN> (otherwise refuse)
#   --dry-run       print every command, execute nothing (except validating
#                   the scenario on the host)
#
# Per run i (name <mode>_<NN>, e.g. baseline_01, as in eval/README.md):
#   1. stop every launched stack in both containers (pkill, no compose down)
#   2. start GAZEBO, LOC, NOISE (seed), NAV, RECORDER (run_name), [ACTIVE]
#      in the background inside the containers, with startup pauses
#   3. readiness is checked by scenario_runner itself (action server
#      /navigate_to_pose and TF map->base_link, with timeouts)
#   4. run scenario_runner (foreground); SIGINT the recorder (meta.yaml);
#      stop the stacks
#   5. copy the container run dir to <out>/<mode>_<NN>/ and write run.yaml
#
# Result per run: <out>/<mode>_<NN>/{gt.tum, est.tum, meta.yaml, result.yaml,
# scenario.yaml, run.yaml, logs/} -> eval/run_evo.sh and eval/aggregate.py.
#
# Exit code: 0 all runs succeeded, 3 at least one run failed (see
# run.yaml:runner_exit_code), 2 bad arguments / scenario, 1 other errors.
#
# Every launch command and container setting below can be overridden from
# the environment, e.g.  GAZEBO_STARTUP_S=30 scripts/run_experiment.sh ...
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

# ===================================================================== config
# ENG: verify every value in this block on the testbed. [unverified]

: "${COMPOSE_FILE:=$REPO_ROOT/compose.yaml}"     # root compose (includes both)
: "${GAZEBO_SVC:=gazebo}"                        # compose service names
: "${NAV_SVC:=navigation}"
# Non-interactive `bash -c` does not run the ROS lines of ~/.bashrc (Ubuntu's
# .bashrc returns early when not interactive), so source explicitly.
: "${ROS_SETUP:=source /opt/ros/humble/setup.bash && source /home/app/ros2_ws/install/setup.bash}"
# Recorder output_dir inside the navigation container (not a host volume:
# run dirs are copied out with `docker compose cp` after each run).
: "${C_RUNS_DIR:=/home/app/al_runs}"
: "${C_GZ_LOG_DIR:=/tmp/al_experiment}"          # gazebo container logs

# Launch commands. Placeholders: @SEED@ @RUN_NAME@ @RUNS_DIR@ @MODE@
# ENG: verify (package / launch file names are taken from the repo).
: "${GAZEBO_LAUNCH:=ros2 launch world_and_robot gazebo_sim.launch.py gazebo_gui:=false}"
: "${LOC_LAUNCH:=ros2 launch rtabmap_landmarks complete_system.launch.py localization:=true}"
: "${NOISE_LAUNCH:=ros2 launch al_odom_noise odom_noise.launch.py seed:=@SEED@}"
: "${NAV_LAUNCH_BASELINE:=ros2 launch nav2_planner_custom navigation_launch.py}"
# T-009: point params_file at the active-mode Nav2 params once they exist.
: "${NAV_LAUNCH_ACTIVE:=ros2 launch nav2_planner_custom navigation_launch.py}"
# Active node from navigation/src/active_localization (T-008, PR #2);
# the scorer is chosen with scorer:=<name> (T-009).
: "${ACTIVE_LAUNCH:=ros2 launch active_localization active_localization.launch.py}"
# NavigateToPose action the runner sends goals to, per mode.
: "${ACTION_BASELINE:=/navigate_to_pose}"
: "${ACTION_ACTIVE:=/al/navigate_to_pose}"
: "${RECORDER_LAUNCH:=ros2 launch al_eval traj_recorder.launch.py run_name:=@RUN_NAME@ output_dir:=@RUNS_DIR@}"
# Extra arguments for scenario_runner (e.g. --server-timeout 180).
: "${RUNNER_ARGS:=}"

# Pauses after starting each stack [s] (readiness itself is checked by
# scenario_runner; these only keep the start order meaningful).
: "${GAZEBO_STARTUP_S:=15}"   # spawn at +6 s, lidar_stitching at +8 s
: "${LOC_STARTUP_S:=10}"
: "${NOISE_STARTUP_S:=2}"
: "${NAV_STARTUP_S:=10}"
: "${RECORDER_STARTUP_S:=2}"
: "${ACTIVE_STARTUP_S:=2}"
: "${STOP_TIMEOUT_S:=20}"            # SIGINT grace before SIGKILL
: "${RECORDER_STOP_TIMEOUT_S:=15}"   # wait for meta.yaml after SIGINT

# Process patterns (pkill -f, extended regex) used to reach a clean state.
# The [x] bracket keeps the pattern from matching a shell whose command line
# contains the pattern itself.
: "${STOP_PATTERN:=[r]os2 launch}"
# Leftovers after SIGKILL of `ros2 launch` (launch_ros nodes carry
# --ros-args; gzserver/gzclient are plain processes).
: "${KILL_PATTERN:=[r]os2 launch|[-]-ros-args|[g]zserver|[g]zclient}"

# ===================================================================== args
usage() { sed -n '2,36p' "$0"; exit "${1:-0}"; }

MODE=''; RUNS=''; SCENARIO=''; SEED0=42; OUT="$HOME/al_runs"
DRY_RUN=0; OVERWRITE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode|--runs|--scenario|--seed0|--seed|--out)
      [[ $# -ge 2 ]] || { echo "option $1 needs a value" >&2; exit 2; } ;;
  esac
  case "$1" in
    --mode) MODE=${2:-}; shift 2 ;;
    --runs) RUNS=${2:-}; shift 2 ;;
    --scenario) SCENARIO=${2:-}; shift 2 ;;
    --seed0|--seed) SEED0=${2:-}; shift 2 ;;
    --out) OUT=${2:-}; shift 2 ;;
    --overwrite) OVERWRITE=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage 0 ;;
    *) echo "unknown option: $1" >&2; usage 2 ;;
  esac
done

die() { echo "ERROR: $1" >&2; exit "${2:-1}"; }   # die MESSAGE [CODE]
[[ $MODE == baseline || $MODE == active ]] \
  || die "--mode must be baseline or active (got '$MODE')" 2
[[ $RUNS =~ ^[0-9]+$ && $RUNS -ge 1 ]] || die "--runs must be an integer >= 1" 2
[[ $SEED0 =~ ^-?[0-9]+$ ]] || die "--seed0 must be an integer" 2
[[ -n $SCENARIO ]] || die "--scenario is required" 2
[[ -f $SCENARIO ]] || die "scenario not found: $SCENARIO" 2
SCENARIO=$(cd "$(dirname "$SCENARIO")" && pwd)/$(basename "$SCENARIO")
[[ -n $OUT ]] || die "--out must not be empty" 2

# Validate the scenario on the host with the same code the runner uses.
if python3 -c 'import yaml' 2>/dev/null; then
  if ! SCENARIO_NAME=$(PYTHONPATH="$REPO_ROOT/navigation/src/al_eval${PYTHONPATH:+:$PYTHONPATH}" \
                       python3 -m al_eval.scenario "$SCENARIO"); then
    exit 2   # the error message is already on stderr
  fi
else
  echo "WARNING: python3 + PyYAML not found on the host: scenario not" \
       "validated here (scenario_runner validates it again)" >&2
  SCENARIO_NAME=$(sed -n 's/^name:[[:space:]]*\([^#[:space:]]*\).*/\1/p' \
                  "$SCENARIO" | head -n 1)
fi

if [[ $MODE == active && $NAV_LAUNCH_ACTIVE == "$NAV_LAUNCH_BASELINE" ]]; then
  # Without config/nav2_active_overrides.yaml MPPI ignores the path headings
  # written by al_navigator (T-009 wires the active params in).
  echo "# WARNING: NAV_LAUNCH_ACTIVE equals NAV_LAUNCH_BASELINE (active Nav2 params not applied yet)" >&2
fi

if [[ $MODE == active ]]; then NAV_LAUNCH=$NAV_LAUNCH_ACTIVE; ACTION=$ACTION_ACTIVE
else NAV_LAUNCH=$NAV_LAUNCH_BASELINE; ACTION=$ACTION_BASELINE; fi

DC=(docker compose -f "$COMPOSE_FILE")

# ===================================================================== helpers
# Readable shell quoting for printed commands.
q() {
  local a out=''
  for a in "$@"; do
    if [[ $a =~ ^[A-Za-z0-9_./:=@%+,-]+$ ]]; then out+="$a "
    else out+="'${a//\'/\'\\\'\'}' "; fi
  done
  printf '%s' "${out% }"
}

# run CMD...: execute, or print in --dry-run.
run() {
  if [[ $DRY_RUN -eq 1 ]]; then echo "+ $(q "$@")"; else "$@"; fi
}

pause() {
  if [[ $DRY_RUN -eq 1 ]]; then echo "+ sleep $1"; else sleep "$1"; fi
}

# subst TEMPLATE: replace @SEED@ @RUN_NAME@ @RUNS_DIR@ @MODE@.
subst() {
  local s=$1
  s=${s//@SEED@/$SEED}
  s=${s//@RUN_NAME@/$RUN}
  s=${s//@RUNS_DIR@/$C_RUNS_DIR}
  s=${s//@MODE@/$MODE}
  printf '%s' "$s"
}

# start NAME SVC LOGDIR CMD: start CMD in the background inside SVC; its pid
# goes to LOGDIR/NAME.pid, its output to LOGDIR/NAME.log. `exec` keeps the
# pid of the shell, so the pid file is the pid of `ros2 launch`.
declare -a STARTED=()   # "NAME SVC LOGDIR"
start() {
  local name=$1 svc=$2 logdir=$3 cmd=$4
  echo "# start $name ($svc): $cmd"
  run "${DC[@]}" exec -d -T "$svc" bash -c \
    "mkdir -p $logdir && echo \$\$ > $logdir/$name.pid && $ROS_SETUP && exec $cmd > $logdir/$name.log 2>&1"
  STARTED+=("$name $svc $logdir")
}

# check_alive: every started stack must still be running.
check_alive() {
  local entry name svc logdir dead=''
  for entry in "${STARTED[@]}"; do
    read -r name svc logdir <<<"$entry"
    if [[ $DRY_RUN -eq 1 ]]; then
      echo "+ $(q "${DC[@]}" exec -T "$svc" bash -c "kill -0 \$(cat $logdir/$name.pid)")"
      continue
    fi
    if ! "${DC[@]}" exec -T "$svc" bash -c "kill -0 \$(cat $logdir/$name.pid)" 2>/dev/null; then
      echo "ERROR: $name ($svc) is not running; last log lines:" >&2
      "${DC[@]}" exec -T "$svc" tail -n 20 "$logdir/$name.log" >&2 || true
      dead+="$name "
    fi
  done
  [[ -z $dead ]] || { DEAD_STACKS=$dead; return 1; }
}

# stop_stacks: SIGINT every `ros2 launch` in both containers, wait, SIGKILL
# whatever ROS / Gazebo process is left. Both containers keep running.
STACKS_UP=0
stop_stacks() {
  local svc t
  echo "# stop all stacks"
  for svc in "$GAZEBO_SVC" "$NAV_SVC"; do
    run "${DC[@]}" exec -T "$svc" pkill -INT -f "$STOP_PATTERN" || true
  done
  if [[ $DRY_RUN -eq 1 ]]; then
    echo "+ wait until no '$STOP_PATTERN' is left (timeout ${STOP_TIMEOUT_S} s)"
  else
    for ((t = 0; t < STOP_TIMEOUT_S; t++)); do
      if ! "${DC[@]}" exec -T "$GAZEBO_SVC" pgrep -f "$STOP_PATTERN" >/dev/null 2>&1 \
         && ! "${DC[@]}" exec -T "$NAV_SVC" pgrep -f "$STOP_PATTERN" >/dev/null 2>&1; then
        break
      fi
      sleep 1
    done
  fi
  for svc in "$GAZEBO_SVC" "$NAV_SVC"; do
    run "${DC[@]}" exec -T "$svc" pkill -KILL -f "$KILL_PATTERN" || true
  done
  STARTED=()
  STACKS_UP=0
}

cleanup() {
  local rc=$?
  if [[ $DRY_RUN -eq 0 && $STACKS_UP -eq 1 ]]; then
    echo "# interrupted / failed: stopping stacks" >&2
    stop_stacks || true
  fi
  exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# YAML single-quoted scalar.
yq() { local s=${1//\'/\'\'}; printf "'%s'" "$s"; }

write_run_yaml() {
  local path=$1
  local text
  text=$(cat <<EOF
run_name: $(yq "$RUN")
mode: $MODE
run_index: $i
seed: $SEED
scenario: $(yq "$SCENARIO_NAME")
scenario_file: $(yq "$SCENARIO")
git_commit: $(yq "$GIT_COMMIT")
git_dirty: $GIT_DIRTY
start_wall: $(yq "$START_WALL")
end_wall: $(yq "$END_WALL")
runner_exit_code: $RUNNER_RC
error: $(yq "$RUN_ERROR")
launch:
  gazebo: $(yq "$(subst "$GAZEBO_LAUNCH")")
  localization: $(yq "$(subst "$LOC_LAUNCH")")
  odom_noise: $(yq "$(subst "$NOISE_LAUNCH")")
  navigation: $(yq "$(subst "$NAV_LAUNCH")")
  recorder: $(yq "$(subst "$RECORDER_LAUNCH")")
  active: $(if [[ $MODE == active ]]; then yq "$(subst "$ACTIVE_LAUNCH")"; else echo null; fi)
EOF
)
  if [[ $DRY_RUN -eq 1 ]]; then
    echo "+ write $path:"
    local line
    while IFS= read -r line; do echo "    $line"; done <<<"$text"
  else
    printf '%s\n' "$text" > "$path.tmp" && mv "$path.tmp" "$path"
  fi
}

# ===================================================================== main
GIT_COMMIT=$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)
if [[ -n $(git -C "$REPO_ROOT" status --porcelain 2>/dev/null) ]]; then
  GIT_DIRTY=true; else GIT_DIRTY=false; fi

# Both containers must be up (they are started with `docker compose up -d`;
# this script never starts, stops or edits them).
if [[ $DRY_RUN -eq 0 ]]; then
  command -v docker >/dev/null || die "docker not found"
  RUNNING=$("${DC[@]}" ps --status running --services 2>/dev/null || true)
  for svc in "$GAZEBO_SVC" "$NAV_SVC"; do
    grep -qx "$svc" <<<"$RUNNING" \
      || die "compose service '$svc' is not running (docker compose up -d)"
  done
fi

# Refuse to clobber existing run dirs before anything is started.
for ((i = 1; i <= RUNS; i++)); do
  RUN=$(printf '%s_%02d' "$MODE" "$i")
  if [[ -e $OUT/$RUN && $OVERWRITE -eq 0 && $DRY_RUN -eq 0 ]]; then
    die "$OUT/$RUN already exists (use --overwrite or another --out)" 2
  fi
done

echo "# mode=$MODE runs=$RUNS seed0=$SEED0 scenario=$SCENARIO_NAME" \
     "($SCENARIO) out=$OUT commit=$GIT_COMMIT dirty=$GIT_DIRTY" \
     "$( [[ $DRY_RUN -eq 1 ]] && echo '[dry-run]')"

declare -a SUMMARY=()
FAILED=0
for ((i = 1; i <= RUNS; i++)); do
  RUN=$(printf '%s_%02d' "$MODE" "$i")
  SEED=$((SEED0 + i - 1))
  HOST_RUN=$OUT/$RUN
  C_RUN=$C_RUNS_DIR/$RUN
  C_LOG=$C_RUN/logs
  G_LOG=$C_GZ_LOG_DIR/$RUN
  RUNNER_RC=null
  RUN_ERROR=''
  echo
  echo "=== run $i/$RUNS: $RUN seed=$SEED ==="
  START_WALL=$(date -Iseconds)

  # 1. clean state
  stop_stacks
  run "${DC[@]}" exec -T "$NAV_SVC" bash -c "rm -rf $C_RUN && mkdir -p $C_LOG"
  run "${DC[@]}" exec -T "$GAZEBO_SVC" bash -c "rm -rf $G_LOG && mkdir -p $G_LOG"
  run "${DC[@]}" cp "$SCENARIO" "$NAV_SVC:$C_RUN/scenario.yaml"

  # 2. stacks
  STACKS_UP=1
  start GAZEBO "$GAZEBO_SVC" "$G_LOG" "$(subst "$GAZEBO_LAUNCH")"
  pause "$GAZEBO_STARTUP_S"
  start LOC "$NAV_SVC" "$C_LOG" "$(subst "$LOC_LAUNCH")"
  pause "$LOC_STARTUP_S"
  start NOISE "$NAV_SVC" "$C_LOG" "$(subst "$NOISE_LAUNCH")"
  pause "$NOISE_STARTUP_S"
  start NAV "$NAV_SVC" "$C_LOG" "$(subst "$NAV_LAUNCH")"
  pause "$NAV_STARTUP_S"
  start RECORDER "$NAV_SVC" "$C_LOG" "$(subst "$RECORDER_LAUNCH")"
  pause "$RECORDER_STARTUP_S"
  if [[ $MODE == active ]]; then
    start ACTIVE "$NAV_SVC" "$C_LOG" "$(subst "$ACTIVE_LAUNCH")"
    pause "$ACTIVE_STARTUP_S"
  fi

  # 3.+4. readiness + goals (scenario_runner), then stop the recorder
  DEAD_STACKS=''
  if check_alive; then
    echo "# scenario_runner (checks $ACTION + TF map->base_link)"
    RUNNER_CMD="ros2 run al_eval scenario_runner --scenario $C_RUN/scenario.yaml --out $C_RUN/result.yaml --run-name $RUN --mode $MODE --seed $SEED --action $ACTION${RUNNER_ARGS:+ $RUNNER_ARGS} --ros-args -p use_sim_time:=true"
    if run "${DC[@]}" exec -T "$NAV_SVC" bash -c \
         "$ROS_SETUP && $RUNNER_CMD 2>&1 | tee $C_LOG/scenario_runner.log; exit \${PIPESTATUS[0]}"; then
      RUNNER_RC=0
    else
      RUNNER_RC=$?
      RUN_ERROR="scenario_runner exit code $RUNNER_RC"
    fi
  else
    RUNNER_RC=5
    RUN_ERROR="stack(s) not running before scenario_runner: ${DEAD_STACKS% }"
    echo "ERROR: $RUN_ERROR" >&2
  fi
  [[ $DRY_RUN -eq 1 ]] && RUNNER_RC=null   # nothing was executed

  echo "# stop RECORDER with SIGINT so that it writes meta.yaml"
  run "${DC[@]}" exec -T "$NAV_SVC" bash -c "kill -INT \$(cat $C_LOG/RECORDER.pid)" || true
  if [[ $DRY_RUN -eq 1 ]]; then
    echo "+ wait until $C_RUN/meta.yaml exists (timeout ${RECORDER_STOP_TIMEOUT_S} s)"
  else
    for ((t = 0; t < RECORDER_STOP_TIMEOUT_S; t++)); do
      "${DC[@]}" exec -T "$NAV_SVC" test -s "$C_RUN/meta.yaml" && break
      sleep 1
    done
    "${DC[@]}" exec -T "$NAV_SVC" test -s "$C_RUN/meta.yaml" \
      || { echo "WARNING: no meta.yaml from the recorder" >&2
           RUN_ERROR="${RUN_ERROR:+$RUN_ERROR; }no meta.yaml"; }
  fi
  stop_stacks

  # 5. collect
  run mkdir -p "$HOST_RUN/logs/gazebo"
  run "${DC[@]}" cp "$NAV_SVC:$C_RUN/." "$HOST_RUN/" \
    || RUN_ERROR="${RUN_ERROR:+$RUN_ERROR; }copy from $NAV_SVC failed"
  run "${DC[@]}" cp "$GAZEBO_SVC:$G_LOG/." "$HOST_RUN/logs/gazebo/" \
    || RUN_ERROR="${RUN_ERROR:+$RUN_ERROR; }copy from $GAZEBO_SVC failed"
  END_WALL=$(date -Iseconds)
  write_run_yaml "$HOST_RUN/run.yaml"

  [[ $RUNNER_RC == 0 || $DRY_RUN -eq 1 ]] || FAILED=$((FAILED + 1))
  SUMMARY+=("$RUN seed=$SEED runner_exit_code=$RUNNER_RC")
done

echo
echo "=== summary ($MODE) ==="
printf '  %s\n' "${SUMMARY[@]}"
echo "next:"
echo "  for r in $OUT/${MODE}_*; do eval/run_evo.sh \"\$r\"; done"
echo "  python3 eval/aggregate.py --runs $OUT/baseline_* $OUT/active_* \\"
echo "    --labels <one label per run> --align none --out results/"
[[ $FAILED -eq 0 ]] || exit 3
