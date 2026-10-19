# scripts

## run_experiment.sh: repeated scenario runs

Runs one scenario (an ordered list of Nav2 goals) N times in one mode. Every
run starts from a clean simulator (the robot cannot be teleported: there is
no `gazebo_ros_state` plugin, so each run restarts the sim, localization and
nav, and the robot starts at the spawn pose (-2.0, -0.5)).

```bash
# prerequisites: containers up, al_eval / al_odom_noise built in the
# navigation container (needed after every change of their setup.py)
docker compose up -d
docker compose exec navigation bash -c \
  'source /opt/ros/humble/setup.bash && cd ~/ros2_ws && colcon build --packages-select al_eval al_odom_noise'

S=navigation/src/al_eval/config/scenarios/example.yaml   # PLACEHOLDER goals
scripts/run_experiment.sh --mode baseline --runs 3 --scenario $S --seed0 42 --dry-run
scripts/run_experiment.sh --mode baseline --runs 3 --scenario $S --seed0 42
NAV_LAUNCH_ACTIVE='ros2 launch nav2_planner_custom navigation_launch.py <active params, T-009>' \
  scripts/run_experiment.sh --mode active --runs 3 --scenario $S --seed0 42
```

Options: `--mode baseline|active`, `--runs N`, `--scenario <yaml>`,
`--seed0 S` (default 42; run i uses S+i-1, the same list for both modes, so
`baseline_k` and `active_k` share the odometry-noise seed), `--out DIR`
(default `~/al_runs`), `--overwrite`, `--dry-run` (prints every command and
executes nothing; only the scenario is validated on the host).

### Testbed set-up before the first run (ENG, T-009d / T-004)

1. **Odometry noise wiring (T-004, D-3).** `gazebo_cmd_vel.xacro` publishes the
   ideal planar_move odometry as `/odom_gt` without TF; `al_odom_noise`
   (started by the runner as `NOISE_LAUNCH`) publishes `/odom` + TF
   `odom->base_link`. Outside the runner start it by hand
   (`ros2 launch al_odom_noise odom_noise.launch.py`), otherwise there is no `/odom`.
2. **Pristine map DB.** RTAB-Map in localization mode starts from the last
   localization saved in the DB and writes to it (FACTS 18.10). The runner copies
   `PRISTINE_DB` over `DB_PATH` before every run. Make the pristine copy once,
   right after mapping (the DB in `navigation/maps/` is mounted at
   `/home/app/ros2_ws/maps`; on the WSL testbed `~/al_maps` is mounted instead):
   `cp maps/astribot_sim.db maps/astribot_sim.pristine.db`.
   `scripts/get_map.sh` does this for you after a verified download (see below).
3. **Start-up nudge** (`scripts/al_nudge.py`, both modes): RTAB-Map publishes no
   `map->odom` until the robot moves, and Nav2 never activates without it. The
   runner drives 0.6 m forward and back on `/odom_gt` before starting Nav2
   (`NUDGE_ENABLE`, `NUDGE_ARGS`).
4. **Build** `al_eval al_odom_noise active_localization` in the navigation
   container (see above) — they are not in the image yet.
5. **Active mode** uses `nav2_planner_custom/config/nav2_params_active.yaml`
   (`NAV_PARAMS_ACTIVE`) = baseline params + the T-009 deltas; it needs the
   `al_nav2_plugins` package (T-009b/c). `ACTIVE_LAUNCH` is empty by default.
6. **Windows / WSL2 with a GPU** (ENG testbed): render the camera on the GPU via
   WSLg D3D12 (`compose.override.yaml` mounts `/dev/dxg` and `/usr/lib/wsl`):
   ```bash
   export GAZEBO_LAUNCH='env LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA ros2 launch world_and_robot gazebo_sim.launch.py gazebo_gui:=false software_rendering:=false'
   ```
   (camera ~27 Hz sim instead of ~11 Hz with llvmpipe; FACTS 17.10).

### Per run

1. Stop every launched stack in both containers: SIGINT to `ros2 launch`,
   wait up to `STOP_TIMEOUT_S`, then SIGKILL of anything left that matches
   `KILL_PATTERN` (`ros2 launch`, `--ros-args`, `gzserver`, `gzclient`).
   This kills **all** ROS processes in both containers, including ones
   started by hand. The containers themselves are never stopped and the
   compose files are not touched.
2. Start in the background, in this order, with a pause after each:
   `GAZEBO_LAUNCH` (gazebo container), `LOC_LAUNCH`, `NOISE_LAUNCH`
   (`seed:=<seed>`), `NAV_LAUNCH_BASELINE` or `NAV_LAUNCH_ACTIVE`,
   `RECORDER_LAUNCH` (`run_name:=<mode>_<NN>`), and `ACTIVE_LAUNCH` in active
   mode. Then check that every one of them is still alive.
3. Readiness is checked by `scenario_runner` itself: it waits (with
   timeouts) for the `/navigate_to_pose` action server and for TF
   `map -> base_link`, then waits `settle_s`, then sends the goals.
4. After `scenario_runner` exits, the recorder gets SIGINT (so it writes
   `meta.yaml`), then all stacks are stopped.
5. The container run dir is copied to the host with `docker compose cp`
   and `run.yaml` is written.

Every launch command, pause, timeout, service name and container path is a
variable at the top of `run_experiment.sh` and can be overridden from the
environment (`GAZEBO_STARTUP_S=30 scripts/run_experiment.sh ...`).
Placeholders in launch commands: `@SEED@`, `@RUN_NAME@`, `@RUNS_DIR@`,
`@MODE@`. **ENG owns the final values** (they are marked `ENG: verify`).

### Output: `<out>/<mode>_<NN>/`

| file | from |
|---|---|
| `gt.tum`, `est.tum`, `meta.yaml` | `traj_recorder` |
| `result.yaml` | `scenario_runner`: per-goal status (`succeeded`, `aborted`, `canceled`, `timeout`, `rejected`, `error`, `skipped`), wall/sim duration, sim-time start/end stamps (`start_sim_s`/`end_sim_s`, the evaluation window of `eval/crop_run.py`), GT path length, final xy/yaw error of the estimated pose, Nav2 recoveries; summary |
| `scenario.yaml` | copy of the scenario used |
| `run.yaml` | mode, seed, scenario name/file, git commit (+ dirty flag), start/end wall time, `runner_exit_code`, error, resolved launch commands |
| `logs/*.log`, `logs/gazebo/*.log` | output of every stack and of `scenario_runner` |

The run dir names (`baseline_01`, `active_01`, ...) are the ones used in
`eval/README.md`; the dirs go to `eval/run_evo.sh` and `eval/aggregate.py`
as they are:

```bash
for r in ~/al_runs/{baseline,active}_0{1,2,3}; do eval/run_evo.sh "$r"; done
python3 eval/aggregate.py \
  --runs ~/al_runs/baseline_0{1,2,3} ~/al_runs/active_0{1,2,3} \
  --labels baseline baseline baseline active active active \
  --align none --out results/
```

### Exit codes

`run_experiment.sh`: 0 all runs succeeded, 3 at least one run failed, 2 bad
arguments / invalid scenario, 1 other errors (e.g. a container is not
running).

`runner_exit_code` in `run.yaml` (from `scenario_runner`): 0 all goals
succeeded, 3 a goal failed / was rejected / timed out, 4 action server or TF
not available in time, 2 bad scenario, 130 interrupted; 5 is set by the
script when a stack died before `scenario_runner` started; `null` in a dry
run.

## get_map.sh: download the map database

The RTAB-Map database (`astribot_sim.db`, about 150 MB) is not in git. `scripts/get_map.sh`
downloads it from a GitHub Release asset into `navigation/maps/`, verifies its SHA-256 against
`navigation/maps/astribot_sim.db.sha256` (`<hex>  astribot_sim.db`, as `sha256sum` prints it) and
makes the pristine copy `astribot_sim.pristine.db` that `run_experiment.sh` restores before each run.

```bash
scripts/get_map.sh                                   # URL and checksum from the script / manifest
scripts/get_map.sh --url <url> --sha256 <hex> --out ~/al_maps   # explicit (e.g. the WSL testbed)
scripts/get_map.sh --verify-only                     # check only; also accepts a pristine copy
scripts/get_map.sh --print-sha256 <file>             # maintainers: hash to publish with the release
```

A mismatch discards the download (exit 3); an existing different DB is only replaced with
`--force` (exit 4; a run modifies the working DB, the pristine copy is what counts). A `.gz` URL is
gunzipped before the check. `AL_MAP_URL` / `AL_MAP_SHA256` override the defaults. **TODO(ENG/LEAD):**
the release URL and the checksum in the manifest are placeholders until the DB is uploaded; the
script refuses to run with them. Exit codes: `--help`.

## Scenario files

See `navigation/src/al_eval/config/scenarios/example.yaml`:

```yaml
name: example
frame_id: map          # only "map" is accepted
goal_timeout_s: 120    # per goal, wall clock; on timeout the goal is cancelled
settle_s: 2.0          # wait after readiness, before the first goal
goals:                 # x, y [m], yaw [rad]
  - {x: 1.0, y: 0.0, yaw: 0.0}
```

Validate a file without ROS:
`PYTHONPATH=navigation/src/al_eval python3 -m al_eval.scenario <yaml>`.
`scenario_runner` can also be run on its own inside the navigation container:
`ros2 run al_eval scenario_runner --scenario <yaml> --out result.yaml --ros-args -p use_sim_time:=true`
(`--help` for all options).

## Not verified yet

Written without ROS / Docker; ENG checks on the testbed: launch commands
and their arguments, startup pauses, that SIGINT/SIGKILL leaves no stale
nodes (e.g. RViz started by `rsp.launch.py` in the gazebo container),
that the recorder writes `meta.yaml` on SIGINT of its `ros2 launch`,
`docker compose cp` of a directory, and the rclpy / Nav2 behaviour of
`scenario_runner`.
