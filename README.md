# Active Localization

Active localization for a mobile robot in simulation: the robot plans its route and its heading
so that the STag markers it localizes against stay observable, and we measure whether that
improves pose estimation. Testbed: the Astribot robot in Gazebo, lidar + marker SLAM
(RTAB-Map) and Nav2 (Theta* + MPPI). Method: two native Nav2 plugins (`al_nav2_plugins`).
Everything runs in two Docker containers (ROS 2 Humble).

> Status: the experiment (baseline vs active vs control) and the 2-page report are in progress.
> Items marked **TODO** are not final or not verified yet.

## Contents

1. [Overview](#1-overview) · 2. [Testbed](#2-testbed) · 3. [Method and plugins](#3-method-and-plugins) ·
4. [Prerequisites](#4-prerequisites) · 5. [Build](#5-build) · 6. [Map](#6-map) ·
7. [Run baseline / active / control](#7-run-baseline--active--control) · 8. [Evaluate](#8-evaluate) ·
9. [Live check](#9-live-check-of-the-plugins) · 10. [Report](#10-build-the-report) ·
11. [Repository layout](#11-repository-layout) · 12. [Operating notes](#12-operating-notes) ·
13. [Known limitations](#13-known-limitations) · 14. [License](#14-license)

## 1. Overview

The experiment compares three modes on the same scenario, start pose, map and odometry-noise
seeds (at least 3 runs each):

| Mode | Nav2 stack |
|---|---|
| `baseline` | stock Nav2 (`nav2_params_v2.yaml`, behavior tree v3), unchanged |
| `active` | the same behavior tree and controller, plus `al_nav2_plugins` through a separate params file (`nav2_params_active.yaml`) |
| `control` | the active stack with the scorer switched off (`scorer: "null"` in planner and layer, `loc_layer.enabled: false`): separates the effect of the method from the effect of swapping plugins and MPPI critics |

Metrics are APE/RPE (translation and rotation) of the SLAM estimate against ground truth,
computed with [evo](https://github.com/MichaelGrupp/evo), plus marker-detection statistics.

## 2. Testbed

- **Simulation** (`gazebo` container): Astribot S1 with an omnidirectional base
  (`gazebo_ros_planar_move`), front and back lidars merged into `/lidar/points_joined`
  (`lidar_stitching`), an RGB-D camera in the torso, and worlds with shelves and STag markers
  (HD15 small, HD17 large). Gazebo 11, run headless in experiments.
- **Navigation** (`navigation` container): STag detector (C++), RTAB-Map SLAM/localization on the
  lidar with the large HD17 markers as landmarks (`rtabmap_landmarks`, `tf_to_fiducial`), Nav2
  (Theta* global planner, MPPI controller, behavior tree v3 with recoveries), experiment tools
  (`al_eval`: scenario runner and trajectory recorder; `al_odom_noise`: reproducible odometry
  noise with a seed).
- **Coupling**: both containers use `network_mode: host` and the same `ROS_DOMAIN_ID`; they only
  exchange ROS 2 topics, so `gazebo` can be replaced by another simulator or a real robot:

| Direction | Topics / TF |
|---|---|
| `gazebo` → `navigation` | `/lidar/points_joined`, `/torso_rgbd/image_raw`, `/torso_rgbd/camera_info`, `/odom`, TF `odom → base_link` and the robot TF tree, `/clock` (simulation) |
| `navigation` → `gazebo` | `/cmd_vel` |
| `navigation` outward | TF `map → odom`, `/map`, `/goal_pose` (Nav2 input), action `/navigate_to_pose` |

Per-module details (still partly in Russian): [`gazebo`](gazebo/README.md),
[`navigation`](navigation/README.md), [`scripts`](scripts/README.md), [`eval`](eval/README.md).

## 3. Method and plugins

Two plugins from [`al_nav2_plugins`](navigation/src/al_nav2_plugins/README.md) change *how* the
robot drives, not where it goes: the behavior tree, the recovery behaviors and the
`/navigate_to_pose` action stay as in baseline.

| Plugin | Plugged in at | Lever |
|---|---|---|
| `al_nav2_plugins/LocalizabilityLayer` | layer `loc_layer` of `global_costmap` (before `inflation_layer`) | **route**: adds `c_max·(1 − min(s*/s_ref, 1))^p` to the cost of every free cell, where `s*` is the best marker observability over 36 headings in that cell; Theta* then prefers cells where markers are seen |
| `al_nav2_plugins/LocalizationAwarePlanner` | `planner_server`, plugin `GridBased` (wraps Theta*) | **heading**: after Theta*, a Viterbi DP over 36 headings (forward prior, smoothness, yaw-rate limit; start and goal pinned) writes the heading into every pose of the path, resampled at 0.20 m |

The observability score `s` is `1 − exp(−n_eff/M)` with `n_eff = Σ w_i` over the markers in view,
where `w_i = σ(k_fov(HFOV/2 − |β_i|))·σ(k_a(A_i/a_min − 1))·σ(k_r(r_i − r_min))·occ_i` is a soft
visibility of marker `i` (field of view, size in pixels, range, occlusion by the costmap). The
reference implementation is the Python package `navigation/src/active_localization`; the C++
core is checked against its JSON fixtures in `al_nav2_plugins/test/fixtures`.

In `active` and `control`, MPPI follows the planned headings: `PathAlignCritic.use_path_orientations: true`
and `PathAngleCritic` / `PreferForwardCritic` disabled (they would fight the path headings).
The main parameters (full table in the [package README](navigation/src/al_nav2_plugins/README.md#parameters)):

| Parameter | Value | Meaning |
|---|---|---|
| `loc_layer.c_max` | integer (`40.0` is rejected by rclcpp) | maximum cost added by the layer; tuned by dry runs, see `nav2_params_active.yaml` |
| `loc_layer.scorer`, `GridBased.scorer` | `marker_soft_count` / `"null"` | observability score / off (control) |
| `k_a`, `r_min`, `k_r` | `20`, `0.66`, `40` | soft visibility by marker size in pixels and range (spec v2) |
| `GridBased.k_yaw_rate` | `2.0` | yaw-rate limit per metre of path in the heading DP |
| `GridBased.step` | `0.20` | path resampling step (about 4× sparser than Theta*'s own output) |
| `marker_normal` / `marker_normal_axis` | `[0, -1]` / off | marker face normal: one for all markers, or derived per marker from the landmark orientation |

## 4. Prerequisites

- Docker with Docker Compose ≥ 2.20.3 (the root `compose.yaml` uses `include`) and an X server
  for the GUI (`xhost +local:docker`).
- GPU rendering for the camera is strongly recommended (on CPU the camera runs far below 30 Hz).
  WSL2/GPU notes: **TODO(ENG)** (experiments used D3D12 through `GALLIUM_DRIVER=d3d12` in the
  `gazebo` container, set via `GAZEBO_LAUNCH`).
- Host Python ≥ 3.10 with `evo` 1.38 (`pip install evo`, preferably in a venv; needs
  matplotlib ≥ 3.6).
- For the report: `latexmk` + `pdflatex` (TeX Live) and `pdfinfo` (poppler-utils).

## 5. Build

```bash
docker compose up -d --build          # both containers; or `cd gazebo` / `cd navigation` for one
docker compose exec navigation bash
# inside the navigation container:
source /opt/ros/humble/setup.bash && cd ~/ros2_ws
colcon build --packages-select al_eval al_odom_noise active_localization al_nav2_plugins
source install/setup.bash
```

`~/ros2_ws` in the container is the mounted `navigation/src`; the Nav2 params files are read from
there at launch time, so editing them needs no rebuild. **TODO:** confirm the package list for
the final run (`active_localization` is the Python reference and tooling; the C++ plugins do not
need it at run time).

Unit tests of the plugins (gtests against the Python oracle fixtures + ament linters), in a
separate overlay workspace so `~/ros2_ws` stays untouched; run on the host, needs docker:

```bash
navigation/src/al_nav2_plugins/tools/build_and_test.sh
```

## 6. Map

The RTAB-Map database (`astribot_sim.db`, about 150 MB) is not in git. Download the one used in
the report and verify its checksum:

```bash
scripts/get_map.sh        # -> navigation/maps/astribot_sim.db (+ astribot_sim.pristine.db)
```

The script downloads a GitHub Release asset, checks its SHA-256 against
`navigation/maps/astribot_sim.db.sha256` and keeps an untouched copy
`astribot_sim.pristine.db`: localization mode writes into the database, so
`scripts/run_experiment.sh` restores the pristine copy before every run. Options: `--url`,
`--sha256`, `--out`, `--force`, `--verify-only`, `--print-sha256 FILE` (see `--help`).
On the WSL testbed `~/al_maps` is mounted into the container instead of `navigation/maps`:
use `--out ~/al_maps` there.
**TODO(ENG/LEAD):** upload the database as a release asset, then set the URL in
`scripts/get_map.sh` and the checksum in `navigation/maps/astribot_sim.db.sha256`
(`scripts/get_map.sh --print-sha256 <file>`); until then the script stops with a clear message.

**Mapping from scratch** (fallback; the result will differ slightly from the shipped map):

1. Start the simulation in the `gazebo` container:
   `ros2 launch world_and_robot gazebo_sim.launch.py` (headless: `gazebo_gui:=false`).
2. In the `navigation` container start mapping:
   `ros2 launch rtabmap_landmarks complete_system.launch.py localization:=false delete_db_on_start:=true`.
3. Drive the robot around the shelf rows, facing +y so that the HD17 markers are seen. The route
   used for the shipped map, as `[x, y, yaw°]` waypoints in the odom/world frame:
   `[-2.0,-0.5,90] [2.75,-0.5,90] [2.75,2.4,90] [-3.8,2.4,90] [-3.8,4.7,90] [2.75,4.7,90]
   [2.75,-0.5,90] [-2.0,-0.5,90] [-2.0,-0.5,0]`.
   **TODO:** add the waypoint driver (currently outside the repository) to `scripts/`.
4. Stop `complete_system.launch.py` with one Ctrl+C and wait until `rtabmap` has written the
   database; then export the marker poses:
   `ros2 run rtabmap_landmarks export_landmarks_from_db`. The result,
   `navigation/src/rtabmap_landmarks/config/landmarks.yaml`, is read by both plugins and must
   match the database.
5. Save the database as `navigation/maps/astribot_sim.db` and make the pristine copy:
   `cp navigation/maps/astribot_sim.db navigation/maps/astribot_sim.pristine.db`.

Localization on a ready map: `ros2 launch rtabmap_landmarks complete_system.launch.py localization:=true`;
Nav2: `ros2 launch nav2_planner_custom navigation_launch.py` (the goal is set with *2D Goal Pose*
in RViz, Fixed Frame `map`).

## 7. Run baseline / active / control

`scripts/run_experiment.sh` runs one scenario N times in one mode. Every run starts from a clean
simulator (the robot cannot be teleported, so each run restarts the simulation, localization and
navigation and restores the pristine map). Run `i` uses seed `seed0 + i − 1` in every mode.

```bash
S=navigation/src/al_eval/config/scenarios/shelves_loop_a.yaml
scripts/run_experiment.sh --mode baseline --runs 3 --scenario $S --seed0 42
scripts/run_experiment.sh --mode active   --runs 3 --scenario $S --seed0 42
scripts/run_experiment.sh --mode control  --runs 3 --scenario $S --seed0 42
```

Add `--dry-run` to print every command without running anything. In `active` the runner uses
`nav2_params_active.yaml` (variable `NAV_PARAMS_ACTIVE`); `control` generates a per-run copy of it
with both scorers `"null"` and `loc_layer.enabled: false` and keeps it in the run directory.
Options, environment overrides and exit codes: [`scripts/README.md`](scripts/README.md).
**TODO:** the `control` mode and the final scenario/noise settings are described from the
runner on the T-009d branch; re-check these lines after it is merged.

**Odometry noise.** The scenario runs use `al_odom_noise` with the noise scaled by a factor `k`
(calibrated in D-3/D-5; the T-010 script used `k = 4`; **TODO:** final `k`): the covariance-like
parameters `k_xx … k_rt` are multiplied by `k²` and `scale_x`, `scale_y`, `scale_yaw`,
`yaw_drift_per_m` by `k`, written to a params file passed as
`NOISE_LAUNCH="ros2 launch al_odom_noise odom_noise.launch.py params_file:=<file> seed:=@SEED@"`.
**TODO(ENG):** ship the scaling helper (currently a snippet in an unversioned script).

## 8. Evaluate

```bash
# APE/RPE (translation + rotation), no alignment, cropped to the goal window
for r in ~/al_runs/{baseline,active,control}_0{1,2,3}; do eval/run_evo.sh "$r"; done
python3 eval/summarize.py ~/al_runs --out eval_out/                # tables (+ summary_table.tex)
python3 eval/plots.py ~/al_runs --out eval_out/plots --score-map   # figures
```

Details (evaluation window, metrics, paired view, sanity flag): [`eval/README.md`](eval/README.md).
`eval/aggregate.py` (APE/RPE only, two labels) still works on the same `evo_none/` directories.

| Path | Content |
|---|---|
| `~/al_runs/<mode>_<NN>/` | `gt.tum`, `est.tum`, `meta.yaml`, `result.yaml`, `scenario.yaml`, `run.yaml`, `logs/` (one directory per run) |
| `~/al_runs/<mode>_<NN>/` (after evaluation) | `*_eval.tum`, `landmarks_eval.csv`, `eval_window.yaml` (`eval/crop_run.py`, called by `run_evo.sh`) |
| `~/al_runs/<mode>_<NN>/evo_none/` | evo results `ape.zip`, `rpe.zip`, `ape_rot.zip`, `rpe_rot.zip`, plots, `commands.txt` |
| `results/` | `results.csv`, `summary.md`, `ape_rmse_boxplot.png` (`eval/aggregate.py`) |
| `eval_out/` | `summary.{csv,md}`, `summary_modes.csv`, `summary_paired.csv`, `summary_table.tex` (read by the report), `plots/` (copy the PDFs to `report/figures/`) |

The run directories and `eval_out/`, `results/` are git-ignored.

## 9. Live check of the plugins

With Nav2 running, `navigation/src/al_nav2_plugins/tools/check_live.sh` prints PASS/FAIL/WARN:
are the plugin classes loaded, do the `loc_layer` and planner parameters match spec v2 (stale
params files give a warning), the layer cost at a few cells, and one `ComputePathToPose` sample
whose headings differ from the path tangent (active) or follow it (control). It works inside
the `navigation` container and from the host.

```bash
navigation/src/al_nav2_plugins/tools/check_live.sh --expect active  --scenario <scenario.yaml>
navigation/src/al_nav2_plugins/tools/check_live.sh --expect control --goal 1.0,1.0,0.0
```

## 10. Build the report

```bash
report/build.sh        # -> report/report.pdf, fails if it is longer than 2 pages
```

The results table is taken from `eval_out/summary_table.tex` when it exists; the expected figure
names are listed in `report/figures/README.md`.

## 11. Repository layout

```
active-localization/
├── compose.yaml            # includes the compose files of both modules
├── README.md  LICENSE  CHANGELOG.md
├── gazebo/                 # simulation: Dockerfile, docker-compose.yml, src/{world_and_robot,lidar_stitching}
├── navigation/             # Dockerfile, docker-compose.yml, docker/ (stag_ros2_enhanced image), maps/
│   └── src/
│       ├── al_nav2_plugins/       # LocalizabilityLayer, LocalizationAwarePlanner (C++), tests, tools/
│       ├── active_localization/   # Python reference (scorer, DP) + fixtures generator
│       ├── al_eval/  al_odom_noise/   # scenario runner, trajectory recorder / seeded odometry noise
│       ├── nav2_planner_custom/   # Nav2 params (baseline, active), behavior trees, launch
│       ├── rtabmap_landmarks/  tf_to_fiducial/  stag_ros2_enhanced/   # SLAM with marker landmarks, STag detector
│       └── stag_experiment_manager/  stag_data_analyzer/              # detector experiments and analysis
├── scripts/                # run_experiment.sh, get_map.sh, al_nudge.py
├── eval/                   # run_evo.sh, summarize.py, plots.py, aggregate.py, tests/
├── report/                 # report.tex, build.sh, figures/
├── docs/                   # reference papers (PDF)
└── logs/                   # build and run logs (git-ignored)
```

## 12. Operating notes

- Allow the containers to use the X server: `xhost +local:docker`. Only one module:
  `cd gazebo` (or `navigation`) and `docker compose up -d --build`. Stop with `docker compose down`
  from the same folder: the root project and a module folder are different Compose projects.
- Build logs go to `logs/` with the module and time in the name:
  `docker compose build --progress=plain navigation 2>&1 | tee logs/build_navigation_$(date +%Y%m%d_%H%M%S).log`.
- DDS: `ROS_DOMAIN_ID=1`, `rmw_fastrtps_cpp`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (Fast DDS shared
  memory does not work between containers). Override before `docker compose up`, e.g.
  `ROS_DOMAIN_ID=25 docker compose up -d`.

## 13. Known limitations

- Simulation only, one world and one map; every run restarts the whole stack (no teleport).
- The C++ core is verified against the Python oracle and gtests; loading in a running Nav2 is
  covered by `tools/check_live.sh` and ENG's dry runs, not by an automated integration test
  (`LocalizabilityLayer` has no ROS-level unit test: `Layer::initialize` needs a full `LayeredCostmap`).
- The planner output is resampled at 0.20 m. MPPI's `PathAlignCritic` therefore needs a smaller
  `offset_from_furthest` than for Theta*'s dense path (see `nav2_params_active.yaml`); in the
  gate runs the active mode was slower and needed more recoveries than control.
  **TODO(ENG):** quote the final numbers from the T-010 summary.
- On the loop scenario the route lever is inert; the experiment tests the heading lever (see the
  report for the claim we make).
- `cppcheck` in the plugin lint run is a no-op with cppcheck 2.7 (ament refuses the version).
- One marker normal for all markers unless `marker_normal_axis` is set; that the local +x axis is
  the face normal of the real markers is not confirmed, so it is off by default.
- Module READMEs (`gazebo`, `navigation`, `eval`, `scripts`) and `CHANGELOG.md` are partly in Russian.

## 14. License

**TODO(DIR):** no license has been chosen yet, see [`LICENSE`](LICENSE). `al_nav2_plugins`
carries MIT headers; check that the Astribot meshes and the PDFs in `docs/` may be
redistributed.
