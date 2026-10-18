# al_nav2_plugins

Native Nav2 (Humble) plugins for the localization-aware navigation of ADR-001
(Option 2: yaw lever + route lever, scorer S2*). Spec: `_team/tasks/T-009_method_impl.md`.
The Python package `active_localization` is the reference implementation (test oracle,
offline score maps); this package must agree with it on the golden fixtures.

| plugin | base class | lever | what it does |
|---|---|---|---|
| `al_nav2_plugins/LocalizabilityLayer` | `nav2_costmap_2d::Layer` | route | adds `c_max * (1 - min(s*/s_ref, 1))^p` to the global costmap, where `s*` is the best-over-yaw marker detectability of the cell |
| `al_nav2_plugins/LocalizationAwarePlanner` | `nav2_core::GlobalPlanner` | yaw | runs an inner planner (Theta*), resamples its path at `step` and writes a heading into every pose by a Viterbi DP over `yaw_bins` headings |

The behaviour tree and the recoveries are the baseline's: the BT keeps calling
`ComputePathToPose` with `planner_id: GridBased`; only the plugin behind that id changes.

## Scorer S2* (spec section 2)

For marker `i` seen from camera position `c` with heading `psi`:

    w_i = sig(k_fov (HFOV/2 - |beta_i|)) * sig(k_a (A_i/a_min - 1)) * sig(k_r (r_i - r_min)) * occ_i
    A_i = (focal_px * marker_size / r_i)^2 * max(cos theta_i, 0)
    s   = 1 - exp(-sum_i w_i / m_sat)

`occ_i` is 0 when the segment camera -> marker (ending `occlusion_margin` in front of the
marker) crosses a cell with cost >= 254. A marker seen from behind keeps a small soft
floor `sig(-k_a)` instead of being clamped to 0 (as in the Python oracle).

## Parameters

Both plugins read the scorer under their own plugin name (`loc_layer.*` for the layer,
`GridBased.*` for the planner). Defaults are the spec v2 values.

| key | default | |
|---|---|---|
| `scorer` | `marker_soft_count` | `null` = control run: layer cost 0, DP with score 0 (forward prior + pins only) |
| `landmarks_file`, `marker_ids`, `marker_normal` | -, `[]` (all), `[0, -1]` | RTAB-Map landmark export; one outward face normal for all markers |
| `hfov`, `k_fov` | 1.089, 40 | camera field of view |
| `focal_px`, `marker_size` | 528.4, 0.08 | projected-area gate |
| `a_min`, `k_a` | 500, 20 | |
| `r_min`, `k_r` | 0.66, 40 | near limit |
| `m_sat` | 1 | |
| `cam_offset_x/_y/_yaw` | 0.07, 0, 0 | camera pose in `base_link` |
| `occlusion_margin` | 0.20 | |

`LocalizabilityLayer`: `enabled` (true), `c_max` (**integer**, 40), `yaw_bins` (36), `power` (1.0).
The layer is static: computed once per master-grid geometry from the static layer's map
(falls back to the master costmap after 20 update cycles without a static map), added
to the master as `min(master + cost, 252)` on cells below INSCRIBED (253), and
never written on INSCRIBED / LETHAL / NO_INFORMATION cells.

`LocalizationAwarePlanner`: `inner_plugin` (`nav2_theta_star_planner/ThetaStarPlanner`; its
parameters live under `<name>.inner.*`), `step` (0.20 m), `yaw_bins` (36), `w_s` (1.0),
`w_f` (0.3), `w_sm` (0.5), `k_yaw_rate` (2.0 rad/m), `w_lim` (100). The start / goal headings
are pinned to the start / goal poses, the goal pose is copied exactly, and on any failure (or a
path shorter than 2 poses) the inner path is returned unchanged with a WARN.
Occluders for the planner are the lethal cells of the live global costmap.

## Layout

    include/al_nav2_plugins/   header-only core (no ROS): geometry, scorer, occlusion,
                               cost_field (+ combineCost), yaw_dp, pose_utils
                               ROS side: landmarks_io, ros_params, the two plugin headers
    src/                       landmarks_io, localizability_layer, localization_aware_planner
    test/                      gtests + golden fixtures from the Python oracle (test/fixtures/)
    tools/build_and_test.sh    build + test in the `navigation` container (overlay workspace)

## Build and test

In WSL, from anywhere (needs docker and the running `navigation` container):

    bash <checkout>/navigation/src/al_nav2_plugins/tools/build_and_test.sh

It copies the package into an overlay workspace `~/dev_ws` inside the container (the
baseline workspace `~/ros2_ws` is not touched), builds with low parallelism
(`MAKEFLAGS=-j2`, one worker) and runs `colcon test`, including the ament linters.
Use the plugins from the overlay with `source ~/dev_ws/install/setup.bash` after
`~/ros2_ws/install/setup.bash`.

Tests: `test_scorer`, `test_yaw_dp`, `test_cost_field` (golden fixtures; scorer 1e-6, layer cost
+-1), `test_landmarks_io`, `test_planner` (planner on a synthetic costmap with the real Theta*).
Regenerate the fixtures with the Python oracle (`active_localization/tools/make_fixtures.py`)
and copy them to `test/fixtures/` when the parameters change.

## Modes

| mode | how |
|---|---|
| baseline | `nav2_params_v2.yaml`, untouched |
| active | `nav2_planner_custom/config/nav2_params_active.yaml`: `GridBased.plugin: al_nav2_plugins/LocalizationAwarePlanner`, `loc_layer` before `inflation_layer`, MPPI `use_path_orientations: true` |
| control | the active file with `GridBased.scorer: "null"` and `loc_layer.enabled: false` (same plugins, no effect: must match the baseline within noise) |

Experiment runs: `scripts/run_experiment.sh --mode baseline|active ...` (see `scripts/README.md`;
the control mode is added by ENG, T-009d).

## Live check

`tools/check_live.sh` (with `check_live.py`) checks a running Nav2: plugin classes loaded
(`ros2 param get` on `planner_server` / `global_costmap`), `loc_layer` and planner parameters
against spec v2, layer cost statistics and the cost at "marker seen from 1.88 m" cells (or
`--cell x,y,min,max`), and one `ComputePathToPose` sample (`--goal x,y[,yaw]` or
`--scenario FILE`): last pose = goal, pose spacing = `step`, start yaw pinned, yaw differs from the
path tangent (`--expect active`) or follows it (`--expect control`). Prints PASS / FAIL / WARN per
check; exit code 1 on any FAIL. Run it in the `navigation` container or from the host
(`NAV_CONTAINER` names the container). Thresholds tagged `[heuristic]` are judgement, not spec values.
