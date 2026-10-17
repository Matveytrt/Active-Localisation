# active_localization — localization-aware navigation on top of Nav2

Skeleton (T-008). Method-agnostic: the localizability **scorer** is a plug-in;
the method chosen at GATE-1 (ADR-001) is added as one more scorer in T-009.
Architecture and integration options: `_team/knowledge/ARCH.md`.

## Layout

| module | ROS? | what |
|---|---|---|
| `geometry.py` | no | SE(2) helpers, path resampling |
| `landmarks.py` | no | reads `rtabmap_landmarks/config/landmarks.yaml`, camera model |
| `scorers/` | no | `Scorer` interface + registry; method scorer `marker_soft_count` (S2*); reference scorers `null`, `visible_count` |
| `yaw_planner.py` | no | **yaw lever**: Viterbi DP over headings along a fixed path |
| `cost_field.py` | no | **route lever**: best-over-yaw score → occupancy cost grid (`cost_grid`, 0..100) and the spec §3 layer cost (`layer_cost_grid`, uint8 costmap units) |
| `config.py` | no | one YAML → scorer + planner + cost field params |
| `navigator_node.py` | yes | `al_navigator`: NavigateToPose server `al/navigate_to_pose`; ComputePathToPose → orient → FollowPath, 1 Hz replanning |
| `cost_field_node.py` | yes | `al_cost_field`: publishes `al/loc_cost` (latched OccupancyGrid) for an extra StaticLayer |

## Scorer contract

`score(x, y, yaw) -> ndarray` — numpy-broadcastable inputs, output has the
broadcast shape, finite, ≥ 0, higher = better localization expected.
`score_scale` = a typical "good enough" value. No ROS, no state.
Add a scorer: new module in `scorers/`, register it in `scorers/__init__.py`,
select it with `scorer.name` in the config or `scorer:=<name>` at launch.

Reference scorers (pipeline tests, **not** the method):
- `null` — 0 everywhere → robot faces the direction of travel, empty cost field
  (control run: our plumbing alone must not change results).
- `visible_count` — markers within HFOV / range / incidence, optional sigmoid
  edges, `inv_d2` weighting and saturation.

## S2* scorer (`marker_soft_count`, ADR-001, spec T-009 §2)

The method scorer and the oracle for the C++ plugins (`al_nav2_plugins`).
Per marker a product of soft gates — FoV (`k_fov`), projected area
A = (f s / r)^2 max(cos θ, 0) ≥ `a_min` (`k_a`), near limit `r_min` (`k_r`) —
times a rule-based occlusion flag; n_eff = Σ w, s = 1 − exp(−n_eff / M),
`score_scale` = s_ref = 1 − e^−1. Camera 0.07 m ahead of base_link
(`camera.offset_x`). Marker normals come from `scorer.params`
(`default_normal` (0, −1), per-id `normals`); `landmarks.normal_axis` is not
used by this scorer. Occlusion: the segment camera → (marker + 0.20 m along
its normal) must not touch any `occlusion_rows` rectangle (merged shelf rows,
[assumption]); the C++ layer ray-casts the static map instead.
Defaults = spec values, also written out in `config/active_localization.yaml`.

`cost_field.layer_cost_grid` / `layer_cost_from_score` = spec §3:
cost = round(c_max (1 − min(s*/s_ref, 1))^p), c_max = 40, uint8.

## Tools (offline, no ROS)

```bash
cd navigation/src/active_localization
python3 -I tools/score_maps.py --out <dir>   # s_star.png, layer_cost.png, best_yaw.png + numbers
python3 -I tools/make_fixtures.py            # rewrites test/fixtures/*.json
```

Both use the real `rtabmap_landmarks/config/landmarks.yaml` (ids 1–5), the
spec §2 defaults and camera offset 0.07 m (`tools/_common.py`).

## Golden fixtures (`test/fixtures/`, for the C++ gtests)

| file | content |
|---|---|
| `scorer_cases.json` | 52 base poses (check values, FoV/incidence/near edges, occlusion, yaw wrap, r = 0, random): per-marker r, β, cos θ, A, gates, occ, w; n_eff, s |
| `layer_cost_cases.json` | 62 cells of the room grid (0.05 m): s*, cost (all cost classes + a line through the band of id 1) |
| `yaw_dp_case.json` | Viterbi DP (spec §4 params) on (−2, −0.5) → (2.6, −0.5) → (2.6, 4.6): resampled points, ds, travel, chosen bins, yaws, total cost |

Every file has a `header` with all parameters and formulas. `test_fixtures.py`
regenerates them and compares with the committed files; after a deliberate
change of the oracle, rerun `make_fixtures.py` and commit the JSON.

## Run

```bash
colcon build --packages-select active_localization && source install/setup.bash
ros2 launch active_localization active_localization.launch.py scorer:=visible_count
# active mode also needs config/nav2_active_overrides.yaml applied to Nav2 (T-009)
```

## Tests

```bash
python3 -m pytest navigation/src/active_localization/test   # pure Python, no ROS
```

## Not verified in ROS yet

ROS nodes were written without a ROS environment (LEAD sandbox). Marked
`[unverified]` in code: action names, TF, FollowPath pre-emption,
MPPI `use_path_orientations` on Humble, StaticLayer scaling, marker normal axis
(for `visible_count`). ENG checks them in T-009.
