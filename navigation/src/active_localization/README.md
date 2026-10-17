# active_localization — localization-aware navigation on top of Nav2

Skeleton (T-008). Method-agnostic: the localizability **scorer** is a plug-in;
the method chosen at GATE-1 (ADR-001) is added as one more scorer in T-009.
Architecture and integration options: `_team/knowledge/ARCH.md`.

## Layout

| module | ROS? | what |
|---|---|---|
| `geometry.py` | no | SE(2) helpers, path resampling |
| `landmarks.py` | no | reads `rtabmap_landmarks/config/landmarks.yaml`, camera model |
| `scorers/` | no | `Scorer` interface + registry; reference scorers `null`, `visible_count` |
| `yaw_planner.py` | no | **yaw lever**: Viterbi DP over headings along a fixed path |
| `cost_field.py` | no | **route lever**: best-over-yaw score → occupancy cost grid |
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
MPPI `use_path_orientations` on Humble, StaticLayer scaling, marker normal axis,
camera offset. ENG checks them in T-009.
