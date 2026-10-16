# al_odom_noise

Turns the ideal odometry of `gazebo_ros_planar_move` (taken from the true Gazebo
pose, so it never drifts) into realistic drifting wheel odometry, so that
RTAB-Map localization has something to correct.

```
planar_move ──/odom_gt──▶ odom_noise_node ──/odom──▶ RTAB-Map, Nav2
                                         └─TF odom→base_link
```

Prerequisite (done by ENG, not in this package): the plugin publishes to
`/odom_gt` and has `publish_odom_tf: false`; otherwise two TF publishers of
`odom→base_link` will fight.

## Run

```bash
colcon build --packages-select al_odom_noise
ros2 launch al_odom_noise odom_noise.launch.py seed:=42
# own parameter file:
ros2 launch al_odom_noise odom_noise.launch.py params_file:=/path/to.yaml seed:=7
```

Tests (pure Python, no ROS needed):

```bash
cd navigation/src/al_odom_noise && python3 -m pytest test/ -q
```

## Parameters (`config/odom_noise.yaml`)

| name | default | meaning |
|---|---|---|
| `input_topic` | `/odom_gt` | GT odometry (nav_msgs/Odometry) |
| `output_topic` | `/odom` | noisy odometry |
| `odom_frame` / `base_frame` | `odom` / `base_link` | frame_id / child_frame_id of output and TF |
| `publish_tf` | `true` | broadcast TF `odom→base_link` |
| `seed` | `42` | RNG seed (`numpy.random.default_rng`), launch arg `seed` |
| `enabled` | `true` | `false` → output equals GT exactly |
| `init_from_gt` | `true` | `true`: odometry starts at the first GT pose; `false`: at (0,0,0) |
| `a1` | 0.05 | trans σ per metre of translation |
| `a2` | 0.01 | trans σ per radian of rotation |
| `a3` | 0.05 | yaw σ per radian of rotation |
| `a4` | 0.01 | yaw σ per metre of translation |
| `scale_x`, `scale_y` | 0.01 | systematic scale error on body x / y |
| `scale_yaw` | 0.0 | systematic scale error on yaw |
| `yaw_drift_per_m` | 0.01 rad/m | deterministic yaw drift per metre travelled |
| `min_motion` | 1e-6 | if trans and rot are both below this, the step is passed through without noise |
| `cov_floor` | 1e-6 | added to every reported variance |

The defaults are only a plausible order of magnitude for wheel odometry; the
final values are a team decision. `use_sim_time` is set in the YAML (`true`).
QoS of input and output: reliable, keep last 50.

## Model

For each new GT pose the increment relative to the previous GT pose is taken
**in the body frame of the previous GT pose**, `(dx_b, dy_b, dyaw)` (no
ambiguity about the twist frame; works for omni motion, including pure strafe).
With `trans = hypot(dx_b, dy_b)`, `rot = |dyaw|`:

```
σt    = a1·trans + a2·rot
σr    = a3·rot + a4·trans
dx'   = dx_b·(1+scale_x) + N(0, σt²)
dy'   = dy_b·(1+scale_y) + N(0, σt²)
dyaw' = dyaw·(1+scale_yaw) + yaw_drift_per_m·trans + N(0, σr²)
pose  = pose ⊕ (dx', dy', dyaw')      (SE(2) composition, yaw wrapped to [-π, π))
twist = (dx', dy', dyaw') / dt        (body frame; zeros if dt ≤ 0)
```

If the simulation time jumps back by more than 1 s (sim reset), the node resets
the model (and re-seeds the RNG).

### Covariance (approximation)

- Pose: diagonal only. Per step `var_x += σt²`, `var_y += σt²`,
  `var_yaw += σr²`; reported value is the sum + `cov_floor`. Since the dx/dy
  noise is isotropic, rotating it into the odom frame does not change the
  diagonal. **Neglected:** propagation of yaw uncertainty into x/y through the
  composition Jacobian and all cross terms. In practice lateral error grows
  faster than the reported σ (measured on a 10 m straight at 50 Hz, random part
  only, 200 seeds: std y ≈ 0.025 m vs reported σ ≈ 0.016 m).
- The deterministic parts (`scale_*`, `yaw_drift_per_m`) are **not** in the
  covariance, so the reported covariance is optimistic w.r.t. them.
- Twist: `(σt², σt², σr²) / dt² + cov_floor`.
- z, roll, pitch: 1e-9. Indices in the 6×6 row-major arrays: x=0, y=7, yaw=35.

### Note: noise level depends on the input rate

σ is proportional to the step length, so after N steps over a distance L the
random error is ~ a1·L/√N: the same parameters give less random noise at a
higher input rate (≈1.7 cm over 10 m at 50 Hz vs ≈3.8 cm at 10 Hz with the
defaults). At 50 Hz the drift is dominated by the deterministic terms
(`yaw_drift_per_m`, `scale_*`). Tune parameters for the actual `/odom_gt` rate.
