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
| `k_xx` | 2.5e-4 m²/m | variance of dx per metre travelled |
| `k_xr` | 1.0e-5 m²/rad | variance of dx per radian turned |
| `k_yy` | 2.5e-4 m²/m | variance of dy per metre travelled |
| `k_yr` | 1.0e-5 m²/rad | variance of dy per radian turned |
| `k_rr` | 5.0e-4 rad²/rad | variance of dyaw per radian turned |
| `k_rt` | 1.0e-5 rad²/m | variance of dyaw per metre travelled |
| `scale_x`, `scale_y` | 0.01 | systematic scale error on body x / y |
| `scale_yaw` | 0.0 | systematic scale error on yaw |
| `yaw_drift_per_m` | 0.01 rad/m | deterministic yaw drift per metre travelled |
| `min_motion` | 1e-6 | if trans and rot are both below this, the step is passed through without noise |
| `cov_floor` | 1e-6 | added to every reported diagonal variance |

The defaults are chosen as described below ("How the defaults were chosen");
the final values are a team decision. `use_sim_time` is set in the YAML (`true`).
QoS of input and output: reliable, keep last 50.

## Model

For each new GT pose the increment relative to the previous GT pose is taken
**in the body frame of the previous GT pose**, `(dx_b, dy_b, dyaw)` (no
ambiguity about the twist frame; works for omni motion, including pure strafe).
With `trans = hypot(dx_b, dy_b)`, `rot = |dyaw|`:

```
var_dx   = k_xx·trans + k_xr·rot
var_dy   = k_yy·trans + k_yr·rot
var_dyaw = k_rr·rot   + k_rt·trans
dx'   = dx_b·(1+scale_x) + N(0, var_dx)
dy'   = dy_b·(1+scale_y) + N(0, var_dy)
dyaw' = dyaw·(1+scale_yaw) + yaw_drift_per_m·trans + N(0, var_dyaw)
pose  = pose ⊕ (dx', dy', dyaw')      (SE(2) composition, yaw wrapped to [-π, π))
twist = (dx', dy', dyaw') / dt        (body frame; zeros if dt ≤ 0)
```

The *variance* (not σ) of each increment is proportional to the distance /
angle travelled, i.e. the random part is a random walk: splitting a path into
more steps does not change the accumulated variance, so the noise level does
**not depend on the `/odom_gt` rate** (checked by Monte-Carlo at 10 and 50 Hz,
see tests).

If trans and rot are both below `min_motion` the increment is passed through
unchanged: a standing robot does not drift. If the simulation time jumps back
by more than 1 s (sim reset), the node resets the model (and re-seeds the RNG).

### Covariance

- Pose: full 3×3 `(x, y, yaw)` covariance in the `odom` frame, first-order
  (EKF-style) propagation through the SE(2) composition, linearized at the
  current estimate and the noisy increment:

  ```
  P ← F P Fᵀ + G Q Gᵀ,   Q = diag(var_dx, var_dy, var_dyaw)
  F = [[1, 0, -s·dx' - c·dy'],     G = [[c, -s, 0],
       [0, 1,  c·dx' - s·dy'],          [s,  c, 0],
       [0, 0,  1            ]]          [0,  0, 1]]      (c, s = cos, sin of current yaw)
  ```

  This captures the growth of lateral error from yaw uncertainty and the
  x/y/yaw cross terms. `cov_floor` is added to the diagonal of the reported P.
  In the message the 3×3 block goes to indices 0,1,5 / 6,7,11 / 30,31,35 of
  the row-major 6×6; z, roll, pitch get 1e-9.
- Monte-Carlo check (tests, 500 seeds, scale/drift off): on a 10 m straight
  and on a 2 m-radius circle, at 10 and 50 Hz, the empirical covariance of the
  final pose matches the reported P within ±20 % (diagonal: relative error;
  off-diagonal: error ≤ 0.2·σᵢσⱼ), and both are the same for 10 and 50 Hz
  within ±20 %.
- **Not modelled by the covariance:** the deterministic errors `scale_x`,
  `scale_y`, `scale_yaw`, `yaw_drift_per_m`. They are a bias, not noise, so
  with them enabled the real error is larger than the reported P (e.g. with
  the defaults the yaw drift alone gives ≈0.1 rad and ≈0.5 m lateral offset
  over 10 m, vs random σ_y ≈ 0.076 m).
- Twist: diagonal `(var_dx, var_dy, var_dyaw) / dt² + cov_floor` (velocity
  noise of a random walk is white; this value scales as 1/dt, i.e. it does
  depend on the rate, as it should).

### How the defaults were chosen

Target from LEAD: random σ_y ≈ 0.05–0.1 m after a 10 m straight.

- `k_xx = k_yy = 2.5e-4 m²/m`: direct translation noise, σ = √(2.5e-4·10) =
  0.05 m over 10 m (0.5 % of distance).
- `k_rt = 1e-5 rad²/m`: yaw noise from driving, σ_yaw = √(1e-5·10) = 0.01 rad
  (0.6°) over 10 m. Yaw errors integrate into lateral error:
  var_y ≈ k_yy·L + k_rt·L³/3 = 2.5e-3 + 3.3e-3 ⇒ **σ_y ≈ 0.076 m over 10 m**
  (reported P and Monte-Carlo agree).
- `k_rr = 5e-4 rad²/rad`: σ ≈ 0.056 rad (3.2°) per full turn in place.
- `k_xr = k_yr = 1e-5 m²/rad`: σ ≈ 8 mm per full turn (small translation
  slip while rotating).
- Deterministic part kept from T-004a: `scale_x = scale_y = 0.01`,
  `yaw_drift_per_m = 0.01 rad/m`. With these on, the bias dominates the random
  part over 10 m; switch them to 0 to test with noise only.
