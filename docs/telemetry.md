# Basic Flight Data Records

position, velocity, acceleration, motor speeds, force, torque, and position error are all recorded. After each simulation run, the following files are saved automatically under `runs/<MMDD-HHMM>_<experiment>_<backend>[_video]/`:

- `basic.csv`: each row is one completed physics step. The column names include the component and the unit, and the file can be plotted with Excel, MATLAB, or Python.
- `telemetry.jsonl`: keeps the full pre-step/post-step states, control inputs, actual actuator states, and the basic data. It also records the synthesized sensor readings `sensors` and the state estimation `estimation` for each step (see "Sensor readings and state estimation").
- `telemetry_schema.json`: the mapping between CSV columns and JSON fields, with units, reference frames, sampling times, and source descriptions.
- `config.json` / `metadata.json`: the actual config (with `extends` and `--set` already expanded), backend description, USD layer hashes (isaaclab), vehicle properties, model limits, the mission phase schedule `mission_phases`, the git commit, and the command-line `provenance`.
- `metrics.json`: overall and per-phase metrics computed from `telemetry.jsonl` (`isaac_drone/analysis/metrics.py`), shared by `summarize`, `compare`, and `sweep`.
- `README.md`: the run card, generated from the files above (`isaac_drone/analysis/catalog.py`). It contains the command line, code version, full physics settings, videos, results, per-phase metrics, and all plots. `python -m isaac_drone runs` lists all runs using the same information.

Both backends (`isaaclab` and `synthetic`) write the same format. The synthetic backend has no PhysX solver acceleration, so the corresponding fields are empty.

## Basic fields

The fields below are in the `basic` object of each JSON record. The CSV expands vectors into components; see `telemetry_schema.json` for the exact column names.

| Requirement | JSON field | Unit and meaning |
|---|---|---|
| Position | `position_w_m` | Position of the vehicle center of mass in the world frame, m |
| Velocity | `velocity_w_m_s` | Velocity of the vehicle center of mass in the world frame, m/s |
| Acceleration | `acceleration_w_m_s2` | World-frame average motion acceleration over the current physics step, m/s² |
| Solver acceleration | `acceleration_native_w_m_s2` | Mass-weighted value of the PhysX center-of-mass accelerations of the rigid bodies, recorded when available, m/s² |
| Motor speeds | `motor_speed_rps` / `motor_speed_rpm` / `motor_speed_rad_s` | Speed magnitudes of the four rotors: rps, rpm, rad/s |
| Force | `force_command_b_n` / `force_motor_b_n` / `force_submitted_b_n` / `force_net_w_n` | Control demand, force produced by the motors, force submitted to the physics engine, and total net force derived from motion, in N |
| Torque | `torque_command_b_nm` / `torque_motor_b_nm` / `torque_submitted_b_nm` / `torque_net_w_nm` | Torque about the vehicle center of mass at the same stages as above, in N·m |
| Position error | `position_error_w_m` / `position_error_norm_m` | Three-axis position error, target minus actual, and the three-dimensional Euclidean distance, m |

The target position/velocity/acceleration, quaternion, angular velocity/angular acceleration, allocated force/torque, disturbance force/torque, and the commanded and actual thrust of each rotor are saved as well.

## Ideal value, actual value, and error

Every tracked quantity is recorded as an ideal value, an actual value, and an error. The error is always **ideal value − actual value**, in the same reference frame and unit as the pair it belongs to. The norm columns give the three-dimensional size of the error.

| Quantity | Ideal value | Actual value | Error |
|---|---|---|---|
| Position | `target_position_w_m` (trajectory target) | `position_w_m` | `position_error_w_m`, `position_error_norm_m` |
| Velocity | `target_velocity_w_m_s` | `velocity_w_m_s` | `velocity_error_w_m_s`, `velocity_error_norm_m_s` |
| Acceleration | `target_acceleration_w_m_s2` | `acceleration_w_m_s2` (velocity difference) | `acceleration_error_w_m_s2`, `acceleration_error_norm_m_s2` |
| Attitude | `target_quaternion_wxyz`, `target_attitude_rpy_rad` (desired attitude R_d from the controller) | `quaternion_wxyz`, `attitude_rpy_rad` | `attitude_error_rpy_rad` (per-angle difference, wrapped to ±π), `attitude_error_angle_rad` (single rotation angle from actual to desired) |
| Angular velocity | `target_angular_velocity_b_rad_s` (desired angular velocity from the controller, rotated into the actual body frame) | `angular_velocity_b_rad_s` | `angular_velocity_error_b_rad_s`, `angular_velocity_error_norm_rad_s` |
| Motor speed | `motor_speed_command_rps` / `_rpm` (command sent to the motor model) | `motor_speed_rps` / `_rpm` | `motor_speed_error_rps` / `_rpm` |
| Rotor thrust | `motor_thrust_command_n` | `motor_thrust_applied_n` | `motor_thrust_error_n` |
| Force | `force_command_b_n` (control demand) | `force_motor_b_n` (motor output) | `force_error_b_n`, `force_error_norm_n` |
| Torque | `torque_command_b_nm` | `torque_motor_b_nm` | `torque_error_b_nm`, `torque_error_norm_nm` |

Notes:

- Attitude is written as Z-Y-X Euler angles (roll, pitch, yaw) so that it is easy to read, and the quaternion is still the unambiguous original record. The per-angle error is not a rotation vector; to judge the size of the attitude error, use `attitude_error_angle_rad`.
- Force/torque error = control demand − motor output. It includes both allocation saturation and motor response lag. The intermediate values after allocation are still recorded in `force_allocated_b_n` / `torque_allocated_b_nm`.
- The net force derived from motion, `force_net_w_n`, is a result. Its ideal value is mass × target acceleration, and its error equals mass × acceleration error, so it is not recorded a second time.
- The target stays the same within a control period (by default physics runs at 200 Hz and control at 100 Hz), while the actual value changes at every physics step. So the error shows a small sawtooth between neighboring steps. This comes from holding the target and is not a recording error.
- When there is no source for the ideal value (for example, old logs or a custom backend), the corresponding ideal-value and error columns are left empty instead of being filled with 0.

## Wind and disturbances

These columns come after the existing columns, and the positions of the existing columns do not change. When the corresponding model is not enabled, the whole column is left empty. Each column is sampled at the start of the physics step and is the value that acts during that step.

| Field | Meaning |
|---|---|
| `wind_w_m_s` / `wind_mean_w_m_s` / `wind_turbulence_w_m_s` / `wind_speed_m_s` | World-frame wind velocity at the center of mass, its deterministic part (mean + gradient + gusts), its OU turbulence part, and the wind speed; 0 outside the wind field time window |
| `rotor_axial_wind_m_s` | Wind velocity component along the rotor axis at each rotor (updraft is positive) |
| `rotor_speed_offset_rps` / `_rpm` | Equivalent rotor speed offset of each rotor converted from the wind, added after the motor model |
| `rotor_speed_effective_rps` / `_rpm` | Motor speed + offset, which is the equivalent rotor speed for the actual thrust when there is wind; it is not the motor state |
| `force_wind_drag_b_n` | Wind drag in the rotor disk plane (body frame, applied at the center of mass), already included in `force_disturbance_b_n` |
| `force_ou_w_n` / `force_ou_mean_w_n` | Actually applied value and mean of the old OU random force at the center of mass (world frame); 0 outside the time window, already included in the total disturbance force |
| `torque_ou_b_nm` / `torque_ou_mean_b_nm` | Actually applied value and mean of the OU random torque (body frame) |

When wind acts on the rotors, `motor_thrust_applied_n` is the thrust actually applied to the blades, kf·(RPS + offset)², so `motor_thrust_error_n`, `force_error_b_n`, and `torque_error_b_nm` also include the effect of wind on the rotors; `motor_speed_rps` is still the motor state itself.

## Sensor readings and state estimation (JSONL)

Each step record has two more top-level fields. They are written only to `telemetry.jsonl` and not to `basic.csv` (the basic CSV still keeps only the motion and dynamics data). For the configuration, see the "Sensors and state estimation" section of docs/configuration.md; for the models and formulas, see Section 5 of docs/architecture.md.

### `sensors`

When `sensors.enabled: false`, this field is `null`. Otherwise it contains four keys, `imu`, `magnetometer`, `barometer`, and `gps`, and each key is `null` when no reading is published in this step. The readings are synthesized from the simulation ground truth at the start/end of this step. They are generated at the end of the step and passed to the state provider (`update`) before the controller reads the state in the next step, so the estimator only ever sees past data.

| Sensor | Reading fields | Ground truth recorded with it |
|---|---|---|
| `imu` (200 Hz by default) | `accelerometer_m_s2` (specific force in the sensor frame, about (0, 0, +9.81) at rest), `gyroscope_rad_s` (body angular velocity at the end of the step) | `true_specific_force_s_m_s2`, `true_angular_velocity_s_rad_s` |
| `magnetometer` (50 Hz) | `magnetic_field_ut` (sensor frame, µT) | `true_magnetic_field_s_ut` (geomagnetic field + motor interference), `motor_interference_s_ut` |
| `barometer` (50 Hz) | `altitude_m` (world z of the mounting point) | `true_altitude_m` |
| `gps` (10 Hz, 0.12 s delay) | `position_w_m`, `velocity_w_m_s` (local ENU position/velocity of the antenna, not latitude and longitude) | `true_position_w_m`, `true_velocity_w_m_s` |

Each reading has its own `time_s`, which is the time the reading describes, that is, the end of the step in which it was sampled. A sensor with delay (only GPS by default) appears in the record `delay_s/dt` steps after sampling. At that point the reading's `time_s` is one delay earlier than the record's `post_step_state.time_s`, so plots and estimators should both use the reading's own `time_s`. The `true_*` fields are noise-free ideal values used to draw the "reading vs ground truth" plots and to compute the sensor error; the estimator should not read them.

### `estimation`

When `estimation.mode: truth` and no state provider is injected, this field is `null`. Otherwise it contains the following fields:

| Field | Meaning |
|---|---|
| `kind` / `mode` / `used_by_controller` | Estimator type (only `passthrough` for now), mode (`shadow`, `closed_loop`, or `injected` for an injected provider), and whether the controller used this state |
| `state` | Estimated state at the end of the step (`time_s`, `position_w`, `quaternion_wxyz`, `linear_velocity_w`, `angular_velocity_b`), at the same time as `post_step_state` |
| `error` | **Estimation error = ground truth − estimate**: `position_w_m`, `velocity_w_m_s`, `angular_velocity_b_rad_s` and their norms; the attitude error is split into the rotation vector `attitude_error_rad` (Log(R_estᵀ R_true)), its angle `attitude_error_angle_rad`, the tilt error `tilt_error_rad` (the angle between the two body Z axes), and the yaw error `yaw_error_rad` |
| `control_error` | **Control error = reference − estimate**: `position_w_m`, `velocity_w_m_s` and their norms; the reference is the target held in this step |
| `covariance_diag` / `nis` | Covariance diagonal and NIS given by the estimator; `null` for `passthrough` |

The three errors are related as follows: `basic.position_error_w_m` is the **total error = reference − ground truth**, so

$$
\text{total error} = \text{control error} - \text{estimation error}
$$

`metrics.json` gives, for each phase, `estimation_position_error_rms_m` / `_max_m`, `estimation_velocity_error_rms_m_s`, `estimation_tilt_error_rms_deg`, `estimation_yaw_error_rms_deg`, `estimation_angular_velocity_error_rms_rad_s`, and `control_position_error_rms_m`; they are `null` when there is no estimation record. `summarize` adds one summary line for the estimation error after the table, and the metrics table of the run card has these extra rows only when there is an estimation record. The top-level `state_source` says which state the controller used: `simulation_truth`, `estimate:<kind>`, or `injected_state_provider`.

## Plots

After each flight, English PNG plots are generated automatically under `runs/<run directory>/plots/`. The CSV/JSONL files are still the raw data, and the plots are only views derived from them. The second line of the title is the run directory name, and the third line is the physics settings (only the items that differ from the ideal plant are listed, such as `OU force (1,0,0)±0.5 N τ 1 s; mass ×1.2`), so a plot taken out on its own still shows which run it belongs to. The following commands can also regenerate the plots for any run (including a directory copied back to the local machine); they only need NumPy and matplotlib:

```bash
python -m isaac_drone plot                  # the latest run
python -m isaac_drone plot runs/<run directory>   # a specific run
```

| File | Content |
|---|---|
| `trajectory_3d.png` | Three-dimensional trajectory of the center of mass and top view, actual vs target |
| `position.png` / `velocity.png` / `acceleration.png` | Three axes, actual vs target; the last row shows the error and the error norm (the legend gives the maximum and RMS); the acceleration plot also shows the PhysX solver value |
| `attitude.png` / `angular_velocity.png` | Roll/pitch/yaw (deg) and body angular velocity (deg/s), actual vs desired; the last row shows the error |
| `motor_speed.png` / `rotor_thrust.png` | Speeds of the four motors (rpm) and thrust of each rotor, actual vs commanded; the last row shows the error; when wind acts on the rotors, the speed plot also shows the equivalent rotor speed |
| `force.png` / `torque.png` | Body-frame force/torque, motor output vs control demand; the last row shows the error |
| `wind.png` | Three axes of the world-frame wind velocity at the center of mass: actual wind velocity vs mean + gusts; the last row shows the OU turbulence part and its norm (maximum, RMS). Drawn only for runs with the wind field enabled |
| `rotor_wind.png` | Equivalent rotor speed of each rotor vs motor speed; the last row shows the rotor speed offset caused by wind. Drawn only for runs with `rotor_wind.axial` enabled |
| `wind_drag.png` | Wind drag in the rotor disk plane (body frame). Drawn only for runs with `rotor_wind.inplane_drag` enabled |
| `ou_force.png` / `ou_torque.png` | The old OU random force at the center of mass (world frame) / torque (body frame): actually applied value vs mean; the last row shows the OU random part. Drawn only for runs with `ou_wrench` enabled and this term not all 0 |
| `sensors_accelerometer.png` / `sensors_gyroscope.png` | IMU readings vs true specific force / true angular velocity (sensor frame); the last row shows ground truth − reading. The horizontal axis is the reading's own time; drawn only for runs with sensors enabled |
| `sensors_magnetometer.png` / `sensors_barometer.png` | Magnetometer reading vs true magnetic field (geomagnetic field + motor interference); barometric altitude vs true altitude of the mounting point |
| `sensors_gps_position.png` / `sensors_gps_velocity.png` | GPS antenna position/velocity readings vs ground truth (local ENU), aligned by reading time, so the delay does not show on the horizontal axis |
| `estimation_position.png` / `estimation_velocity.png` / `estimation_attitude.png` / `estimation_angular_velocity.png` | Estimate vs ground truth; the last row shows the estimation error (ground truth − estimate); in the attitude plot the error norm is the rotation angle. Drawn only for `shadow` / `closed_loop` runs (or runs with an injected state provider) |

The vertical dashed lines in the plots are the boundaries between mission phases (taken from `mission_phases` in `metadata.json`, for example spin_up / takeoff / helix / hold; old run directories use the earlier delay / takeoff / helix / hold boundaries). Error columns missing from old logs are computed as "ideal − actual" when plotting, and for quantities without an ideal value only the actual value is drawn.

## Time alignment

Each `basic` record has four time fields:

- `step_start_time_s`: the start time of this physics step.
- `sample_time_s`: the end time of this physics step; the actual position, attitude, and velocity are taken at this time.
- `interval_dt_s`: the difference between the two, which is the physics step length actually used.
- `reference_sample_time_s`: the time at which the target used in this step was last sampled by the control loop.

The control rate can be lower than the physics simulation rate. For example, with the default 200 Hz physics and 100 Hz control, the second physics step keeps using the control target sampled in the first physics step. The recorded error is defined as:

```text
position_error_w_m = target position held in this physics step - actual center-of-mass position after the physics step ends
position_error_norm_m = sqrt(error_x² + error_y² + error_z²)
```

An error defined this way reflects the target that was actually used. The trajectory generator is not called again to produce a future target that was never passed to the controller. If a future helix task also needs "the error of the continuous analytic trajectory at the current time", that reference should be added as a separate quantity so that the two definitions stay apart.

To stay compatible with the original logs, the top-level JSON `time_s` still means the start of the step, `state` is still the start-of-step state used for control, and `post_step_state` is the simulation ground truth at the end of the step. When plotting `basic` data, use its own `sample_time_s`.

## Where the acceleration comes from

The acceleration in the main record is computed at every physics step with the formula below:

```text
acceleration_w = (velocity_w_end - velocity_w_start) / interval_dt
```

The calculation uses the actual simulated center-of-mass velocity at both ends of this step. It does not use the target acceleration, the estimator output, or the velocity last written to the file. So even if the log downsampling rate changes, the acceleration still uses the correct time interval. The first record after a reset uses the reset state and the state at the end of the first physics step, so no extra zero has to be added.

This is the **interval-averaged motion acceleration**. It includes the effect of gravity on the motion and is not the IMU specific force. The accelerometer reading is synthesized separately by the sensor layer (`sensors.imu`): the specific force `f_b = R_pairᵀ(Δv/Δt − g_w)` plus the lever arm terms, passed through the error model; see "Sensor readings and state estimation" and Section 5 of docs/architecture.md.

`acceleration_native_w_m_s2` separately records the acceleration provided by PhysX, mass-weighted over the rigid bodies to the vehicle center of mass. It is recorded only when the solver data is available and the corresponding physics step has finished. When it is uninitialized or unavailable, it is recorded as null and is empty in the CSV; it is not recorded as zero. `acceleration_native_source` gives the source. The two accelerations are sampled and numerically defined in different ways, so they may not be exactly equal at every point.

The world-frame angular acceleration is computed by differencing the world-frame angular velocity; if the output is in the body frame, this world-frame average is rotated into the body frame at the end of the step. The body frame has a different attitude at the two times, so body-frame components cannot be subtracted directly.

## Where the motor speed comes from

In the USD of this vehicle the rotors are attached with fixed joints, so there is no real rotary joint encoder that can be read directly. The native thruster uses internally:

```text
thrust_state = sampled_kf × rps²
```

The log reads the integrated RPS motor state directly and then converts it to rpm and rad/s. The actual thrust is each motor's **own coefficient from the current sampling** multiplied by the actual RPS²; the target rotor speed and the actual rotor speed are recorded separately.

The rotor order is fixed as back-left BL, back-right BR, front-left FL, front-right FR. The recorded values are non-negative magnitudes; the sign of the reaction torque in the config does not represent a measured mechanical rotation direction.

`motor_speed_source` says that this rotor speed is a state computed with the native integration formula, not data from a real encoder. `motor_thrust_command_n` and `motor_thrust_applied_n` are kept separately so that motor response lag and saturation are easy to see. When the motor model is off (the default, `backend.motor_model_enabled: false`), the rotor speed is set directly to the limited command at every physics step, `motor_speed_source` is `direct_rps_state_set_to_limited_command_each_step_motor_model_off`, and the actual value differs from the command only by the clipping at the operating limits.

## Stages of force and torque

| Field prefix | What it means |
|---|---|
| `force_command` / `torque_command` | The vehicle force/torque the controller wants |
| `force_allocated` / `torque_allocated` | The target after allocation with the thrust limits of the four rotors taken into account |
| `force_motor` / `torque_motor` | The vehicle force/torque from the thrust actually output after the native motor dynamics update |
| `force_disturbance` / `torque_disturbance` | Additional force/torque produced by the explicit disturbance and aerodynamic models |
| `force_submitted` / `torque_submitted` | Sum, about the vehicle center of mass, of the per-rotor submitted values and the explicit additional effects; it is only a diagnostic total and is not submitted as a vehicle-level control quantity |
| `force_net` / `torque_net` | Interval-averaged total net force/torque derived from the actual motion in this physics step |

The submitted values **do not include** the gravity, collision contact, and native damping that PhysX computes on its own, so they are not the sum of all physical effects. The force/torque commands and submitted values are expressed in the body frame at the start of the physics step; the net values derived from motion are expressed in the world frame.

For the current rigid-body assembly with fixed joints, the net effect is computed with the formulas below:

```text
force_net_world = mass × Δ(velocity_world) / Δt
angular_momentum_world = R_body_to_world × inertia_com_body × angular_velocity_body
torque_net_world = Δ(angular_momentum_world) / Δt
```

The angular momentum uses the full inertia tensor and the actual attitude at both end points, so the rotational coupling is kept. These quantities are derived back from the motion and are not measurements from an independent force sensor, so they cannot serve as independent evidence for checking whether the same motion data is correct.

## Recording rate and what has been verified

Currently `logging.every_n_steps: 1`, so by default one record is written per physics step. With `simulation.dt: 0.005`, this is 200 records per second of simulation time. If this interval is increased later, only the write rate goes down; the calculation still runs at every physics step and does not turn into a difference across several log rows. JSON end/exception events are not mixed into the CSV data rows.

A missing reading is written as null or an empty cell. A non-finite number or a wrong vector length raises an error and is not filled with zero. The field sources, units, and time descriptions are also written to a machine-readable schema, so that the data is easy to pass to later analysis programs.

The current verification covers the numerical calculations, the rotor speed state and unit conversion, time pairing, reset, coordinate frames, and the JSON/CSV output format. The development machine does not have the full Isaac Sim stack, so real logs and flight behavior still need to be verified by actually running on the server.

## Diagnostics for the mission and controller

`motor_command_rps` is the command to the four motors in the current step (after the spin-up envelope and the operating limits), `backend.commanded_motor_speed_rps` is the target accepted by the actuator, and `basic.motor_speed_rps` is the actual rotor speed after the dynamic response (the same as the accepted target when the motor model is off). `startup_fraction` is the spin-up envelope in 0..1, and `motor_command_thrust_n` is the static thrust target for the accepted rotor speed; `allocated_thrust_n` keeps the closed-loop allocation result before the startup scaling.

`external_wrench_b` is the total disturbance force/torque from effects (body frame), and `gyroscopic_torque_b_nm` is the rotor gyroscopic torque, recorded separately; the sum of the two is the additional force/torque submitted to the backend. `setpoint.jerk_w` and `setpoint.snap_w` record the analytic reference derivatives. `controller` records the controller type `kind` and its own details: for `geometric`, `force_derivative_source` says whether the path is analytic reference/numerical feedback or full numerical differencing after limiting. The "ideal values" of attitude/angular velocity are the desired attitude and desired angular velocity given by the controller. `mission_phase` is `spin_up` (spin-up on the ground) or the name of a trajectory segment; for example, a helix mission has `spin_up / takeoff / helix / hold`, and a hold mission has `hold`. When `completion` is configured, `mission` stores the post-step ground-truth error, the continuous time within tolerance, and the success and timeout flags; otherwise it is null. These diagnostics are in the JSONL; the basic CSV still keeps the motion and dynamics data fields.
