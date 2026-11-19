# Experiment Configuration and Extension (schema_version 2)

An experiment consists of one YAML file (which can inherit from another) and optional command-line overrides. All configs are strictly validated before the flight: unknown fields, duplicate keys, NaN/Inf, wrong vector lengths, and a boolean written where a number is expected all raise an error, and default values are not substituted automatically.

## 1. File organization and inheritance

| File | Contents |
|---|---|
| `configs/arl_robot_1.yaml` | Base config: simulation, vehicle, thrusters, scene, logging, recording, default geometric controller, hover in place from an airborne initial state (hold) |
| `configs/helix.yaml` | Inherits the base config: ground takeoff from zero rotor speed + minimum-time helix, completion criterion |
| `configs/helix_moderate.yaml` | Inherits helix: a slower version with the same geometry and a fixed duration (peak about 3.3 m/s), which leaves margin for feedback |
| `configs/helix_rotor_wind.yaml` | Inherits helix: turns off the center-of-mass OU force and applies a world-frame wind field to the rotors instead (an OU example that is random only along the x axis; the coefficients are nominal values with their sources noted) |

A child file inherits from a parent file with `extends: <relative path>` and only needs to write the differences:

- Mappings are merged level by level, while lists and scalars are replaced as a whole.
- If the `kind` of a section in the child file differs from the parent (for example `trajectory.kind: hold → helix`), the **whole section is replaced** and the parameters of the old kind are not inherited.
- Multi-level inheritance is supported. Circular inheritance raises an error.

For a new experiment, the recommended way is to copy the closest file, rename it, and keep only the fields that need to change.

## 2. Command-line overrides

All commands accept `--set key.path=value` (can be repeated), and the value is parsed as YAML:

```bash
python -m isaac_drone run --backend synthetic --config configs/helix.yaml \
    --set controller.position_kp=[10,10,6] --set trajectory.turns=3 --set simulation.seed=1
# Replace a whole section: switch to hold and turn off the completion criterion
python -m isaac_drone run --backend synthetic --config configs/helix.yaml \
    --set 'trajectory={kind: hold}' --set completion=null
```

The full config after overrides is written to `config.json` in the run directory. The command line itself is recorded in `provenance.argv` of `metadata.json`, together with the git commit and whether there were uncommitted changes.

## 3. What each config section means

| Section | Purpose |
|---|---|
| `simulation` | Physics step `dt`, control decimation, render interval, device, gravity, random seed, duration, native SimulationCfg overrides |
| `recording` | Optional multi-view video (isaaclab backend only) |
| `vehicle` | Asset, rotor directions and their source, rotor inertia `rotor_inertia_kg_m2` (gyroscopic torque; default null, off), motor response model switch `motor_model_enabled` (default false: the rotor speed equals the command within the same step, with no response delay; true: native first-order lag using `tau_inc_range`/`tau_dec_range`), thruster parameters, initial state, `launch` (ground takeoff, ground clearance, spin-up duration `spin_up_s`), native/USD overrides |
| `limits` | Flight envelope **shared** by the controller and the trajectory planner: `max_tilt_rad`, `max_yaw_rate_rad_s`, `max_acceleration_m_s2` |
| `controller` | `kind` + the parameters of that controller (see Section 4) |
| `allocation` | Weights and regularization of the bounded weighted allocation |
| `trajectory` | `kind` + the parameters of that trajectory (see Section 5) |
| `completion` | Completion criterion for hovering at the end point; `null` means no criterion (the run succeeds once all steps finish) |
| `effects` | Constant external force/torque, `ou_wrench` random disturbance (force in the world frame, torque in the body frame; mean + per-axis standard deviation + correlation time), wind field `wind`, wind acting on the rotors `rotor_wind`, aerodynamic drag. `ou_wrench` is on by default (force mean [1,0,0] N, standard deviation 0.5 N on each axis, τ=1 s; in `helix.yaml` it takes effect from 2 s, when spin-up ends), and the others are off by default. The wind field and wind acting on the rotors are described below |
| `model_mismatch` | Ratio of the plant's actual value to the controller's nominal value: `mass_ratio`, `inertia_ratio`, `thrust_coefficient_ratio`, `reaction_torque_ratio`; all 1 means no mismatch, and the default is all 1 (off) |
| `scene` / `logging` | Ground and lighting; log directory, the experiment name `run_name` used in the run directory name (`null` = the YAML file name), logging interval, flush interval |
| `sensors` (optional) | IMU, magnetometer, barometer, and GPS readings synthesized from ground truth, with their error models (see below); when omitted, the default section in `configs/arl_robot_1.yaml` is used (all on). Turning sensors on or off does not change the flight result |
| `estimation` (optional) | Which state the controller uses: `mode` is `truth` (default, ground truth), `shadow` (the estimator runs online and is recorded, while the controller still uses ground truth), or `closed_loop` (the controller uses the estimate); `kind` currently only has `passthrough` (returns ground truth directly, used to check the pipeline), and the filter is left for the next stage |

`vehicle.launch.spin_up_s`: during spin-up, the reference stays at the start point and the motor command is multiplied by a smooth 0→1 envelope, and the trajectory starts only when spin-up ends. For ground takeoff this value must be > 0. For ground takeoff, the trajectory is anchored to the backend ground truth (the physical placement position); for an airborne start, it is anchored to the state the controller sees (which can be the estimator output).

### Wind field and wind acting on the rotors (`effects.wind`, `effects.rotor_wind`)

`wind` generates the wind velocity in the world frame (in m/s, describing the direction the air moves toward). `rotor_wind` then splits this wind velocity into two parts in the body frame and applies them to the vehicle. Both are off by default.

The wind field is the sum of the mean wind `velocity_w_m_s`, the gusts `gusts`, and the OU turbulence `turbulence`. A linear gradient `spatial_gradient_per_s` can also be added:

- **Time window** `start_time_s` / `end_time_s`: outside the window the air is still, and at `start_time_s` the wind appears at full strength. The OU process takes its value from the stationary distribution at reset and evolves at every physics step after that, so the turbulence is already at full strength when the window opens and does not grow gradually from 0. `helix.yaml` sets `start_time_s` to 2 s, which is when spin-up ends.
- **Turbulence** `turbulence.mode`:
  - `axes` (default): each world axis uses an independent OU process, and `time_constant_s` and `stationary_std_w_m_s` each take 3 numbers. To make only one axis random, set the standard deviation of the other two axes to 0.
  - `direction`: a single OU process is used along `direction_w` (normalized automatically), and `time_constant_s` and `stationary_std_w_m_s` each take 1 number. `[1, 0, 0]`, `[0, 1, 0]`, and `[0, 0, 1]` mean a single axis, and `[1, 1, 0]` means 45° in the horizontal plane.
- **Gusts** `gusts`: each item needs `start_time_s`, `duration_s`, `delta_velocity_w_m_s`, and `shape`. `one_minus_cosine` (default) rises smoothly and then falls back. `step` appears instantly at `start_time_s` and disappears instantly when the duration ends, and `duration_s: null` means it lasts until the end.

When wind acts on the rotors (`rotor_wind`, which requires `wind.enabled` to be on first), it is split into two parts in the body frame, and each part has its own switch:

- **Along the rotor axis** (`axial`): the wind at each rotor is rotated into the body frame and its component along the rotor axis `u_axial,i` is taken (updraft is positive). It is converted into an equivalent rotor speed offset `δΩ_i = c/2 · Ω_hover,i · u_axial,i`, which is added after the motor model: thrust and reaction torque are computed from `k_f·(Ω_i + δΩ_i)²`, and the motor state itself does not change. `c` is `thrust_gain_per_m_s`, the fractional change in thrust per rotor for each 1 m/s of axial wind at hover, and BEMT linearization gives `c = κ / v_h`. `Ω_hover,i` is the speed of rotor i when the four rotors share the weight equally. The offset does not depend on the current rotor speed, so the thrust change is proportional to rotor speed, not to thrust.
- **In the rotor disk plane** (`inplane_drag`): rotor speed cannot produce a horizontal force, so this part is applied at the center of mass as a drag `k · u_inplane` in the direction of the wind. `k` is `drag_n_per_m_s`.
- Only the wind itself is included, not the relative airflow from the vehicle's own motion, so the result in still air is exactly the same as with wind turned off. The linear model is reliable up to a total wind speed of about 3 m/s (about half of the hover downwash velocity).
- The coefficients have no default values. They must be filled in and their sources noted. `configs/helix_rotor_wind.yaml` gives nominal values and how they were derived.

The old center-of-mass OU force `ou_wrench` is kept unchanged and can be used as a control group.

```bash
# Wind that is random only along the z axis; everything else as in helix_rotor_wind.yaml
python -m isaac_drone run --backend synthetic --config configs/helix_rotor_wind.yaml \
    --set 'effects.wind.turbulence.direction_w=[0,0,1]'
# Sweep the wind speed standard deviation and the seed along the same direction
python -m isaac_drone sweep --config configs/helix_rotor_wind.yaml \
    --grid 'effects.wind.turbulence.stationary_std_w_m_s=[0.5,1.0,1.5,2.0]' --grid 'simulation.seed=[0,1,2,3,4]'
# A 2 m/s downdraft that appears suddenly at 6 s and lasts 2 s
python -m isaac_drone run --backend synthetic --config configs/helix_rotor_wind.yaml \
    --set 'effects.wind.gusts=[{start_time_s: 6.0, duration_s: 2.0, delta_velocity_w_m_s: [0, 0, -2], shape: step}]'
```

### Sensors and state estimation (`sensors`, `estimation`)

At the end of each physics step, `sensors` synthesizes four kinds of readings from the ground truth at the start and end of the step and writes them to the `sensors` field of `telemetry.jsonl` (see docs/telemetry.md). The error models follow the MATLAB documentation for `imuSensor` (`accelparams` / `gyroparams` / `magparams`), `altimeterSensor`, and `gpsSensor`. The parameter names match MATLAB one to one (in lowercase with underscores), and the default values are taken from the devices on the Pixhawk 6C: ICM-42688-P, IST8310, MS5611, and ZED-F9P-class RTK. For the formulas, see Section 5 of docs/architecture.md.

General rules:

- Every sensor has `enabled`, `rate_hz`, and `delay_s`. The sample rate must divide the physics step rate 1/dt (at 200 Hz it can be 200, 100, 50, 40, 25, 20, 10 …), and the delay must be an integer multiple of dt (the GPS default is 0.12 s = 24 steps).
- The mounting position `position_b_m` is relative to the origin of the root link (`base_link`). The lever arm relative to the center of mass is computed automatically from the backend's mass properties (the IMU is at the origin by default, so the lever arm is r = −com_b). `orientation_rpy_rad` is the Z-Y-X Euler angles of the sensor frame relative to the body frame.
- Random numbers are derived from `SeedSequence([simulation.seed, fixed domain tag])`, with one substream per sensor, and they do not overlap with the `SeedSequence(seed)` used by the wind field/OU disturbance. So `simulation.seed` determines both the disturbance and the sensor noise, while turning sensors on or off or changing sensor parameters does not change the flight result.
- `noise_type`: the (one-sided) spectral density given in datasheets corresponds to MATLAB's default `double-sided`, where the per-sample σ = density·√(fs/2) (10 × density at 200 Hz); for `single-sided`, σ = density·√fs.

| Section | Contents |
|---|---|
| `imu.accelerometer` / `imu.gyroscope` | MATLAB parameters: `measurement_range` (`null` = no saturation), `resolution` (0 = no quantization), `constant_bias`, `axes_misalignment` (in percent: a scalar = all off-diagonal elements; a 3-element vector or 3×3 matrix follows the MATLAB rules), `noise_density`, `bias_instability`, `random_walk`, `noise_type`, `temperature_bias`, `temperature_scale_factor`; the gyroscope also has `acceleration_bias` (g-sensitivity, multiplied by the specific force in the sensor frame). `bias_instability` is the stationary standard deviation of a Gauss-Markov process, and `bias_instability_time_constant_s` is its correlation time (MATLAB uses a 1/f filter; here only the statistics are reproduced, not the sample-by-sample values). The temperature bias and temperature scale factor terms are nonzero only when `temperature_c` differs from 25 °C |
| `imu.vibration` | Rotor vibration term that is not part of MATLAB: white-noise standard deviation of the accelerometer/gyroscope (value at hover); with `scale_with_thrust: true` it scales with total thrust/weight |
| `magnetometer` | `field_w_ut` is the constant geomagnetic field in the world frame (ENU); `motor_interference_b_ut_per_n15` is the motor current interference coefficient, multiplied by Σ T_i^1.5 (`applied_thrust_n`); `model` holds the parameters with the same names as in magparams |
| `barometer` | `altimeterSensor`: `constant_bias_m`, `noise_density_m_sqrt_hz`, `bias_instability_m`, `decay_factor`. The bias instability follows the recursion β(k) = decay·β(k−1) + σ·w; decay 0 gives white noise and 1 gives a random walk, and the stationary standard deviation is σ/√(1−decay²) |
| `gps` | `gpsSensor`: `profile` is `rtk` or `standard`. For the four numeric items (`horizontal_accuracy_m`, `vertical_accuracy_m`, `velocity_accuracy_m_s`, `decay_factor`), `null` uses the profile value and a number overrides it. The position error on each axis is e(k) = decay·e(k−1) + σ·w, and the velocity error is white noise; the output is the local ENU position/velocity of the antenna |

```bash
# Switch only to the standard GPS profile and run the estimator in shadow mode (the controller still uses ground truth)
python -m isaac_drone run --backend synthetic --config configs/helix.yaml \
    --set sensors.gps.profile=standard --set estimation.mode=shadow
# Close the loop on the estimate (only passthrough exists for now, so the result should be exactly the same as truth)
python -m isaac_drone run --backend synthetic --config configs/helix.yaml --set estimation.mode=closed_loop
# Turn off the sensors (the flight result does not change; the readings are just not recorded)
python -m isaac_drone run --backend synthetic --set sensors.enabled=false
```

For an airborne start, the `closed_loop` trajectory is anchored to the estimator output at t=0. The helix requires the initial angular speed to be below 0.001 rad/s, and a noisy estimate may raise an error at reset, so initialization has to be handled when a real filter is connected (align first, then release).

## 4. Controller

### `geometric`: geometric control (Lee SO(3), the only controller)

This controller consists of a position-loop PID and an SO(3) attitude PD, and includes jerk/snap and full-inertia feedforward.

| Parameter | Meaning |
|---|---|
| `position_kp` [s⁻²] | P: position error → acceleration |
| `velocity_kd` [s⁻¹] | D: velocity error → acceleration |
| `position_ki` [s⁻³] | I: integral of position error → acceleration; on by default (base config `[1.5, 1.5, 0.5]`, `helix.yaml` `[2, 2, 0.5]`); set an axis to 0 to turn off the integral on that axis |
| `integral_limit_m_s` [m·s] | Per-axis upper limit of the position error integral; the integral stops growing when allocation saturates or the thrust direction is limited |
| `attitude_kp` [N·m/rad], `angular_rate_kd` [N·m·s/rad] | Direct torque gains of the attitude-loop PD (the attitude loop has no integral term) |

The position-loop PID in `helix.yaml` (kp `[20, 20, 16]`, kd `[9, 9, 7]`, ki `[2, 2, 0.5]`) was tuned in Isaac on 2026-10-20, using the tracking error at the same time instants of the undisturbed helix. The error decreases monotonically with D, but the larger D is, the more easily the numerical differentiation of the feedback force under the OU force disturbance saturates the motors (at σ=2 N, 1.1% for kd 9 and 14% for kd 20), so kd 9 was chosen. kp has almost no effect on the undisturbed error. The smaller ki is, the better, so the smallest tested value was chosen. For the ideal double-integrator closed loop `s³ + kd s² + kp s + ki`, the stability limit is `ki < kp·kd`.

## 5. Trajectory

A trajectory is made of several **geometric segments**. Each segment is traversed with a rest-to-rest time law, and at the end the vehicle hovers at the end point indefinitely (phase name `hold`). When the duration is a number, a ninth-degree polynomial (C4) is used; when it is `null`, the minimum time of an "accelerate-cruise-decelerate" profile is planned from the actual mass, inertia, allocation matrix, thrust limits, and motor lag (this requires `thrust_utilization`). All segments go through nominal feasibility screening before the flight (thrust, tilt, yaw rate, acceleration).

### `hold`

The parameters are `position_w_m` (`null` means the actual center of mass after reset) and `yaw_rad` (`null` means the actual heading).

### `helix`

The phases are, in order, `takeoff` (vertical climb in place), `helix` (cylindrical helix climb with constant radius), and `hold`. The parameters are: `takeoff_height_m`, `takeoff_duration_s`, `radius_m`, `turns` (positive is counterclockwise seen from +Z; can be negative or fractional), `climb_height_m`, `helix_duration_s`, `initial_phase_rad`, `yaw_mode` (`fixed`/`tangent`), `yaw_offset_rad`, `thrust_utilization`, `rest_speed_tolerance_m_s`, `rest_angular_speed_tolerance_rad_s` (the vehicle must be at rest at reset). For the math, see [spiral_math.md](spiral_math.md).

## 6. Adding a controller or trajectory

Both kinds of plugins consist of a parameter dataclass and a registered factory function. The parameters are strictly parsed by `isaac_drone.core.params` (which supports `float`, `int`, `bool`, `str`, `Literal`, `Vec3`, `X | None`, and so on), and range checks go in `__post_init__`.

**Controller** (`isaac_drone/control/my_controller.py`; import it once in `control/__init__.py` to register it):

```python
@dataclass(frozen=True)
class MyGains:
    kp: Vec3

class MyController:
    output = "wrench"            # or "rotor_thrust": outputs per-rotor thrust directly (e.g. a learned policy), only clipped to the limits
    def __init__(self, gains, vehicle, limits): ...   # vehicle: actual mass/inertia/allocation matrix/thrust limits/control period
    def reset(self): ...
    def compute(self, state, setpoint, dt_s): ...      # returns a Wrench (body frame, about the center of mass) or per-rotor thrust
    def notify_allocation(self, result): ...           # allocation result, used for anti-windup; can be ignored
    def diagnostics(self): ...                         # desired attitude/angular velocity (written to telemetry) and any other details

@CONTROLLERS.register("my_controller", params=MyGains)
def build(params, *, vehicle, limits):
    return MyController(params, vehicle, limits)
```

It is also possible to skip registration and pass a controller instance directly to `MotionControlLoop(config, backend, controller=...)`.

**Trajectory**: a new trajectory needs to inherit from `SegmentedTrajectory`, implement `_build(position_w, yaw_rad)`, and return `(segment list, end position, end yaw)`. Each `Segment` needs to provide `setpoint(progress)` (which maps the progress and its 1st–4th derivatives to position through snap and yaw), the path length, the yaw change, and the duration (`None` means minimum-time). It then gets minimum-time planning, feasibility screening, phase division, the completion criterion, trajectory overlay display, and per-phase metrics automatically. See `trajectories/helix.py` for reference.

## 7. Batch experiments

Robustness experiments follow the approach of Kidambi (IEEE Access 2025) and sweep over parameter mismatch and disturbance strength, for example:

```bash
python -m isaac_drone sweep --config configs/helix_moderate.yaml --grid model_mismatch.mass_ratio=[0.75,0.9,1.0,1.15,1.3]
python -m isaac_drone sweep --config configs/helix_moderate.yaml --set effects.ou_wrench.enabled=true \
    --grid effects.ou_wrench.force_std_w_n=[[0.5,0,0],[1,0,0],[2,0,0]] --grid simulation.seed=[0,1,2,3,4]
```

```bash
# Grid sweep (CPU synthetic backend, multiple processes in parallel): each point is a full run directory
python -m isaac_drone sweep --config configs/helix.yaml \
    --grid 'controller.position_ki=[[0,0,0],[3,3,2],[6,6,4]]' \
    --grid 'controller.velocity_kd=[[6,6,4],[7,7,5]]' \
    --rank helix.position_error_max_m
# Compare several runs (a single phase can be compared on its own)
python -m isaac_drone compare runs/<A> runs/<B> --labels no_integral integral --phase helix
```

Before a sweep starts, the configs of all points are validated. If a single point diverges, it is only recorded as failed and the sweep continues. The results are written to `runs/sweep_<MMDD-HHMM>/summary.{csv,json,md}`, sorted in ascending order of the `--rank` metric. The run directory of each point is `points/<index>/`, where the index is the `index` column in the summary. For metric names, see `isaac_drone/analysis/metrics.py` (write the name directly for an overall metric, and `phase.metric` for a per-phase metric).

## 8. Migrating from schema_version 1

| v1 | v2 |
|---|---|
| `control:` (gains and limits mixed together) | `limits:` (`max_tilt_rad`, `max_yaw_rate_rad_s`, `max_acceleration_m_s2`) + `controller: {kind: geometric, gains}` |
| `trajectory: {kind, hold: {...}, spiral: {...}, completion: {...}}` | `trajectory: {kind: hold/helix, parameters of that kind}` + top-level `completion:` |
| `trajectory.spiral.start_delay_s` | `vehicle.launch.spin_up_s` |
| `spiral_duration_s` | `helix_duration_s` |
| `initial_speed_tolerance_m_s`, `initial_angular_speed_tolerance_rad_s` | `rest_speed_tolerance_m_s`, `rest_angular_speed_tolerance_rad_s` |
| `kind: spiral` alias | Removed; use `helix` |
| `configs/arl_robot_1_helix.yaml` | `configs/helix.yaml` (`extends: arl_robot_1.yaml`) |
| Telemetry `mission_phase`: `delay/takeoff/spiral/hold` | `spin_up/takeoff/helix/hold` |
