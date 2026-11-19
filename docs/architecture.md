# ARL-Robot-1 Motion Control Architecture

This project is based on the local ARL USD and the current `IsaacLab/` source code, and targets **a single vehicle, a rigid-body assembly with fixed joints, and PhysX**. The general motion control infrastructure already supports takeoff in place, a three-dimensional cylindrical helix climb, and hover at the end point. The interfaces, config validation and numerical tests can run in plain Python, but actual physics stepping requires an environment with the current Isaac Lab / Isaac Sim installed.

## 1. Data flow and directories

```text
configs/*.yaml (inherited with extends + overridden with --set)
        │
        ├── sim/: backend (isaaclab: USD + PhysX actual mass/center of mass/inertia/geometry; synthetic: CPU synthetic rigid body)
        │
        └── runtime.run_experiment (backend-independent: stepping, recording, hooks)
              └── runtime.MotionControlLoop (once per physics step)
                    ├── trajectories: position, velocity, acceleration, jerk, snap, yaw and its derivatives of the segmented trajectory; holds the start point during spin-up
                    ├── sensors: at the end of the step, IMU/magnetometer/barometer/GPS readings are synthesized from the ground truth at the start/end of the step (MATLAB error models, delay queues)
                    ├── StateProvider: truth = ground truth / shadow = estimator runs alongside and is recorded / closed_loop = controller uses the estimate (estimation package; filters not implemented yet)
                    ├── control: registered controller (geometric / injected instance) → force/torque about the center of mass, or per-rotor thrust
                    ├── control.BoundedAllocator: bounded thrust target for each rotor [N]
                    ├── effects: additional center-of-mass force/torque at each physics step (constant/OU random disturbance, wind, aerodynamic drag) + rotor gyroscopic torque
                    └── backend: RPS motor dynamics (off by default) → force applied to each rotor separately → physics step
              Hooks: trajectory overlay, synchronized multi-view video recording, wall-clock paced display, metrics.json, PNG plots
```

| Directory/file | Responsibility |
|---|---|
| `isaac_drone/core/` | Shared definitions of state, center-of-mass inertia, trajectory and force/torque (`types`); SO(3) math (`rotations`); numerical checks; plugin parameter parsing (`params`) and registry (`registry`) |
| `isaac_drone/config/` | Strict YAML parsing, `extends` merging, `--set` overrides, checks for duplicate/unknown fields, validation of physical quantities and of relations between fields, local asset checks |
| `isaac_drone/trajectories/` | `SegmentedTrajectory` framework, hold, C4 three-dimensional cylindrical helix, time laws, minimum-time planning, feasibility screening, completion criterion at the end point |
| `isaac_drone/control/` | Controller interface and registry, geometric control (position PID, SO(3) attitude, full inertia feedforward), thrust direction limiting, bounded control allocation |
| `isaac_drone/effects/` | Constant external force/torque, OU random disturbance, wind field, gusts, drag from relative airflow; combining disturbances, application points and frame conversion, time windows |
| `isaac_drone/sensors/` | Noise processes (white noise, random walk, Gauss-Markov, MATLAB decay recursion), the `imuSensor` error chain (`triaxial`), ground-truth kinematics and specific force within a step (`truth`), the four sensors, management of sample rates/delays/random streams (`suite`) |
| `isaac_drone/estimation/` | Estimation modes and the estimator interface (`base`), the `passthrough` state provider, definitions of estimation/control errors (`errors`); the filters (attitude, altitude, 15-dimensional error-state EKF) are left for the next stage |
| `isaac_drone/runtime/` | Control decimation, physics step scheduling, spin-up envelope, the nominal model for `model_mismatch`, rotor gyroscopic torque, reset and handling of abnormal states (`loop`); runner and hooks (`runner`, `hooks`) |
| `isaac_drone/sim/` | Backend/driver interface (`base`), reuse of the native motor integration (`actuation`, `native_thruster`), CPU synthetic plant (`synthetic`), offline USD check (`usd_audit`) |
| `isaac_drone/sim/isaaclab/` | Isaac Lab API, quaternion conversion, extraction of the real geometry/inertia, native motors and force application (`backend`); scene, driver, refreshing the streaming UI (`app`); standalone camera capture (`camera`) |
| `isaac_drone/telemetry/` | Actual motion data at each step, full angular momentum differencing, position error and the separation of force/torque stages (`measurements`); config snapshot, asset layer hashes, JSONL/CSV and the field schema (`recorder`) |
| `isaac_drone/viz/` | Trajectory overlay geometry, wall-clock pacing, camera framing, synchronized multi-view encoding |
| `isaac_drone/analysis/` | Metrics, per-phase summaries, comparison of multiple runs, parameter sweeps, plotting |
| `isaac_drone/cli.py` | The `python -m isaac_drone` command line: validate / run / inspect / audit-usd / sweep / compare / summarize / plot |

## 2. Model parameters and where they come from

Reference source code:

- `IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/arl_robot_1.py`
- `IsaacLab/source/isaaclab_contrib/isaaclab_contrib/actuators/thruster.py` and `thruster_cfg.py`
- `IsaacLab/source/isaaclab_contrib/isaaclab_contrib/assets/multirotor/`
- `IsaacLab/source/isaaclab_contrib/isaaclab_contrib/controllers/`
- All USD layers under `assets/Robots/NTNU/ARL-Robot-1/`

The parameters below are read directly from the current USD model file:

- The asset contains one `base_link`, four rotor rigid bodies and four **fixed joints**.
- The masses written in the USD are about 1.20 kg for the body and 0.01 kg for each rotor, about 1.24 kg in total. This value is only used to check the model. The mass used by the controller is read from the PhysX loading result every time.
- The rotor origins are BL=(-0.10,+0.10,0), BR=(-0.10,-0.10,0), FL=(+0.10,+0.10,0), FR=(+0.10,-0.10,0), in m, relative to the root link.
- The center of mass in the USD is a default value that was not written out, so it cannot be replaced with zero. At run time the center of mass evaluated by PhysX is used.
- The upstream reference allocation matrix uses ±0.13 m, and its signs/ordering also do not match the actual geometry. When its spin directions `[-1,+1,-1,+1]` are combined with the actual positions, the collective thrust/roll/pitch/yaw matrix has rank 3.
- The **diagonal same-direction simulation configuration** is used. The current action order is fixed as `[back_left, back_right, front_left, front_right]`, the direction coefficients are `[-1,+1,+1,-1]`, and the source tag is `simulation_diagonal_pairs`. These directions are a setting in the simulation, not values measured on the real vehicle.

`rotor_directions` keeps the native field name, and the backend uses it to decide the sign of the reaction torque `torque_to_thrust_ratio * thrust`. When a real vehicle is connected in the future, the physical spin direction of the rotors, the motor numbering, the positive sign of the reaction torque and the coordinate frame must all be checked together.

The allocation matrix is built from the actual model: each column consists of the thrust unit axis and `(rotor position - vehicle center of mass) × thrust axis + reaction torque`. The upstream matrix in the YAML is kept only for diagnosing differences. After the USD geometry is changed, there is no need to assume the same arm length by hand again.

## 3. Coordinates, mass and inertia

- The world frame has Z up, and all values use SI units.
- Quaternions in the project are always **wxyz** and represent the rotation from the body frame to the world frame. The state/write interfaces of the current local Isaac Lab 3.0 use **xyzw**, so the conversion is done only at the interface boundary.
- `VehicleState.position_w/linear_velocity_w` are the position/velocity of the **vehicle center of mass**; `angular_velocity_b` is the angular velocity in the body frame.
- `Wrench` is `[Fx,Fy,Fz,Tx,Ty,Tz]` about the vehicle center of mass, expressed in the body frame.
- The YAML `initial_state.pos` keeps the native meaning, which is the position of the root link origin; `lin_vel` is the world-frame linear velocity of the native root rigid body's center of mass, and `ang_vel` is the world-frame angular velocity. They are kept separate from the definition of the shared state on purpose, and the backend converts between them.
- The vehicle inertia is summed from the mass, actual center of mass, principal-axis orientation and full inertia tensor of every rigid body, using rotation and the parallel axis theorem. The calculation uses the actual center-of-mass position and the full inertia tensor, and includes the mass of each rotor.
- The thrust and reaction torque of the four rotors are applied at the origin of each rotor rigid body and passed through the fixed joints. The controller's whole-vehicle target wrench is not submitted to the root rigid body. Only external disturbance forces that are explicitly configured act on the root rigid body, and their torque is converted using the offset between the vehicle center of mass and the root rigid body's center of mass. The total force/torque of the vehicle is only used for control allocation and for diagnostics in the logs.

OpenUSD's `diagonalInertia` is the inertia in principal-axis coordinates, and `principalAxes` is the orientation of the principal axes relative to the prim; `centerOfMass` is in the prim's local coordinates. See [OpenUSD MassAPI](https://openusd.org/25.11/api/class_usd_physics_mass_a_p_i.html). For the COM local frame meaning of PhysX `get_inertias`, see [Omni Physics Tensor API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.0/extensions/runtime/source/omni.physics.tensors/docs/api/python.html).

## 4. What can be configured

The base config is `configs/arl_robot_1.yaml`. Each experiment can use a file that inherits it with `extends` and is passed with `--config`, or a one-off override with `--set` (see [configuration.md](configuration.md)). Unrecognized fields, duplicate YAML keys, NaN/Inf, wrong unit shapes and negative time constants all raise an error.

- `simulation`: physics step size, control decimation, render decimation, gravity, device, random seed, duration.
- `recording`: off by default. The default frame rate is 60, each camera view is 1280×720, and the camera views can be any non-empty list without duplicates chosen from `overview/follow/top`. When an old config omits the whole section, the default values are filled in. When recording is enabled, the frame rate must not exceed the physics rate and the image dimensions must be positive even numbers. The CLI can override the switch, frame rate, size and camera views for a single run.
- `vehicle.thrusters`: all upstream dynamics fields, keeping the thrust limits, the thrust coefficient range, the rise/fall time constants, the reaction torque coefficient, the rate limit, the integration method and the discrete approximation.
- `vehicle.initial_state`: full attitude, position, velocity, and the initial rps of each rotor.
- `vehicle.launch`: ground height, initial clearance, and whether the initial root Z is computed from the real collision bodies; when enabled, zero initial velocity and zero rotor speed are required. The calculation includes the enabled collision bodies inside instance proxies and recomputes the shape extent, so that the outdated authored extent of the current asset does not put the vehicle through the ground. This position is written only during reset initialization.
- `vehicle.native_overrides`: recursively overrides the current native robot config, including rigid bodies, joints, collision, mass, materials and so on. Unknown native fields are rejected when the native config is instantiated.
- `simulation.native_overrides`: the remaining parameters of the native `SimulationCfg` / `PhysxCfg`. This way the whole config class does not have to be rewritten just to expose one PhysX field.
- Main fields such as dt, the asset path and the action order have a single source and cannot be overridden again in native overrides, so that the controller and the physics model do not end up using two different values.
- `vehicle.usd_overrides`: after the scene starts and before physics is initialized, modifies the mass, center of mass, principal inertia and principal-axis attributes of existing prims, without writing back to the original USD file. For example:

```yaml
usd_overrides:
  - prim_path: base_link
    attribute: physics:centerOfMass
    value: [0.01, 0.0, 0.0]  # example format, not a default physical parameter of this project
```

`null` does not mean that the measured value is zero. A null in the native overrides means the native value is kept; aerodynamic calibration values that are not filled in are unknown and must be filled in once the feature is enabled. Values that the native config already has, such as `linear_damping=0`, are marked with their upstream source, and these values do not mean that the real vehicle has no drag.

Fields can be changed, but not every physical setup works with the current controller. Free flight requires gravity to match the control config, rigid-body dynamics to be enabled, no fixed base, and all four thrust axes to point along body +Z. Tilting rotors, movable internal joints, tether constraints and kinematic bodies need matching controller/backend extensions. In inspect mode these combinations can be examined, but incompatible combinations are rejected before flight.

## 5. Timing, actuators and control

The default physics step size is 0.005 s and the control decimation is 2, which means a control rate of 100 Hz and a motor and disturbance rate of 200 Hz. These are settings for the numerical experiment and can be changed. They are unrelated to the hardware sample rate and are not necessarily the best step size.

Each physics step runs in the following order:

1. Read the state; if it is a control instant, sample the trajectory and compute the desired wrench.
2. Do a bounded weighted allocation within the thrust envelope that is valid for this step. For a quadrotor, at most 81 constraint combinations are enumerated, a weighted least-squares problem is solved, and the actual allocation residual is recorded. This is used instead of a simple pseudo-inverse followed by clipping.
3. Compute the wind/aerodynamics/external force for this step; when wind on the rotors is enabled, also compute the equivalent rotor speed offset for each rotor.
4. Convert the allocated thrust into an **RPS** command using each motor's own sampled kf; a smooth start-up envelope is applied during spin-up. When the motor model is on, RPS is integrated directly; when it is off (the default, `vehicle.motor_model_enabled: false`), the RPS of this step equals the limited command. The actual thrust is computed as kf·RPS² (kf·(RPS + offset)² when wind acts on the rotors), and the force is applied to each rotor separately. One per-rigid-body array is submitted to PhysX, and the root rigid body only receives explicit external disturbances.
5. Call `sim.step()` and `robot.update(dt)`, and record the actual thruster output and state.
6. At the end of the step, synthesize the sensor readings from the ground truth at the start/end of the step and pass them to the state provider's `update`; `read_state` only queries and does not advance anything. The readings of step k arrive before the controller reads the state in step k+1, so causality is preserved.

The ground truth used by the sensor layer is defined in `sensors/truth.py`: the angular rate is the body angular rate at the end of the step, and the specific force is the average over the interval

$$
f_b = R_{\text{pair}}^{\top}\left(\frac{\Delta v_w}{\Delta t} - g_w\right)
$$

The Isaac backend pairs it with the attitude at the start of the step (external forces are applied in the body frame at the start of the step and held for one step). The synthetic backend pairs it with the slerp midpoint attitude between the start and end of the step (each RK4 substep converts the thrust with the attitude at that moment). Each backend declares the pairing it uses through `specific_force_attitude_pairing`. This way the ground support force is included automatically, and the reading at rest is (0, 0, +9.81). The specific force at the mounting point also needs the lever arm terms

$$
f_{\text{imu}} = f_b + \alpha_b \times r + \omega_b \times (\omega_b \times r)
$$

where `α_b` is obtained by differencing the world-frame angular velocity and then converting it to the body frame at the end of the step (the same as in `telemetry/measurements.py`), and `r` is the body-frame position of the mounting point relative to the center of mass. The GPS antenna position is `p + R r`, its velocity is `v + R(ω × r)`, and the barometer takes the world z of the mounting point.

The error chain follows the Algorithms order of MATLAB `imuSensor` (`sensors/triaxial.py`): misalignment and constant bias, bias instability, white noise, random walk, temperature drift, gyroscope g-sensitivity, temperature scale factor, saturation, and quantization:

$$
d = \Big[\tfrac{M}{100} a + b_0 + \beta_1 + \beta_2 + \beta_3 + (T-25)\,b_T + k_g f\Big]\Big(1 + \tfrac{T-25}{100}\,s_T\Big),\qquad
y = \text{Res}\cdot\operatorname{round}\!\Big(\tfrac{\operatorname{clip}(d,\pm\text{Range})}{\text{Res}}\Big)
$$

$$
\beta_2 = w\sqrt{\tfrac{f_s}{s}}\,N,\qquad
\beta_3(k) = \beta_3(k-1) + \frac{w(k)\,K}{\sqrt{f_s/s}},\qquad
\beta_1(k) = e^{-\Delta t/\tau}\beta_1(k-1) + \sigma_B\sqrt{1-e^{-2\Delta t/\tau}}\,w(k)
$$

`s` is 2 (`double-sided`, the MATLAB default, which corresponds to the single-sided density in datasheets) or 1 (`single-sided`). `β₁` uses a Gauss-Markov process with stationary standard deviation `σ_B` = BiasInstability and correlation time `τ` in place of MATLAB's 1/f filter, and it is discretized exactly in the same way as `effects/stochastic.py`. The barometer and GPS follow `altimeterSensor` / `gpsSensor`:

$$
z_{\text{baro}} = z + b_0 + w\sqrt{\tfrac{f_s}{s}}\,N + \beta(k),\qquad
\beta(k) = a\,\beta(k-1) + \sigma\,w(k),\qquad
e_{\text{gps}}(k) = a\,e_{\text{gps}}(k-1) + \sigma\,w(k)
$$

`a` is the DecayFactor. A value of 0 gives white noise and a value of 1 gives a random walk (the MATLAB documentation's description of the two ends is only consistent with this unnormalized recursion). The stationary standard deviation is `σ/√(1−a²)` and the time constant is `−Δt/ln a`; the GPS velocity error is white noise. The magnetometer reading is `Rᵀ B_w + k_m Σ T_i^{1.5}`, which then goes through the same error chain; the IMU also gets vibration white noise that scales with total thrust/weight. The random stream `SeedSequence([seed, domain tag])` is independent of the disturbance stream `SeedSequence(seed)`.

When the motor model is on, the native motor keeps the random sampling of `kf` and tau at reset, the asymmetric dynamics, the rps² thrust relation, RK4/Euler, and saturation. When it is off, only the kf sampling, the rps² thrust relation and the rotor speed limits are kept, and there is no response delay. Minimum-time planning always leaves a lead margin based on the tau range, so turning the motor model on or off does not change the reference trajectory. `max_thrust_rate` keeps the native field name/config value, but the source code applies the limit in the integration of the rps error, so it should not be read as a strict N/s limit just because of its name. The logs record the kf/tau that were actually sampled.

The current upstream implementation has two initialization problems, and local workarounds are in place. First, the thruster may get a copy of the initial rps that has not been filled in yet when it is created. Second, reset writes rps into the N-valued target. The workaround is that the backend rebinds the initial rps at reset and overwrites the target with the real initial thrust before the first dynamics update. The official copy of `IsaacLab/` is not modified.

The position loop outputs an acceleration command. The attitude loop uses direct torque gains, with full `ω×Jω` and desired angular acceleration feedforward. The trajectory can provide velocity, acceleration, jerk, snap and yaw derivatives. When jerk/snap are provided and no limiting occurs, the derivative of the reference force uses the analytic value and the feedback part still uses numerical differencing; after limiting, it falls back to differencing the whole force vector, to avoid using a wrong analytic derivative for the projected force. The position integral is on by default (P, I and D all on) and includes per-axis integral limits and anti-windup through allocation feedback; setting `position_ki` to 0 on an axis turns off the integral on that axis.

The range of application needs to be stated here: the current attitude controller is designed for upright flight, and the smooth SO(3) error has an equilibrium at exactly 180°. When jerk/snap are not provided, the derivative of the thrust direction comes from history differencing; there is no history at the first frame, and abrupt references and noise cause transients in the derivative. `max_yaw_rate_rad_s` limits the angular rate feedforward, and it does not turn a yaw angle step into a smooth turn automatically.

`prepare_step/finish_step` must be called in pairs. After any plugin or stepping failure, the same frame must not be retried, so that the controller state is not integrated twice; a full reset is needed in this case. When the simulation is paused the model does not advance, and when the simulation stops the run ends.

## 6. Disturbances and aerodynamic extensions

On by default:

- `ou_wrench`: Kidambi-style random disturbances d1/d2. The force acts on the center of mass in the world frame and the torque is in the body frame. Each is an OU process described by a mean, a per-axis standard deviation and a correlation time (stationary initialization, exact discretization, reproducible from `simulation.seed`). By default the force mean is [1,0,0] N, the standard deviation on each axis is 0.5 N, τ=1 s, and the torque is zero. This is a demonstration magnitude, not a calibrated value. In `helix.yaml`, which takes off from the ground, the disturbance starts when spin-up ends (2 s), so that the vehicle is not pushed along the ground.

Implemented but off by default:

- Constant force/torque in the world frame or body frame, start and end time windows, and application points relative to the center of mass or absolute in the world.
- Uniform wind, spatial gradient, smooth or step gusts, and reproducible OU random wind (per world axis, or a single process along a custom direction; stationary initialization). The whole wind field has a start/end time window, and it appears at full strength when the window opens.
- Wind on the rotors (`rotor_wind`), split in the body frame: wind along the rotor axis is converted into an equivalent rotor speed offset for each rotor and added after the motor model (thrust and reaction torque use kf·(RPS + offset)²); wind in the rotor disk plane acts as drag at the center of mass. Only the wind itself is counted, not the vehicle's own motion. The formulas and the sources of the coefficients are in configuration.md.
- Linear matrix drag on relative wind speed, per-axis quadratic drag, angular rate drag, and a center-of-pressure arm.

When aerodynamics is enabled, the wind field must be written out; even for still air, `[0,0,0]` must be written. The drag calculation includes the `ω×r` velocity of the application point. Matrix damping is checked for dissipativity. Wind can put energy into the vehicle, so the total work in the world frame is not required to be always negative.

For example, to add an extra crosswind, the identified drag parameters must be filled in as well as the wind speed, and the program does not provide a default such as a "typical drone coefficient"; the two `rotor_wind` coefficients also have no defaults and must be filled in explicitly with their source. Ground effect, rotor-rotor interference, induced flow, blade flapping, translational lift and similar effects do not have dedicated physical models yet. They will be added later through `EffectModel` or actuator plugins.

The rotor gyroscopic torque is off by default (`vehicle.rotor_inertia_kg_m2: null`). It is computed as τg = ω × (J_r e3 Σ σ_i Ω_i) (arXiv 2609.12188, Eq. 7), where σ_i is taken from `rotor_directions` (the reaction torque sign) and Ω_i is the actual rotor speed at this step. J_r must be filled in to enable it; 3.5e-5 is a placeholder of the right order of magnitude estimated for an 8-inch propeller and needs to be replaced once it is measured. At reset the Isaac backend randomly samples kf for each motor from `thrust_const_range`. Unequal speeds on the four rotors mean the net rotor angular momentum does not cancel, and the gyroscopic torque can grow to the same order as the control torque (a minimum-time helix combined with mismatch and OU goes out of control), so kf should be fixed before enabling it.

## 7. Model mismatch

`model_mismatch` follows Kidambi (IEEE Access 2025, Δm, ΔI, ΔK_T, ΔK_D in Section II-C) and defines the ratio of the plant's actual value to the controller's nominal value. The controller, the allocator and the minimum-time planner use the nominal mass, inertia (scaled per axis by congruence, which keeps it symmetric positive definite), thrust coefficient and reaction torque coefficient given by "backend ground truth ÷ ratio", while the plant still moves according to the ground truth. With a thrust coefficient mismatch, the actual thrust of each rotor equals the ratio times the thrust the controller thinks it is commanding, and the rotor speed limits stay as hardware limits. When all ratios are 1 there is no mismatch, and this is the default. For example, `mass_ratio: 1.2` means the plant is 20% heavier than the controller thinks (with the position integral off, hover settles g·0.2/kp_z lower in steady state; with the integral on this offset is removed). It can be swept with `sweep --grid model_mismatch.mass_ratio=[...]`.

## 8. Running and verification

A plain Python environment needs NumPy and PyYAML, and the tests also need pytest. In your own environment, use the following commands to install the project in editable mode and run the tests and config validation:

```bash
python -m pip install -e '.[dev]'
python -m pytest -q
python -m isaac_drone validate
```

The offline USD check also needs `usd-core` (`pip install -e '.[usd]'`). Use the following command to read the asset files and view unevaluated attributes without starting Isaac Sim:

```bash
python -m isaac_drone audit-usd
```

In an Isaac Lab environment with the dependencies installed correctly (on the server, after activating `env_isaacsim`), run the following commands from the root of this repository. Isaac Lab 3.0 removed `--headless`, so use `--visualizer none` to run without a display:

```bash
python -m isaac_drone inspect --visualizer none
python -m isaac_drone run --backend isaaclab --visualizer none --duration 10
```

`inspect` loads the actual PhysX properties, prints the allocation matrix/mass/inertia/sampled parameters, and then exits without running flight control. The code does not download the 29GB runtime environment automatically, and it does not switch to another vehicle model.

WebRTC streaming uses `--livestream 1` (public network, with `PUBLIC_IP` giving the server IP). Isaac Lab 3.0 treats the streaming host as headless and does not call `app.update()` when stepping, so when streaming, `sim/isaaclab/app.py` refreshes the Kit UI itself at every render step and turns off `playSimulations` during the refresh so that physics is not advanced an extra time. `--wait-for-start` keeps the UI refreshing after the scene loads without advancing physics, and the task starts only after Enter is pressed in the terminal.

Display, video recording, metrics and plotting are all hooks (`runtime/hooks.py`) of the runner (`runtime/runner.py`) and do not change physics stepping, control timing or logging. When there is a display (streaming or a local window), `DisplayHook` and `viz/pacing.py` align simulation time with the wall clock. The default `--playback-speed 1` plays in real time, and setting it to `0` turns the alignment off. At each render step, if the simulation is ahead, it waits; if refreshing the UI is slower than the frame interval and the simulation is more than 50 ms behind, that frame is skipped so that physics can catch up, but at least one frame is drawn every 0.25 s. The end event `finished.playback` records the wall-clock duration, the actual speed factor, the number of frames drawn/skipped and the average refresh time; if physics and recording are themselves slower than real time, this shows the lower speed factor that was actually reached. `viz/overlay.py` creates display-only USD geometry under `/World/Visuals` (without any physics/collision API): the green dashed line is the reference center-of-mass trajectory, the red solid line is the actual center-of-mass trajectory (one point for every 5 mm of movement, written at render frames), and the small green sphere is the reference point at the current time. `--no-path-overlay` turns the trajectory overlay off. With no display and no recording, this geometry is not created and there is no wall-clock alignment; the trajectory overlay can also be kept for off-screen recording.

Video recording (`VideoHook`) is not connected to `MotionControlLoop` or the dynamics backend. `--record-video` checks for the optional `imageio-ffmpeg` and the encoder before AppLauncher starts, and enables `enable_cameras` (the gym recording pipeline that comes with Isaac Lab is not started). `IsaacCameraRig` (`sim/isaaclab/camera.py`) creates a separate camera and render product for each selected camera view; the overview is framed from the reference path, and follow/top are positioned from the actual center of mass at each step. Each capture updates the cameras together, calls `sim.forward()` to sync Fabric, then temporarily turns off `playSimulations` and refreshes Kit once to read all RGB images; the physics step is not called. These render products are paused between captures, and their resources are released on exit.

`MultiViewRecorder` (`viz/video.py`) computes the sampling instants `index/fps` from integer frame indices, separately from `render_interval` and the wall-clock display throttling. Nothing is encoded while the cameras are being initialized or while waiting for input. Sampling starts at task t=0, images are taken at the physics step that reaches each sampling instant, and at the end only the sampling instants earlier than the end point are filled in. Each image is sent to both the separate MP4 files and `combined.mp4` at the same time, so the synchronized tiled video does not need time matching afterwards. The encoder uses streaming FFmpeg/H.264 ([imageio-ffmpeg provides the executable](https://github.com/imageio/imageio-ffmpeg)), so the raw frames of the whole experiment do not pile up in memory. `video_frames.jsonl` records the encoded frame index/video time/actual simulation sampling time; the finish/abort events record the frame count and encoding status of each file. On normal completion, simulation stop and exceptions (including Ctrl+C), the runner closes all hook resources in reverse order (cameras before encoders).

By default the run results are written to `runs/<MMDD-HHMM>_<experiment>_<backend>[_video]/` (US Eastern time, `telemetry/naming.py`; `_2` is appended when the name already exists). The directory contains the config snapshot, the actual physical parameters, the schedule of task phases, the git commit, the command line and UTC start time, `telemetry.jsonl`, `basic.csv`, `telemetry_schema.json`, `metrics.json`, and the run card `README.md` generated from them (`analysis/catalog.py`, also used by the `runs` summary table). The isaaclab backend also records the SHA256 of all local USD layers, and when recording succeeds it copies the tiled video to `runs/videos/<run directory name>.mp4`. By default one record is written per physics step (currently 200 Hz), including position, velocity, acceleration, the speeds of the four motors, force/torque and position error; the logs separate the desired value, the allocation result, the actual output of the native motors, the additional forces, and the net effect derived from the motion. The full fields and timing definitions are in [telemetry.md](telemetry.md).

The three-dimensional helix task is run with the following commands:

```bash
python -m isaac_drone validate --config configs/helix.yaml
# then run it in the full simulation Python environment (add --visualizer none to run without a display)
python -m isaac_drone run --backend isaaclab --config configs/helix.yaml
```

The start point is the real center of mass after reset. During the spin-up phase set by `vehicle.launch.spin_up_s`, the reference stays at the start point and the motor commands are scaled up by a smooth envelope. After that the reference goes through vertical takeoff in place, a constant-radius helix climb and a smooth deceleration, and finally holds at the end point indefinitely. The reference trajectory is C4 continuous and provides analytic fourth-order derivatives. The derivation and the constraints are explained in [spiral_math.md](spiral_math.md).

Before running, the code checks hover feasibility under the actual thrust limits, the analytic maximum yaw rate, and the torque/tilt feasibility at the discrete reference sample points. These checks cannot replace a closed-loop test flight with finite motor response, contact and disturbances. `trajectories/completion.py` uses the real state at every physics step to check the continuous hover time, and a timeout failure is not overwritten by a state that arrives later. After the end point is reached normally, feedback control keeps running. `mission` in the JSONL gives the tolerances, the dwell time and the success/timeout status, and `finished.success` is the final result of the whole run.

## 9. Scope of verification

So far the plain Python numerical tests, interface tests and config validation have been run, and the structure and parameters have been checked by parsing the actual USD. `tests/test_golden_regression.py` uses the CPU synthetic plant to lock the closed-loop trajectories from before the refactor (helix 20 s, hold 5 s), so any change that alters flight behavior is detected. The tests cover a nonzero center of mass, non-diagonal inertia, rotor order, torque application points, allocation saturation, reset and time pairing, disturbances and more.

The sensor layer and the estimation interface (`sensors/`, `estimation/`) have been verified on the synthetic backend: turning sensors on or off does not change the flight; `closed_loop` + `passthrough` matches the ground-truth flight point by point (golden regression); and there are unit tests for the noise statistics (white noise σ, Allan slope ∓1/2, Gauss-Markov correlation time, both limits of the decay recursion), misalignment/saturation/quantization, lever arm and paired attitude, and sample rate/delay/timestamps. The estimator algorithm itself has not been implemented yet, and the specific force pairing on the Isaac side (attitude at the start of the step) still needs to be measured on the server.
