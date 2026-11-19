# Isaac Drone Sim

This is a quadrotor simulation project built in Isaac Sim / Isaac Lab, using the ARL-Robot-1 model. I made this repository to learn drone flight control and to get familiar with the whole process: modeling, writing the control program, running experiments, and analyzing the data. I use Isaac Sim because it has excellent simulation quality, and it is the same platform I used before for reinforcement learning robot training. This also gives me a base for transferring my reinforcement learning experience to drones later.

For now I start with the simplest tasks, hovering and helix trajectory tracking. This helps me learn how the control program behaves and what each parameter does, and build basic intuition for flight control. Wind disturbances and sensor errors can also be added to the simulation, and the data and plots of each run are saved automatically.

The default branch is `isaac-drone/Version5`.

<!-- New experiment results: video, 3D trajectory, tracking error, experiment conditions and metrics. To be added when the material arrives. -->

## Control flow

```text
Trajectory → Position and attitude control → Thrust allocation → Motor speed → Physics simulation
```

## How to run

Python 3.10+ is required. Install the dependencies from the repository root:

```bash
python -m pip install -e '.[dev]'
```

To check that it works, run the CPU synthetic model with the commands below:

```bash
python -m isaac_drone run --backend synthetic --config configs/helix.yaml
python -m isaac_drone summarize
```

To use Isaac Sim / PhysX, run inside a configured Isaac environment. The USD model is stored in Git LFS. Use the command below to pull the model:

```bash
git lfs pull
```

Then run:

```bash
python -m isaac_drone inspect --visualizer none
python -m isaac_drone run --backend isaaclab --config configs/helix.yaml --visualizer none
```

Test command:

```bash
python -m pytest -q
```

Results are saved automatically under `runs/`, including the config, run information, telemetry, metrics, and plots.

## Code layout

| Path | Contents |
|---|---|
| `configs/` | Vehicle parameters and experiment configs |
| `isaac_drone/trajectories/`, `isaac_drone/control/` | Trajectories, controllers, thrust allocation |
| `isaac_drone/sim/`, `isaac_drone/effects/` | Simulation backends, motor and disturbance models |
| `isaac_drone/sensors/`, `isaac_drone/estimation/` | Sensor models and the estimator interface |
| `isaac_drone/runtime/` | Control loop and experiment scheduling |
| `isaac_drone/telemetry/`, `isaac_drone/analysis/`, `isaac_drone/viz/` | Data recording, analysis, plotting, and video recording |
| `isaac_drone/core/`, `isaac_drone/config/` | Data structures, rotation math, config loading and validation |
| `assets/` | USD model |
| `tests/` | Unit, interface, and regression tests |

## Documentation

- [Configuration](docs/configuration.md): what each parameter does, how configs inherit from each other, and how to set up batch experiments.
- [Architecture](docs/architecture.md): how the modules relate to each other, the coordinate conventions and the physics model, with the verification record.
- [Trajectory derivation](docs/spiral_math.md): the geometry of the helix trajectory, time planning, and constraints.
- [Telemetry](docs/telemetry.md): the meaning, units, and timestamps of each field, and how the errors are defined.
- [Running on the server](docs/server_runbook.md): the steps to set up the environment, start WebRTC, and launch on the server.
