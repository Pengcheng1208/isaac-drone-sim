# Running on the Server

Replace `<SERVER_PUBLIC_IP>` in the commands below with the public IP of the server `uc1`. If the IP changes after the instance restarts, update both the commands and the HostName of `uc1` in your local `~/.ssh/config`.

## First-time setup

```bash
sudo apt install git-lfs && git lfs install
git clone https://github.com/Pengcheng1208/isaac-drone-sim.git ~/isaac-drone-sim
cd ~/isaac-drone-sim
source ~/isaacsim-6.1/env_isaacsim/bin/activate
python -m pip install -e '.[plot]'          # add '.[video]' for video recording
```

To use the model from this repository in the Isaac Sim Script Editor, use the code below:

```python
import sys; sys.path.insert(0, "/home/ubuntu/isaac-drone-sim")
from isaac_drone.assets import ARL_ROBOT_1_USD
```

## Each flight

**1. Power on**: start the instance in the cloud provider's console.

**2. Connect to the server from a local terminal**

```bash
tmux attach -t isaac || tmux new -s isaac    # enter the shared terminal (attach if it exists, otherwise create it)
ssh uc1                                      # key-based login is already set up
```

**3. Prepare the environment on the server**

```bash
nvidia-smi                                   # check that the GPU is idle and no Isaac Sim process is left over
cd ~/isaac-drone-sim
git pull
source ~/isaacsim-6.1/env_isaacsim/bin/activate
python -m isaac_drone validate --config configs/helix.yaml   # optional: checks the config and assets in a few seconds
```

**4. Start Isaac Sim and load the scene and the drone**

```bash
mkdir -p runs && PYTHONUNBUFFERED=1 PUBLIC_IP=<SERVER_PUBLIC_IP> python -m isaac_drone run --backend isaaclab \
    --config configs/helix.yaml --livestream 1 --wait-for-start 2>&1 | tee runs/helix_$(TZ=America/New_York date +%m%d-%H%M).log
```

The log name and the run directory name use the same clock (US Eastern Time). The server's own clock is UTC+8, so without `TZ=` the two would differ by 12–13 hours.

Wait until the terminal shows `Scene ready: connect the stream client, then press Enter here to start.` This usually takes about 1 minute. The first run after power-on has to compile shaders and may take 2–3 minutes. At this point the drone sits on the ground and the physics simulation does not advance.

**5. Connect WebRTC**: in the client, enter Server `<SERVER_PUBLIC_IP>`, Signal `49100`, Stream `47998`, and click Connect. You will then see the control page and the drone.

**6. Start the run**: click the terminal where the command is running so that it has the input focus, then press **Enter**. The task lasts 20 s of simulation time and plays back in real time by default. If the physics and recording are slower than real time, playback slows down accordingly (measured at about 0.3× when streaming from uc1, about 70 s; see `playback` in the output). When the run ends, it prints `{"success": ...}`, the program exits, and the stream disconnects. To fly again, repeat step 4.

To change the trajectory or the gains, just change `--config` or add `--set`, for example `--config configs/helix_moderate.yaml` or `--set vehicle.motor_model_enabled=true`. For batch runs without display, use `--visualizer none` (`--wait-for-start` is not needed when not streaming).

**7. Check the results and power off**

```bash
python -m isaac_drone summarize              # per-phase error, tilt, motor speed, and saturation ratio of the latest run
python -m isaac_drone runs --last 5          # config, video, physics settings, and results of the last 5 runs
ls runs/<run directory name>/plots/          # English plots generated automatically after the flight; redraw: python -m isaac_drone plot
ls runs/videos/                              # runs that recorded video, one tiled MP4 per run, named after the run directory
```

To copy the results back, run `rsync -az --info=progress2 uc1:isaac-drone-sim/runs/ runs/` in the root of the local repository. This command incrementally syncs all run records to the local `runs/`, including `runs/videos/`. `runs`, `summarize`, `plot`, and `compare` also work locally. Power off the instance in the console when the server is not in use.

## Notes

- Do not run the standalone Isaac Sim streaming app (`isaacsim.exp.full.streaming`) at the same time, because both use port 49100 and only one process can hold this port at a time.
- Use `Ctrl+C` to stop the program. `Ctrl+Z` only suspends the process, and it still holds GPU memory and the port.
- Use `Ctrl+Shift+V` to paste in the terminal.
- WebRTC accepts only one client at a time. If it does not connect, close all old client windows first and then connect again.
- Isaac Lab 3.0 has removed `--headless`. Use `--visualizer none` to run without display.
