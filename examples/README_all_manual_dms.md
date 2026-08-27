# all_manual_dms.py — CARLA Manual Control + Driver Monitoring System

`all_manual_dms.py` is a modified version of CARLA's classic `manual_control_steeringwheel.py`
example. It adds a real-time **Driver Monitoring System (DMS)** built on an Intel RealSense
camera and three AI models, displayed as a picture-in-picture overlay while you drive with a
Logitech racing wheel:

| Mode | Model | What it shows |
|---|---|---|
| **Gaze** | TGGNet (graph transformer, PyTorch Geometric) | Gaze direction arrows + compass overlay |
| **Anti-Spoofing** | ShuffleNetV2 hybrid depth-guided classifier | Live/spoof face liveness score |
| **Action** | ViFi-CLIP (video-CLIP, 16 driver-action classes) | Current driver action + danger highlighting |
| **ALL** | All three | All overlays stacked at once |

You cycle between modes at runtime with a single key press — nothing to restart.

---

## 1. Hardware requirements

- **A Logitech racing wheel** (configured for G920 by default — see [Wheel config](#4-wheel-configuration)).
  `DualControl.__init__` calls `pygame.joystick.Joystick(0)` unconditionally, so **the script
  will crash on startup if no joystick is connected.** There is currently no keyboard fallback.
- **An Intel RealSense camera** (tested with a D400-series depth camera — needs color, depth,
  and infrared streams). Pass `--no-realsense` to skip it and run plain manual control instead.
- **An NVIDIA GPU is strongly recommended.** All three models auto-select
  `cuda` if `torch.cuda.is_available()`, otherwise fall back to CPU (Action recognition on CPU
  will be very slow — it runs a CLIP video transformer over 32-frame clips).
- CARLA server (`CarlaUnreal.sh`) running and reachable (default `127.0.0.1:2000`).

## 2. Software / environment

The script has a `conda` environment shebang baked in:

```
#!/home/michael/anaconda3/envs/carlair-env/bin/python
```

Activate that environment (or your own equivalent) before running so the compiled
`carla` module and every ML dependency resolve correctly:

```bash
conda activate carlair-env
```

Python packages required beyond the standard CARLA client deps (`carla`, `numpy`, `pygame`):

```
opencv-python (cv2)
pyrealsense2
torch
torch_geometric
mediapipe
networkx
```

Install the CARLA wheel first if you haven't already (see the main repo README), then the rest:

```bash
pip install opencv-python pyrealsense2 torch torch_geometric mediapipe networkx
```

(Use a `torch`/`torch_geometric` build matching your CUDA version if you want GPU inference.)

### External model assets

The script loads checkpoints from **hardcoded absolute paths** — these must exist on disk or
the corresponding processor silently disables itself (a warning is printed, the script keeps
running without that overlay):

| Processor | Path(s) it expects |
|---|---|
| Gaze | `../../GazeTGGNet-main/Demo/trained_model_No_Or.pt` and `face_landmarker.task` (relative to the script) |
| Anti-Spoofing | `_SPOOF_ROOT = /home/michael/LFAS-NewBackbone Ablation Study` (repo providing `model.build_model` / `realtime_test.py`) and checkpoint `/home/michael/CARLA_UE5/Face Anit-Spoofing/test_min_acer_model_20260307_13_14_45.pth` |
| Action | `_VIFI_ROOT = /home/michael/CARLA_UE5/ViFi-CLIP` (for `utils.config` / `trainers.vificlip`) and checkpoint `/home/michael/CARLA_UE5/ViFi-CLIP/vifi_clip_finetuned_LAST02.pth` |

> ⚠️ **Known issue:** `_SPOOF_ROOT` and `_RT_PATH` (line ~74/241) currently point at
> `/home/michael/LFAS-NewBackbone Ablation Study`, which does not exist on this machine anymore
> — the anti-spoofing model source now lives under
> `/home/michael/CARLA_UE5/Face Anit-Spoofing/realtime_test.py`. Until those paths are updated,
> **Anti-Spoofing mode will fail to load** (you'll see
> `[SpoofingProcessor] realtime_test load failed: ...` and the game keeps running without it —
> Gaze and Action still work). Fix by editing the two paths in `all_manual_dms.py` to point at
> the current location, or restoring the old directory.

If you're running on a machine without these extra research repos, that's fine — each processor
fails independently and gracefully; you'll just be left with plain manual control (or whichever
subset of Gaze/Anti-Spoofing/Action does resolve).

## 3. Running it

1. Start the CARLA server:
   ```bash
   ./CarlaUnreal.sh
   ```
2. In another terminal, with the `carlair-env` environment active and from
   `PythonAPI/examples/`:
   ```bash
   python all_manual_dms.py
   ```

### Useful flags

```bash
python all_manual_dms.py --host 127.0.0.1 -p 2000 \
    --res 2560x1440 \
    --filter vehicle.taxi.ford \
    --no-realsense
```

| Flag | Default | Meaning |
|---|---|---|
| `--host` | `127.0.0.1` | CARLA server address |
| `-p`, `--port` | `2000` | CARLA server port |
| `-a`, `--autopilot` | off | Start the vehicle in autopilot |
| `--res` | `2560x1440` | Render resolution (`WxH`) |
| `--display-res` | same as `--res` | Physical window/monitor resolution the render surface is scaled up to (e.g. a triple-monitor `7680x1440` setup) |
| `--filter` | `vehicle.taxi.ford` | Blueprint filter for the spawned vehicle |
| `--no-realsense` | off | Skip RealSense/DMS entirely and just drive |

Output videos and frame dumps are written to `output/` (created automatically) and `_out/`.

## 4. Wheel configuration

`wheel_config.ini` (same directory) maps physical wheel axes/buttons to controls:

```ini
[Logitech G920 Driving Force Racing Wheel]
steering_wheel = 0
throttle = 1
brake = 2
reverse = 8
```

If you're using a different wheel, find its axis/button indices (e.g. with
`jstest` or `pygame.joystick`) and edit this file — the section header itself is
also read literally by the script, so keep the name as-is unless you edit
`DualControl.__init__` to match.

To drive, **press the brake pedal once** before applying throttle (per the in-app
help text) — this seats the pedal's resting axis value correctly.

## 5. Controls

| Key | Action |
|---|---|
| Steering wheel / pedals | Steer / throttle / brake (see `wheel_config.ini`) |
| Wheel button (`reverse` index) | Toggle reverse gear |
| **M** | Cycle DMS overlay mode: Gaze → Anti-Spoofing → Action → ALL → Gaze… |
| **T** | Start/stop recording **both** the CARLA camera and RealSense feed to `output/*.mp4` |
| **R** | Toggle per-frame CARLA image dump to `_out/` |
| **P** | Toggle autopilot |
| **C** | Cycle weather preset |
| **TAB** | Toggle camera view (dashboard cam / hood cam) |
| **`** (backquote) | Cycle sensor (RGB camera / LiDAR) |
| **F1** | Toggle HUD info panel |
| **H** / **?** | Toggle help overlay |
| **Backspace** | Respawn vehicle |
| **Esc** / **Ctrl+Q** | Quit |

The RealSense DMS overlay is shown as a bordered picture-in-picture in the
**top-left corner** of the window, labeled with the current mode.

## 6. Troubleshooting

- **`ValueError: Please connect just one joystick`** — more than one joystick/wheel is plugged
  in; unplug extras.
- **Crash on `pygame.joystick.Joystick(0)`** — no joystick detected at all; connect the wheel
  before launching (no keyboard-only mode exists yet).
- **`[SpoofingProcessor] realtime_test load failed: ...`** — see the known path issue above;
  Anti-Spoofing overlay will just be unavailable, everything else still runs.
- **RealSense init hangs or throws** — check the camera is plugged into a USB3 port and no other
  process (e.g. `realsense-viewer`) is holding the device open.
- **Slow / stuttering Action recognition** — it only re-runs inference every 16 frames once its
  32-frame buffer fills, but a CPU-only PyTorch install will still be noticeably slow; verify
  `torch.cuda.is_available()` returns `True` in your environment.
- **Wrong `carla` module / import errors** — make sure you're using the `carlair-env` conda
  environment (or one with the matching `carla-0.10.0` wheel installed) rather than system Python.
