# robotx-navigation

CSE145/237D Spring 2026 project. HSV-based buoy and balloon detection for RobotX UAV flights, with GPS projection from a nadir camera and a full Gazebo SITL simulation stack.

Detection classes: `red`, `green` (teal/cyan physical balloon), `blue`.

---

## Real Flight

### Prerequisites

- Ubuntu 22.04 (or WSL Ubuntu-22.04 on Windows)
- Python 3.10 with dependencies installed (see Environment Setup below)
- Drone with a downward-facing camera accessible via OpenCV (`/dev/video*` or USB index)
- ArduPilot flight controller with MAVLink telemetry reachable at a UDP/serial port (for GPS projection)

### Run the detector in flight

```bash
python camera_live_feed.py --camera-index 0 --altitude-m 10
```

Key flags:

| Flag | Default | What it does |
|------|---------|--------------|
| `--camera-index` | 0 | OpenCV camera index |
| `--altitude-m` | 10 | Assumed AGL altitude for GPS projection |
| `--fx-px` | 1500 | Focal length in pixels (match your lens) |
| `--target-diameter-m` | 0.32 | Expected buoy diameter for size gating |
| `--det-width` / `--det-height` | 1920x1080 | Detection resolution |
| `--no-display` | off | Headless mode (no OpenCV window) |
| `--ros-topic` | - | Read frames from a live ROS 2 `sensor_msgs/Image` topic instead of a camera |

Hotkeys during live feed:
- `q` - quit
- `c` - calibrate S/V threshold floor for the selected `--calib-color`

Detection logs (CSV + annotated frames) are written to `detection_logs/`.

### Capture training images

```bash
python camera_capture_spacebar.py --camera-index 0 --output-dir captures --prefix capture
```

Press `Spacebar` to save a frame, `q` to quit.

### Camera calibration

Intrinsics JSON lives in `calibration/camera_intrinsics_latest.json`. Pass `--calibration calibration/camera_intrinsics_latest.json` to `camera_live_feed.py` to enable undistortion. If your camera has no significant distortion, use `--no-undistort`.

---

## Scripts

| Script | Purpose |
|--------|---------|
| `camera_live_feed.py` | Main detector - HSV contour detection, Kalman tracking, GPS projection, detection logging |
| `camera_capture_spacebar.py` | Capture training images by spacebar |
| `hsv_batch_detect.py` | Run HSV detection on a folder of saved images |
| `color_utils.py` | Shared HSV range utilities (imported by other scripts) |
| `augment_test.py` | Robustness demo - runs detector on one image plus 3 UAV-noise augmentations |
| `metrics_summary.py` | Read a detections CSV and print per-class metrics + bar chart |
| `visualize_results.py` | Render a single-slide results diagram |

---

## Gazebo Simulation

A full ArduPilot-SITL simulation of the RobotX navigation course with animated ocean, red/green gate buoys, and a nadir camera on the drone. See [simulation/README.md](simulation/README.md).

---

## Environment Setup

```bash
conda create -y -n robotx python=3.10 opencv numpy pip
conda activate robotx
pip install -r requirements.txt   # if present, else: pip install pymavlink
```

Or with the system Python on Ubuntu:

```bash
pip3 install opencv-python numpy pymavlink
```

### Camera permission (macOS)

Go to System Settings -> Privacy & Security -> Camera and enable access for Terminal / iTerm / Cursor. If it still fails:

```bash
tccutil reset Camera
```
