# robotx-navigation

CSE145/237D Spring 2026 - RobotX UAV buoy detection. HSV color pipeline on a Jetson, GPS projection from a nadir camera, MAVLink telemetry to a Mac ground station, and a full Gazebo SITL simulation stack.

Detection classes: `red`, `green` (teal/cyan physical balloon), `blue`.

---

## Real Flight - Quick Start

**Mac (Terminal 1):**
```bash
bash fulldemo/run_gcs_mac.sh
```

**Jetson (Terminal 2):**
```bash
ssh babydragon@<JETSON_IP>
cd ~/robotx-navigation
GCS_IP=<MAC_IP> bash fulldemo/run_detection_jetson.sh
```

See `fulldemo/README.md` and `fulldemo/PARTNER_INSTRUCTIONS.md` for WiFi router setup, tuning, and troubleshooting.

---

## Repo Layout

| Path | Purpose |
|------|---------|
| `camera_live_feed.py` | Main detector - HSV contour detection, Kalman tracking, GPS projection, MAVLink, ROS topic input |
| `camera_capture_spacebar.py` | Capture training images by spacebar |
| `fulldemo/` | One-command Mac/Jetson demo scripts |
| `mavlink_comms/` | UDP buoy protocol and ground station |
| `scripts/` | Jetson WiFi helpers |
| `simulation/` | Gazebo Harmonic SITL simulation stack |
| `calibration/` | Camera intrinsics JSON |
| `jetson_setup.sh` | Jetson dependency bootstrap |
| `yolo11n.pt` | YOLO nano model weights |

---

## Real Flight - Manual Operation

Run the detector directly on the Jetson with a connected camera:

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
| `--ros-topic` | - | Read frames from a ROS 2 sensor_msgs/Image topic instead of a camera |
| `--no-undistort` | off | Skip undistortion (use if no significant lens distortion) |

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

Intrinsics JSON lives in `calibration/camera_intrinsics_latest.json`. Pass `--calibration calibration/camera_intrinsics_latest.json` to enable undistortion.

---

## Gazebo Simulation

A full ArduPilot-SITL simulation of the RobotX navigation course with animated ocean, red/green gate buoys, and a nadir camera on the drone.

**Two ways to run any of the 3 courses:**

```bash
# Headless — terminal progress updates (no windows)
bash simulation/run_course.sh --course 1   # or 2 or 3

# Visual — Gazebo 3D view + SITL console + camera detector + live GPS coords
bash simulation/run_course.sh --course 1 --visual
```

All outputs (detections, accuracy report, GPS map, logs) save to `simulation/sim_tests/run_N/` automatically after each run. See [simulation/README.md](simulation/README.md) for full details.

---

## Environment Setup

```bash
conda create -y -n robotx python=3.10 opencv numpy pip
conda activate robotx
pip install pymavlink
```

Or on Ubuntu/Jetson:

```bash
pip3 install opencv-python numpy pymavlink
```

### Camera permission (macOS)

Go to System Settings -> Privacy & Security -> Camera and enable access for Terminal / iTerm / Cursor. If it still fails:

```bash
tccutil reset Camera
```
