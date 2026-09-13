# robotx-navigation

RobotX UAV buoy detection. HSV color pipeline on a Jetson, GPS projection from a nadir camera, MAVLink telemetry to a Mac ground station, and a full Gazebo SITL simulation stack.

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
| `simulink/` | MATLAB/Simulink sensor noise models (GPS, rangefinder, optical flow), an upgrade over the plain-Python noise stand-in in `simulation/mock_fix_publisher.py` |
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
| `--fx-px` | None (uses `--calibration-file`) | Override focal length in pixels; only needed if not using calibration |
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
# Headless - terminal progress updates (no windows)
bash simulation/run_course.sh --course 1   # or 2 or 3

# Visual - Gazebo 3D view + SITL console + camera detector + live GPS coords
bash simulation/run_course.sh --course 1 --visual
```

All outputs (detections, accuracy report, GPS map, logs) save to `simulation/sim_tests/run_N/` automatically after each run. See [simulation/README.md](simulation/README.md) for full details.

---

## Sensor Realism: Simulink vs. Plain Python

The simulation stack needs noisy, realistic sensor data, not the perfect
ground truth Gazebo produces by default. This repo has two ways to generate
that noise, and the Simulink one is the better of the two.

The original approach is a plain Python script,
[`simulation/mock_fix_publisher.py`](simulation/mock_fix_publisher.py). It
fakes GPS noise with a hand-written Markov chain and sigma values picked by
hand. It works, and it's still what the automated Docker tests use, but
there's no real sensor model behind the numbers, and every new sensor type
means writing a new noise heuristic from scratch.

The [`simulink/`](simulink/) folder replaces that with real MATLAB
Navigation Toolbox `gpsSensor` blocks, matching actual GPS receiver quality
tiers (RTK-fixed, RTK-float, DGPS, Standard), plus two more sensor models
built the same way: a rangefinder and an optical flow sensor. All three
share one block-diagram architecture, so adding a sensor means building a
new diagram on the same pattern, not writing new noise code. The GPS model
has been flown live end to end: ArduPilot's own EKF, fed the Simulink-noisy
GPS, tracked a real flight to within 0.06% of the actual distance flown.
See [`simulink/README.md`](simulink/README.md) for the full writeup,
including the real block diagrams and the bugs found and fixed along the
way.

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
