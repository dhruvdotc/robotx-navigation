# Project Summary: RobotX UAV Buoy Detection System
**CSE145/237D Spring 2026 — Team dhruvdotc / saxysteph**

---

## What We Built

A complete UAV buoy detection system for the Maritime RobotX 2026 competition, consisting of two integrated parts: a real-world flight detection pipeline running on a Jetson embedded computer, and a full Gazebo simulation stack for testing and tuning the detector before competition.

---

## Real-World Flight Pipeline

### The Problem
In Maritime RobotX, a UAV drone flies overhead and must identify red and green navigation buoys from a nadir (straight-down) camera, estimate their GPS coordinates, and relay that information to the surface vehicle. The detection must be fast enough to run onboard a Jetson in real time at flight altitude.

### Our Approach: HSV Color Detection
We chose an HSV (Hue-Saturation-Value) based pipeline over YOLO/deep learning because:
- No GPU inference delay on embedded hardware
- Fully tunable to the exact physical balloon/buoy colors in the competition
- Works deterministically without needing a large labeled training dataset

**Detection classes:** `red`, `green` (teal/cyan physical balloon), `blue`

### How It Works (`camera_live_feed.py`)
1. Frame comes in from the camera (USB, or ROS 2 topic in sim)
2. CLAHE histogram equalization applied to the V channel to normalize exposure
3. HSV color ranges applied per class to build candidate masks
4. Contours extracted, filtered by area, circularity, and solidity
5. ROI cropping and morphological cleanup per candidate
6. Kalman filter tracker assigns IDs and smooths detections across frames
7. Each confirmed detection is projected from pixel coordinates to GPS using:
   - Drone altitude (from MAVLink telemetry)
   - Camera intrinsics (calibrated, stored in `calibration/camera_intrinsics_latest.json`)
   - Nadir geometry (straight-down projection, no gimbal angle required)
8. GPS coordinates sent over MAVLink UDP to the Mac ground station

### Architecture
```
Jetson (onboard)                       Mac (ground station)
  camera → camera_live_feed.py           mavlink_comms/
    HSV detect → GPS project               UDP receiver
    MAVLink → buoy positions  ─────────►   display / log
```

The `fulldemo/` folder contains one-command launch scripts for both sides. The Jetson runs `run_detection_jetson.sh`, the Mac runs `run_gcs_mac.sh`. WiFi router bridges them on the field.

---

## Gazebo Simulation Stack

### Motivation
We needed a way to:
1. Test the detection pipeline against a realistic camera feed before flying
2. Quantify GPS projection accuracy with known ground-truth buoy positions
3. Tune false-positive rejection against non-buoy objects

### Infrastructure
- **Simulator:** Gazebo Harmonic with the gz-sim physics and sensor pipeline
- **Flight controller:** ArduPilot SITL (Software In The Loop) via FDM UDP lockstep — the same ArduCopter firmware that runs on the real drone, just simulated
- **Ocean:** VRX (Virtual RobotX) animated Gerstner-wave ocean (`coast_waves` model, `libWaveVisual.so`). Moving water with ripple texture makes the scene visually realistic from nadir
- **Drone model:** `iris_uav` — stock iris airframe with a fixed nadir camera pod (`gimbal_nadir`), 1920x1080 with calibration-matched intrinsics, publishing `/drone/camera`
- **ROS bridge:** `ros_gz_image` bridges the Gazebo camera topic to ROS 2 so `camera_live_feed.py` can subscribe to it exactly as it would on the real drone

### Accuracy Verification (`accuracy_verify.py`)
Runs live during a simulated flight:
- Subscribes to `/drone/camera` and runs the full detection pipeline
- Reads drone pose (position + attitude) over MAVLink in a background thread
- For every level frame (|roll|, |pitch| < 1.5°), projects each detection to an absolute GPS coordinate using live altitude
- Logs to `simulation/accuracy_logs/detections_<ts>.csv` in real time
- On flight end, cross-references every detection against ground-truth buoy positions parsed directly from the world `.sdf` file
- Enforces a 15-second minimum sustained flight duration before accepting results
- Writes a Markdown report and a `summary.json` with: mean GPS error (m), max error, per-buoy breakdown, detection count, confidence

Typical result on the 3-gate course at 10 m AGL: **6/6 colour buoys detected, mean GPS error ~0.15 m**

### Detection Map Diagram (`plot_run.py`)
Auto-generated after every sim run — a top-down map showing:
- **Ground truth positions** as hollow coloured circles (where buoys actually are)
- **Mean detected position** per buoy as a solid filled circle
- **Individual detection scatter** as faint background dots showing clustering
- **Dashed error lines** from GT to detected mean, labelled with distance in metres
- Compass, gate markers, runtime, mean error in the title bar
- Dark background matching the team's visual style

---

## Three Simulation Courses

Three distinct world files test different aspects of the detection pipeline, all inspired by real Maritime RobotX 2026 task types.

### Course 1: Straight Navigation Channel
**Inspiration:** RobotX 2026 "Safe Passage" (Task 1 — navigate through sequential red/green gate buoys)

Three gate pairs (red/green Sur-Mark cylinders) along a straight East axis at 15 m spacing (within RobotX spec of 7.6–30.5 m). Gate width 2.5 m (spec 1.8–3.0 m). The drone dollies East at 1.5 m/s and pauses 4 s over each gate for clean nadir frames.

- **Tests:** Baseline detection, GPS projection accuracy, confidence calibration
- **7 buoys total:** 3 green, 3 red, 1 light buoy (scan-the-code style black box)

### Course 2: Open Water Survey — Lawnmower
**Inspiration:** RobotX aerial recon / "Scan the Code" — UAV pre-surveys the field before the surface vehicle enters

Seven buoys scattered across a 60 × 30 m open water area with no repeating gate structure. The drone runs a three-strip lawnmower pattern (N = −15, 0, +15 m) at 2 m/s to ensure every buoy falls within 8 m of nadir at 10 m AGL.

- **Tests:** Detection without a predictable layout, lawnmower coverage, handling of buoys at varying lateral offsets
- **7 buoys total:** 3 green, 3 red, 1 light buoy at irregular positions across the field

### Course 3: L-Shaped Dogleg
**Inspiration:** RobotX "Gymkhana" / multi-leg obstacle avoidance — a course with at least one 90-degree turn

Two gates heading East, then a 90-degree right turn, then two more gates heading North. Maritime port/starboard convention maintained on both legs: port = South on leg 1 (red at N = −1.25), port = West on leg 2 (red at E = 33.75). Scan-the-code light buoy at the end of leg 2.

- **Tests:** Detection across different approach headings, GPS projection on the North-heading leg, turn handling
- **9 buoys total:** 4 green, 4 red, 1 light buoy

---

## Distractor Obstacles (False-Positive Tuning)

All three courses contain ~11 floating debris objects designed to generate false-positive candidates in the HSV detector. The goal is to expose weak thresholds and tune them before competition.

### Four Obstacle Types

| Type | Shape | Color | Why It's Challenging |
|------|-------|-------|----------------------|
| Olive-green flat panels | 0.8 × 0.5 × 0.1 m box | Dull olive green, hue ~60–70 | Just below the detector's green range (75–99). Exposes confidence floor and HSV boundary tightness |
| Orange-brown crates | 0.5 × 0.5 × 0.4 m box | Warm tan, hue ~15–25 | Bleeds toward red at lower saturation. Exposes red channel S/V floor |
| Gray flat panels | 0.9 × 0.6 × 0.1 m box | Mid-gray | Tests minimum area and circularity gates |
| Gray barrels | Cylinder r = 0.2 m | Dark gray | Tests shape-based rejection — cylinders look similar to buoys from nadir |

**Key design principle:** No obstacle has an emissive material. Real buoys have `emissive = 1.0` (they glow). Debris relies only on ambient/diffuse lighting, so from nadir it looks duller and flatter. Any detection that fires on these reveals where our thresholds are too loose.

**Course 2 is the hardest:** olive panels sit within 5 m of real green buoys, forcing the detector to distinguish bright spring-green emissive (buoy, hue ~81) from dull olive-green diffuse (debris, hue ~65).

---

## Automated Test Infrastructure

### `run_sim_test.sh`
One-command test runner that creates `simulation/sim_tests/run_N/` and collects all outputs:

```bash
bash simulation/run_sim_test.sh --course 1   # straight channel
bash simulation/run_sim_test.sh --course 2   # lawnmower survey
bash simulation/run_sim_test.sh --course 3   # dogleg
bash simulation/run_sim_test.sh --course 2 --gui   # with Gazebo window + screen recording
```

Auto-increments run number. Each `run_N/` contains:
- `detections.csv` — every per-frame detection with GPS estimate
- `accuracy_report.md` — cross-referenced accuracy vs ground truth
- `summary.json` — machine-readable metrics (run number, course, duration, mean error, buoys found, pass/fail)
- `gz.log` — Gazebo and SITL stdout/stderr
- `map.png` — auto-generated top-down detection diagram
- `recording.mp4` — screen recording if running with `--gui` and ffmpeg

### `fly_course.py`
Drives the SITL drone over a course via MAVLink GUIDED mode. `--course 1/2/3` selects a preset waypoint sequence; `--gates` accepts manual north:east overrides. Handles arming, takeoff, smooth interpolated movement between waypoints, and landing/RTL.

---

## Key Technical Decisions

**Why HSV over YOLO?**
We evaluated YOLO (fine-tuned on Roboflow buoy dataset) vs HSV. YOLO had higher raw mAP on clean images but HSV was more robust to glare, wave reflections, and slight color variation at altitude — and ran 4x faster on Jetson without a GPU inference pipeline. Both model weights are kept in the repo (`yolo11n.pt`, `buoy_balloon_roboflow_best.pt`/`.onnx`) for comparison.

**Why ArduPilot SITL with lockstep?**
Lockstep means Gazebo physics and SITL advance together frame-by-frame. Without it, the camera and the drone's MAVLink telemetry get out of sync, causing GPS projection errors when associating a camera frame to a drone pose. The `eeprom.bin` in the repo root persists `FRAME_CLASS=1` so SITL arms correctly without parameter wipes.

**Why nadir projection instead of stereo/depth?**
At competition altitude (8–12 m AGL), a single calibrated nadir camera plus barometric altitude from the flight controller gives sub-0.3 m GPS accuracy for buoys — well within the scoring tolerance. No depth sensor required.

**Why VRX animated ocean?**
The animated `coast_waves` ocean (Gerstner wave GLSL shader) is the same environment VRX uses for surface vehicle simulation. Training the detector in a realistic moving-water scene reduces the sim-to-real gap compared to a flat static plane.

---

## Files and Structure

```
camera_live_feed.py          Main detector (HSV, Kalman, GPS projection, ROS/camera input)
camera_capture_spacebar.py   Capture training images
color_utils.py               Shared HSV range utilities
calibration/                 Camera intrinsics JSON
fulldemo/                    One-command Jetson + Mac GCS demo scripts
mavlink_comms/               UDP buoy report protocol
simulation/
  gz_env.sh                  Single source of truth for Gazebo env paths
  run_sim_test.sh             One-command sim test runner (course 1/2/3)
  run_demo_windows.sh         3-window live demo launcher for screen recording
  fly_course.py               Scripted MAVLink flight (course presets + manual)
  accuracy_verify.py          Live detection accuracy verifier + report writer
  plot_run.py                 Top-down detection map generator
  gazebo/worlds/
    robotx_uav_course.sdf     Course 1: straight nav channel
    course_2_search_field.sdf Course 2: scattered open-water survey
    course_3_dogleg.sdf       Course 3: L-shaped multi-leg
  gazebo/models/
    iris_uav/                 ArduPilot iris with nadir camera
    gimbal_nadir/             Fixed downward camera pod
  sim_tests/run_N/            Per-run outputs (gitignored)
```

---

## Results

- Detector runs at ~28 fps on the Jetson (headless, 960×540 detection resolution)
- Sim accuracy on Course 1: 6/6 colour buoys, mean GPS error ~0.15 m at 10 m AGL
- False positives on obstacle-heavy courses: 2–4 per run at default thresholds, reducible to 0 by tightening HSV saturation floor from 0.12 to 0.20
- Full pipeline latency (frame in → GPS coordinate out): ~35 ms on Jetson
