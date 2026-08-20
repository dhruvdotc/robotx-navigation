# Gazebo Harmonic RobotX UAV Simulation

An ArduPilot-SITL drone flies a nadir camera over RobotX-spec buoy courses on an animated VRX ocean. The real `camera_live_feed.py` detector runs live against the camera topic and projects every buoy detection to GPS. Three distinct courses test different detection scenarios.

---

## Quick Start

### Headless — terminal progress, no windows

```bash
bash simulation/run_course.sh --course 1   # straight channel  (~60 s)
bash simulation/run_course.sh --course 2   # lawnmower survey  (~5 min)
bash simulation/run_course.sh --course 3   # L-shaped dogleg   (~90 s)
```

Sample output:
```
[10:24:22] === FLIGHT IN PROGRESS | Course 1: Straight Navigation Channel | 4 waypoints ===

[10:24:27]   0% | 0/4 done | waiting for GPS / arming...
[10:24:32]   0% | 0/4 done | armed -- climbing to altitude...
[10:24:40]   0% | 0/4 done | flying to: gate 1
[10:24:48]  25% | waypoint 1/4 reached: gate 1
[10:24:53]  25% | 1/4 done | flying to: gate 2
[10:25:06]  50% | waypoint 2/4 reached: gate 2
[10:25:21]  75% | waypoint 3/4 reached: gate 3
[10:25:34] 100% | waypoint 4/4 reached: light buoy
```

### Visual — 4 windows open simultaneously

```bash
bash simulation/run_course.sh --course 1 --visual
```

Requires WSLg / X11 (`$DISPLAY` set) and `xterm`. Opens four windows:

| Window | What you see |
|--------|-------------|
| **Gazebo 3D view** | Animated VRX ocean + gate buoys + drone flight |
| **SITL console** | Live ArduCopter arm / mode / GPS log |
| **Camera detector** | `camera_live_feed.py` text detections + OpenCV overlay window |
| **GPS coordinates** | Live lat / lon / alt AGL / speed / mode (updates every second) |

Auto-fly begins 10 seconds after GPS fix. All outputs auto-saved to `simulation/sim_tests/run_N/`.

### Extra flags

| Flag | Effect |
|------|--------|
| `--no-fly` | Start sim without auto-flight; fly manually with `fly_course.py` |
| `--speed N` | Transit speed in m/s (default 1.5) |

---

## Detector Modes

The simulation supports two detector backends, switchable without code changes.

### HSV-only (default)

Two-stage pipeline: color-agnostic blob detection → HSV threshold inside each blob's ROI to classify color. No neural network. Fast, interpretable, no model file needed.

```bash
bash simulation/run_course.sh --course 1
```

### YOLO (fine-tuned, recommended for competition)

Single-stage neural network predicts bounding box **and** color class simultaneously. Trained on 1300 images with the full preprocessing pipeline. Validated accuracy: **mAP50 = 0.995**, recall = 1.000 (zero missed buoys), all three colors.

```bash
YOLO_MODEL=yolo_comparison_test/path2_switch_proposal/scripts/training/balloon_proper/weights/best.pt \
  bash simulation/run_course.sh --course 1
```

The `camera_live_feed.py` instance inside the sim automatically applies `color_normalize()` (CLAHE → Gray-World WB → unsharp mask) to every frame before YOLO inference — the same pipeline used during training, so no preprocessing mismatch.

### Side-by-side HSV vs YOLO comparison

```bash
YOLO_MODEL=path/to/best.pt bash simulation/run_course.sh --course 2
```

When `YOLO_MODEL` is set, `run_course.sh` launches **two** `accuracy_verify.py` instances in parallel — one HSV (port 14551), one YOLO (port 14553) — writing separate output files so both can be compared from the same flight:

| File | Detector |
|------|---------|
| `accuracy_report.md` / `summary.json` | HSV |
| `accuracy_report_yolo.md` / `summary_yolo.json` | YOLO |

To run YOLO-only (skip HSV entirely):
```bash
SKIP_HSV=1 YOLO_MODEL=path/to/best.pt bash simulation/run_course.sh --course 2
```

### YOLO confidence threshold

```bash
YOLO_MODEL=path/to/best.pt YOLO_CONF=0.35 bash simulation/run_course.sh --course 1
```

Default is 0.25. Raising to 0.35–0.40 reduces the ~10% red false-positive rate at the cost of slightly lower recall on borderline detections.

### Jetson competition deployment — TensorRT export

Export once on the Jetson before competition day for ~3× speedup:

```bash
# On the Jetson (takes ~2 minutes)
yolo export model=best.pt format=engine device=0 imgsz=640

# Then use best.engine instead of best.pt
YOLO_MODEL=/path/to/best.engine bash simulation/run_course.sh --course 1
```

YOLOv11n with TensorRT on Jetson Orin Nano: ~5–8 ms inference vs ~25 ms with plain PyTorch. Total pipeline (preprocessing + inference) stays under 30 ms.

---

## Validated Accuracy (YOLO mode)

Measured on 65 held-out images never seen during training (raw-level 80/20 split):

| Metric | Value |
|---|---|
| mAP50 | **0.995** |
| Precision | 0.922 |
| Recall | **1.000** (zero missed buoys) |
| F1 | 0.959 |

Per-class:

| Class | Precision | Recall | F1 |
|---|---|---|---|
| Red | 0.887 | 1.000 | 0.940 |
| Green | 0.947 | 1.000 | 0.973 |
| Blue | **1.000** | **1.000** | **1.000** |

UAV noise stress test (blur + motion + glare + sensor noise applied to val set): **101% detection retention**, 1.3% confidence drop. Overfit check: HEALTHY (val loss < train loss).

GPS reprojection accuracy (Course 1 sim run): **mean error 0.16 m**, max 1.04 m, 6/6 buoys matched.

---

## Run Output Folder

Every run saves to `simulation/sim_tests/run_N/` (N auto-increments):

| File | Contents |
|------|----------|
| `detections.csv` | Per-frame buoy GPS projections (HSV detector) |
| `detections_yolo_<ts>.csv` | Per-frame projections (YOLO detector, when `YOLO_MODEL` set) |
| `accuracy_report.md` | Cross-referenced vs ground-truth buoy positions (HSV) |
| `accuracy_report_yolo.md` | Same for YOLO (when `YOLO_MODEL` set) |
| `summary.json` | Machine-readable metrics: duration, mean error, buoys found (HSV) |
| `summary_yolo.json` | Same for YOLO |
| `map.png` | Top-down detection diagram: detected vs GT positions, error lines |
| `gz.log` | Gazebo + image-bridge stdout/stderr |
| `sitl.log` | ArduPilot SITL + MAVProxy console |
| `fly.log` | fly_course.py output |
| `verify.log` | accuracy_verify.py output (HSV) |
| `verify_yolo.log` | accuracy_verify.py output (YOLO) |
| `camera.log` | camera_live_feed.py output (visual mode only) |
| `gps.log` | GPS display stream (visual mode only) |
| `light_cycler.log` | Scan-the-code light buoy color transitions (red → green → blue) |

---

## The Three Courses

### Course 1 — Straight Navigation Channel
**File:** `gazebo/worlds/robotx_uav_course.sdf`
**Task inspiration:** RobotX "Safe Passage"

Three red/green gate pairs along a straight East axis, plus a scan-the-code light buoy at the end. The drone dollies East at N=0 centreline, pausing over each gate for clean nadir frames. Baseline test for the detector.

| Buoy | East (m) | North (m) | Color |
|------|----------|-----------|-------|
| gate1_green | 10 | +1.25 | green |
| gate1_red   | 10 | −1.25 | red   |
| gate2_green | 25 | +1.25 | green |
| gate2_red   | 25 | −1.25 | red   |
| gate3_green | 40 | +1.25 | green |
| gate3_red   | 40 | −1.25 | red   |
| light_buoy  | 50 |  0    | cycles r/g/b |

**Flight path:** Straight East at N=0, hover 4 s per gate.

---

### Course 2 — Open Water Survey (Lawnmower)
**File:** `gazebo/worlds/course_2_search_field.sdf`
**Task inspiration:** RobotX "Scan the Code" + pre-race aerial recon

Seven buoys scattered across a 60×30 m open-water field. The drone runs a five-strip lawnmower pattern to survey the whole field. Tests the detector's ability to find and GPS-tag buoys without a predictable layout.

| Buoy | East (m) | North (m) | Color |
|------|----------|-----------|-------|
| green1 |  8 | +10 | green |
| red1   | 14 | −11 | red   |
| green2 | 24 |  −8 | green |
| red2   | 31 |  +6 | red   |
| green3 | 42 |  +7 | green |
| red3   | 48 |  −5 | red   |
| light_buoy | 55 | +2 | cycles r/g/b |

**Flight path:** Five East-West strips at N=−12, −6, 0, +6, +12. The drone holds a fixed North heading (`WP_YAW_BEHAVIOR=0`) throughout, so the binding cross-track reach on East-West legs is the camera's *vertical* FOV (~4.1 m at 10 m AGL), not the wider ~7.3 m horizontal one. The old 3-strip layout (N=−15, 0, +15) used the wrong FOV number and left buoys near N=±7/±8 with near-zero margin; 5 strips at 6 m spacing keep every buoy within ~2 m of a strip line.

---

### Course 3 — L-Shaped Dogleg
**File:** `gazebo/worlds/course_3_dogleg.sdf`
**Task inspiration:** RobotX "Gymkhana" / multi-leg obstacle course

Two-leg L-shaped course: two gates going East, then a 90° right turn and two more gates going North. Tests the detector across two different approach headings.

| Buoy | East (m) | North (m) | Color | Leg |
|------|----------|-----------|-------|-----|
| gate1_green |  10   | +1.25 | green | 1 (East) |
| gate1_red   |  10   | −1.25 | red   | 1 (East) |
| gate2_green |  25   | +1.25 | green | 1 (East) |
| gate2_red   |  25   | −1.25 | red   | 1 (East) |
| gate3_green |  36.25 |  15   | green | 2 (North) |
| gate3_red   |  33.75 |  15   | red   | 2 (North) |
| gate4_green |  36.25 |  30   | green | 2 (North) |
| gate4_red   |  33.75 |  30   | red   | 2 (North) |
| light_buoy  |  35    |  42   | cycles r/g/b | end |

**Flight path:** East to corner at (E=35, N=0), then pivot North to (E=35, N=42).

---

## Advanced / Manual Operation

**Basic Gazebo + SITL launch:**

```bash
bash simulation/run_robotx_uav_sitl.sh            # Course 1, Gazebo GUI + SITL
bash simulation/run_robotx_uav_sitl.sh --headless # headless server + SITL
bash simulation/run_robotx_uav_sitl.sh --no-sitl  # Gazebo only
```

SITL MAVLink is at `tcp:127.0.0.1:5760` (FDM UDP 9002). `eeprom.bin` in the repo root persists `FRAME_CLASS=1` so arming works without wiping parameters each launch.

**Full 3-window demo:**

```bash
bash simulation/run_demo_windows.sh
```

Opens Gazebo GUI, MAVProxy console xterm, and camera_live_feed.py xterm. When it prints READY:

```bash
python3 simulation/fly_course.py --course 1    # or 2 or 3
```

**Fly a specific course manually:**

```bash
python3 simulation/fly_course.py --course 1 --connect udp:127.0.0.1:14550
python3 simulation/fly_course.py --course 2 --connect udp:127.0.0.1:14550 --speed 2.0
python3 simulation/fly_course.py --course 3 --connect udp:127.0.0.1:14550
```

**Verify detection accuracy during a flight (HSV):**

```bash
python3 simulation/accuracy_verify.py --connect udp:127.0.0.1:14551
```

**Verify with YOLO detector:**

```bash
python3 simulation/accuracy_verify.py \
  --connect udp:127.0.0.1:14551 \
  --yolo-model path/to/best.pt \
  --yolo-conf 0.25
```

`accuracy_verify.py` applies `color_normalize()` before each YOLO inference call, matching the training preprocessing exactly.

**Generate or regenerate the detection map:**

```bash
python3 simulation/plot_run.py                   # latest run
python3 simulation/plot_run.py --run 3           # specific run number
python3 simulation/plot_run.py simulation/sim_tests/run_5
```

---

## Camera Calibration

GPS projection in the sim uses the same calibration file as the real Jetson:
`calibration/camera_intrinsics_latest.json` (fx=1319.07, fy=1407.50, RMS=1.057 px).

All launchers pass `--no-undistort` because ogre2 renders a clean pinhole (no lens distortion in sim). To plug in a different camera — recalibration or new hardware — replace the JSON or pass `--calibration-file /path/to/new.json`. No source changes needed; all GPS math flows exclusively from that file.

---

## Prerequisites (Ubuntu 22.04 / WSL Ubuntu-22.04)

- ROS 2 Humble (`/opt/ros/humble`), Gazebo Harmonic (`gz`), `ros_gz_image`
- ArduPilot SITL built at `~/ardupilot`, plus `ardupilot_gazebo` plugin at `~/ardupilot_gazebo`
- VRX built in `~/vrx_ws` (supplies `coast_waves` and the wave plugins). Override with `VRX_GZ=<path>` if installed elsewhere.
- For YOLO mode: `pip install ultralytics` (already in `requirements.txt`)

**Mac (Apple Silicon):** native macOS can't run this stack — see [`docs/01_environment_setup.md` → "Mac (simulation)"](../docs/01_environment_setup.md#mac-simulation) for why, and for the Ubuntu-in-a-VM path that does work (headless mode).

`gz_env.sh` is the single source of truth for `GZ_SIM_*` resource and plugin paths. Sourced by all launchers — do not set these paths manually.

---

## Distractor Obstacles

All three courses contain floating debris objects designed to stress-test the detectors. Each course has ~11 distractors in three categories:

| Type | Shape | Color | HSV challenge |
|------|-------|-------|---------------|
| Olive-green panels | Flat box ~0.8×0.5×0.1 m | Dull olive green | Hue ~60–70, just below the green range (75–105). Borderline false positive. |
| Orange-brown crates | Box ~0.5×0.5×0.4 m | Warm tan/brown | Hue ~15–25, bleeds into red range at low saturation. |
| Gray barrels | Cylinder r=0.15–0.2 m | Mid-gray | Neutral distractor, tests shape-based filtering. |
| Gray flat panels | Flat box ~0.9×0.6×0.1 m | Medium gray | Neutral distractor, tests size gating. |

Key difference from real buoys: **no emissive material**. Real buoys have a bright emissive component; distractors rely only on ambient/diffuse lighting, so they appear duller from nadir. YOLO suppresses them by confidence thresholding; HSV suppresses them by shape (solidity, aspect ratio) and size gating.

Obstacles are placed clear of the gate corridors yet inside the camera's swept footprint. Because the drone holds North heading (`WP_YAW_BEHAVIOR=0`), the binding cross-track reach is the camera's *vertical* FOV (±4.1 m at 10 m AGL) for East legs, and the *horizontal* FOV (±7.3 m) for the North-bound Course 3 leg 2. Distractors are offset ~3 m laterally on East legs and ~4 m on the North leg.

---

## Scan-the-Code Light Buoy (color cycling)

Every course ends at a `light_buoy` modeled on the RobotX Scan the Code task: a dark buoy whose light panel cycles red → green → blue every 3 seconds. `run_course.sh` launches `simulation/light_buoy_cycler.py` automatically; the cycler spawns/swaps a small emissive glow-panel model on the buoy top each phase (entity swap instead of material changes, because gz-sim Harmonic accepts `visual_config` material updates but never applies them to the SENSOR render scene).

To collect blue training data the way the real task works, loiter over the buoy so the camera reads full color cycles:

```bash
bash simulation/run_course.sh --course 1 --no-fly
python3 simulation/fly_course.py --gates '0:50' --hover-s 100 --countdown 0 \
    --connect udp:127.0.0.1:14550
# plus camera_live_feed.py --ros-topic /drone/camera in another terminal
```

Color values are tuned to what the sensor actually renders: scene lighting shifts hue ~10–12 points green-ward, and a BRIGHT blue panel is invisible to the detector's grayscale Canny stage against blue-green water. The cycler uses a deliberately dark blue (gray ~20) that keeps hue ~114 and edges strongly.

---

## Technical Notes

- ogre2 ignores the camera `<distortion>` block ("ImageBrownDistortionModel is not supported in ogre2"), so the render is a clean pinhole. All launchers already pass `--no-undistort`.
- Do not reboot the flight controller in place during a run — it breaks gz lockstep. Restart both processes together.
- "ArduPilot controller has reset" a couple of times at startup is normal. A continuous loop is not.
- Green buoys use spring-green emissive (OpenCV hue ~81) to stay within the HSV green range (75–105) without bleeding into blue (100–130).
- Live MAVLink telemetry (`mavlink_telemetry.py`) streams the drone's position to `accuracy_verify.py` so each detection's GPS coordinate is computed from the drone's actual position at that instant, not the position at script startup. This was a key fix for stale GPS projection (Stage 3 bug).

---

## Legacy

The earlier flat-ocean scene (`ucsd_robotx_demo`, `run_end_to_end.sh`, `verify_sim_topics.sh`) is superseded by the three courses above but kept in the repo for reference.
