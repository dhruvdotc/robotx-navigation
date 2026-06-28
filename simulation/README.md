# Gazebo Harmonic RobotX UAV Simulation

An ArduPilot-SITL drone flies a nadir camera over RobotX-spec buoy courses on an animated VRX ocean. The real `camera_live_feed.py` detector runs live against the camera topic and projects every buoy detection to GPS. Three distinct courses test different detection scenarios.

---

## Quick Start

### Headless — terminal progress updates, no windows

```bash
bash simulation/run_course.sh --course 1   # straight channel  (~60 s)
bash simulation/run_course.sh --course 2   # lawnmower survey  (~3 min)
bash simulation/run_course.sh --course 3   # L-shaped dogleg   (~90 s)
```

Runs entirely in the terminal. Progress is printed every 5 seconds with a
percentage and waypoint label; a milestone line appears when each waypoint is
reached. All outputs auto-saved to `simulation/sim_tests/run_N/` on completion.

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
bash simulation/run_course.sh --course 2 --visual
bash simulation/run_course.sh --course 3 --visual
```

Requires WSLg / X11 (`$DISPLAY` set) and `xterm`. Opens four windows:

| Window | What you see |
|--------|-------------|
| **Gazebo 3D view** | Animated VRX ocean + gate buoys + drone flight |
| **SITL console** | Live ArduCopter arm / mode / GPS log |
| **Camera detector** | `camera_live_feed.py` text detections + OpenCV overlay window |
| **GPS coordinates** | Live lat / lon / alt AGL / speed / mode (updates every second) |

Auto-fly begins 10 seconds after GPS fix so you have time to arrange windows.
All outputs auto-saved to `simulation/sim_tests/run_N/` on Ctrl-C or flight end.

### Extra flags (both modes)

| Flag | Effect |
|------|--------|
| `--no-fly` | Start sim without auto-flight; fly manually with `fly_course.py` |
| `--speed N` | Transit speed in m/s (default 1.5) |

---

## Run Output Folder

Every run saves to `simulation/sim_tests/run_N/` (N auto-increments):

| File | Contents |
|------|----------|
| `detections.csv` | Per-frame buoy GPS projections |
| `accuracy_report.md` | Cross-referenced vs ground-truth buoy positions |
| `summary.json` | Machine-readable metrics: duration, mean error, buoys found |
| `gz.log` | Gazebo + image-bridge stdout/stderr |
| `sitl.log` | ArduPilot SITL + MAVProxy console |
| `fly.log` | fly_course.py output |
| `verify.log` | accuracy_verify.py output |
| `camera.log` | camera_live_feed.py output (visual mode only) |
| `gps.log` | GPS display stream (visual mode only) |
| `map.png` | Top-down detection diagram: detected vs GT positions, error lines |

---

## The Three Courses

### Course 1 - Straight Navigation Channel
**File:** `gazebo/worlds/robotx_uav_course.sdf`
**Task inspiration:** RobotX 2026 "Safe Passage" (Task 1)

Three red/green gate pairs along a straight East axis, plus a scan-the-code light buoy at the end. The drone dollies East at N=0 centreline, pausing over each gate for clean nadir frames. Baseline test for the detector.

| Buoy | East (m) | North (m) | Color |
|------|----------|-----------|-------|
| gate1_green | 10 | +1.25 | green |
| gate1_red   | 10 | -1.25 | red   |
| gate2_green | 25 | +1.25 | green |
| gate2_red   | 25 | -1.25 | red   |
| gate3_green | 40 | +1.25 | green |
| gate3_red   | 40 | -1.25 | red   |
| light_buoy  | 50 |  0    | -     |

**Flight path:** Straight East at N=0, hover 4s per gate.

```bash
bash simulation/run_course.sh --course 1
```

---

### Course 2 - Open Water Survey (Lawnmower)
**File:** `gazebo/worlds/course_2_search_field.sdf`
**Task inspiration:** RobotX "Scan the Code" + pre-race aerial recon

Seven buoys scattered across a 60x30 m open-water field with no channel structure. The drone runs a three-strip lawnmower pattern to survey the whole field. Tests the detector's ability to find and GPS-tag buoys without a predictable layout.

| Buoy | East (m) | North (m) | Color |
|------|----------|-----------|-------|
| green1 |  8 | +10 | green |
| red1   | 14 | -11 | red   |
| green2 | 24 |  -8 | green |
| red2   | 31 |  +6 | red   |
| green3 | 42 |  +7 | green |
| red3   | 48 |  -5 | red   |
| light_buoy | 55 | +2 | -  |

**Flight path:** Three East-West strips at N=-15, N=0, N=+15 (lawnmower). Every buoy falls within 8 m of nadir at 10 m AGL.

```bash
bash simulation/run_course.sh --course 2
```

---

### Course 3 - L-Shaped Dogleg
**File:** `gazebo/worlds/course_3_dogleg.sdf`
**Task inspiration:** RobotX 2026 "Gymkhana" / multi-leg obstacle course

Two-leg L-shaped course: two gates going East, then a 90-degree right turn and two more gates going North. Maritime port/starboard convention is maintained on both legs. Tests the detector across two different approach headings.

| Buoy | East (m) | North (m) | Color | Leg |
|------|----------|-----------|-------|-----|
| gate1_green | 10   | +1.25 | green | 1 (East) |
| gate1_red   | 10   | -1.25 | red   | 1 (East) |
| gate2_green | 25   | +1.25 | green | 1 (East) |
| gate2_red   | 25   | -1.25 | red   | 1 (East) |
| gate3_green | 36.25 | 15   | green | 2 (North) |
| gate3_red   | 33.75 | 15   | red   | 2 (North) |
| gate4_green | 36.25 | 30   | green | 2 (North) |
| gate4_red   | 33.75 | 30   | red   | 2 (North) |
| light_buoy  | 35    | 42   | -     | end      |

**Flight path:** East to corner at (E=35, N=0), then pivot North to (E=35, N=42).

```bash
bash simulation/run_course.sh --course 3
```

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

Opens three windows: Gazebo GUI, MAVProxy console xterm, and camera_live_feed.py xterm. When it prints READY:

```bash
python3 simulation/fly_course.py --course 1    # or 2 or 3
```

**Fly a specific course manually:**

```bash
# Course 1: straight channel
python3 simulation/fly_course.py --course 1 --connect udp:127.0.0.1:14550

# Course 2: lawnmower survey
python3 simulation/fly_course.py --course 2 --connect udp:127.0.0.1:14550 --speed 2.0

# Course 3: L-shaped dogleg
python3 simulation/fly_course.py --course 3 --connect udp:127.0.0.1:14550
```

**Verify detection accuracy during a flight:**

```bash
python3 simulation/accuracy_verify.py --connect udp:127.0.0.1:14551
```

Subscribes to `/drone/camera`, runs the detection pipeline live, and writes a cross-referenced accuracy report on exit.

**Generate or regenerate the detection map:**

```bash
python3 simulation/plot_run.py                   # latest run
python3 simulation/plot_run.py --run 3           # specific run number
python3 simulation/plot_run.py simulation/sim_tests/run_5
```

---

## Prerequisites (Ubuntu 22.04 / WSL Ubuntu-22.04)

- ROS 2 Humble (`/opt/ros/humble`), Gazebo Harmonic (`gz`), `ros_gz_image`
- ArduPilot SITL built at `~/ardupilot`, plus `ardupilot_gazebo` plugin at `~/ardupilot_gazebo`
- VRX built in `~/vrx_ws` (supplies `coast_waves` and the wave plugins). Override with `VRX_GZ=<path>` if installed elsewhere.

`gz_env.sh` is the single source of truth for `GZ_SIM_*` resource and plugin paths. It is sourced by all launchers - do not set these paths manually.

---

## Distractor Obstacles

All three courses contain floating debris objects designed to generate false-positive candidates for the HSV detector. Each course has ~11 distractors in three categories:

| Type | Shape | Color | HSV challenge |
|------|-------|-------|---------------|
| Olive-green panels | Flat box ~0.8x0.5x0.1 m | Dull olive green | Hue ~60-70, just below the detector's green range (75-99). Borderline false positive. |
| Orange-brown crates | Box ~0.5x0.5x0.4 m | Warm tan/brown | Hue ~15-25, warm tones that can bleed into the red range at low saturation. |
| Gray barrels | Cylinder r=0.15-0.2 m | Mid-gray | Neutral distractor, tests shape-based filtering. |
| Gray flat panels | Flat box ~0.9x0.6x0.1 m | Medium gray | Neutral distractor, tests size gating. |

Key difference from real buoys: **no emissive material**. Real buoys have a bright emissive component; obstacles rely only on ambient/diffuse lighting, so they look duller from nadir. The detector should suppress them via confidence thresholding and size gating, but the olive panels in particular will stress-test the green HSV range boundary.

Obstacles are placed clear of the gate corridors (so they never occlude a gate) yet inside the nadir camera's swept footprint, so they actually appear in frame. Because the drone holds a North heading (`WP_YAW_BEHAVIOR=0`), the binding cross-track reach is the camera's *vertical* FOV — only ±4.1 m at 10 m AGL — for legs flown East (Course 1, Course 2 strips, Course 3 leg 1), and the *horizontal* FOV (±7.3 m) for the North-bound Course 3 leg 2. Distractors are offset ~3 m laterally on East legs and ~4 m on the North leg. (Earlier versions sat at 5–12 m offsets, outside every frame.)

## Technical Notes

- ogre2 ignores the camera `<distortion>` block ("ImageBrownDistortionModel is not supported in ogre2") so the render is a clean pinhole. Run with `--no-undistort` - all launchers already do this.
- Do not reboot the flight controller in place during a run - it breaks gz lockstep. Restart both processes together.
- "ArduPilot controller has reset" a couple of times at startup is normal. A continuous loop is not.
- Green buoys use a spring-green emissive (OpenCV hue ~81) to stay within the HSV detector's green range (75-99) without bleeding into blue (90-114).

---

## Legacy

The earlier flat-ocean scene (`ucsd_robotx_demo`, `run_end_to_end.sh`, `verify_sim_topics.sh`) is superseded by the three courses above but kept in the repo for reference.
