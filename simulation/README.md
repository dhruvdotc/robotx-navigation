# Gazebo Harmonic RobotX UAV Simulation

An ArduPilot-SITL drone flies a nadir camera over RobotX-spec buoy courses on an animated VRX ocean. The real `camera_live_feed.py` detector runs live against the camera topic and projects every buoy detection to GPS. Three distinct courses test different detection scenarios.

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
bash simulation/run_sim_test.sh --course 1
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
bash simulation/run_sim_test.sh --course 2
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
bash simulation/run_sim_test.sh --course 3
```

---

## Sim Tests - Automated Output Collection

`run_sim_test.sh` launches everything in one command and saves all outputs to `simulation/sim_tests/run_N/` (auto-incrementing run number):

```bash
bash simulation/run_sim_test.sh --course 1          # headless + auto fly
bash simulation/run_sim_test.sh --course 2 --gui    # Gazebo window + screen recording
bash simulation/run_sim_test.sh --course 3 --no-fly # launch only, fly manually
```

**Flags:**

| Flag | Effect |
|------|--------|
| `--course 1/2/3` | Select course world and flight path (default: 1) |
| `--gui` | Show Gazebo window instead of headless server |
| `--no-fly` | Start sim but don't auto-launch fly_course.py |
| `--no-record` | Skip ffmpeg screen recording even if available |

**Each `run_N/` folder contains:**

| File | Contents |
|------|----------|
| `detections.csv` | Per-frame buoy detections with GPS estimates |
| `accuracy_report.md` | Cross-referenced vs ground-truth buoy positions |
| `summary.json` | Machine-readable metrics: duration, mean error, buoys found, pass/fail, course |
| `gz.log` | Gazebo and SITL stdout/stderr |
| `verify.log` | accuracy_verify.py console output |
| `map.png` | Top-down detection map: detected vs GT positions, error lines, runtime |
| `recording.mp4` | Screen recording (--gui mode + ffmpeg available) |

A minimum 15-second sustained flight is enforced. Short runs are flagged in the report but the folder is still saved.

---

## Run Manually (advanced)

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

Obstacles are placed outside the gate corridors (|lateral offset| > 3 m from each course centreline) so they do not occlude the actual gates.

## Technical Notes

- ogre2 ignores the camera `<distortion>` block ("ImageBrownDistortionModel is not supported in ogre2") so the render is a clean pinhole. Run with `--no-undistort` - all launchers already do this.
- Do not reboot the flight controller in place during a run - it breaks gz lockstep. Restart both processes together.
- "ArduPilot controller has reset" a couple of times at startup is normal. A continuous loop is not.
- Green buoys use a spring-green emissive (OpenCV hue ~81) to stay within the HSV detector's green range (75-99) without bleeding into blue (90-114).

---

## Legacy

The earlier flat-ocean scene (`ucsd_robotx_demo`, `run_end_to_end.sh`, `verify_sim_topics.sh`) is superseded by the three courses above but kept in the repo for reference.
