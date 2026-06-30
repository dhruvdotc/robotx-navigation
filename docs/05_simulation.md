# Simulation

Gazebo Harmonic SITL with ArduCopter, animated VRX ocean, and the real `camera_live_feed.py` detector running live against the camera topic.

---

## Quick start

```bash
# Headless — terminal progress only
bash simulation/run_course.sh --course 1   # straight channel  ~60 s
bash simulation/run_course.sh --course 2   # lawnmower survey  ~3 min
bash simulation/run_course.sh --course 3   # L-shaped dogleg   ~90 s

# Visual — 4 windows: Gazebo 3D, SITL console, camera detector, GPS coords
bash simulation/run_course.sh --course 1 --visual
```

All outputs auto-save to `simulation/sim_tests/run_N/` on completion.

---

## The three courses

### Course 1 — Straight Navigation Channel
**World:** `simulation/gazebo/worlds/robotx_uav_course.sdf`
**Inspired by:** RobotX 2026 "Safe Passage" (Task 1)

Three red/green gate pairs along a straight East axis, plus a scan-the-code light buoy at the end. Baseline test for the detector.

| Buoy | East (m) | North (m) | Color |
|------|----------|-----------|-------|
| gate1_green | 10 | +1.25 | green |
| gate1_red | 10 | −1.25 | red |
| gate2_green | 25 | +1.25 | green |
| gate2_red | 25 | −1.25 | red |
| gate3_green | 40 | +1.25 | green |
| gate3_red | 40 | −1.25 | red |
| light_buoy | 50 | 0 | — |

Flight: straight East at N=0, 4 s hover per gate.

### Course 2 — Open Water Survey (Lawnmower)
**World:** `simulation/gazebo/worlds/course_2_search_field.sdf`

Seven buoys scattered across a 60×30 m open field. Drone runs three East-West strips (lawnmower). Tests detection without predictable layout.

### Course 3 — L-Shaped Dogleg
**World:** `simulation/gazebo/worlds/course_3_dogleg.sdf`
**Inspired by:** RobotX 2026 "Gymkhana"

Two legs: East (gates 1–2) then a 90° right turn North (gates 3–4). Tests detector on two different approach headings.

---

## Run output folder

Every run saves to `simulation/sim_tests/run_N/` (N auto-increments):

| File | Contents |
|------|----------|
| `detections.csv` | Per-frame buoy GPS projections |
| `accuracy_report.md` | Cross-referenced vs ground-truth positions |
| `summary.json` | Machine-readable metrics: duration, mean error, buoys found |
| `map.png` | Top-down detection diagram: detected vs GT, error lines |
| `gz.log` / `sitl.log` / `fly.log` | Process logs |

---

## Distractor obstacles

All three courses include ~11 floating debris objects designed to stress-test the HSV detector:

| Type | Shape | Hue challenge |
|------|-------|---------------|
| Olive-green panels | Flat box ~0.8×0.5×0.1 m | Hue ~60–70, just below green range |
| Orange-brown crates | Box ~0.5×0.5×0.4 m | Hue ~15–25, bleeds into red at low sat |
| Gray barrels | Cylinder r=0.15–0.2 m | Neutral — tests shape filtering |
| Gray flat panels | Flat box ~0.9×0.6×0.1 m | Neutral — tests size gating |

Distractors have **no emissive material** (unlike real buoys), so they appear duller from nadir and should be suppressed by confidence thresholding + size gating.

---

## Manual / advanced operation

### Start Gazebo + SITL without auto-flight

```bash
bash simulation/run_robotx_uav_sitl.sh             # Course 1, GUI
bash simulation/run_robotx_uav_sitl.sh --headless  # headless
bash simulation/run_robotx_uav_sitl.sh --no-sitl   # Gazebo only
```

SITL MAVLink: `tcp:127.0.0.1:5760`, FDM UDP: 9002

### Fly a course manually

```bash
python3 simulation/fly_course.py --course 1 --connect udp:127.0.0.1:14550
python3 simulation/fly_course.py --course 2 --connect udp:127.0.0.1:14550 --speed 2.0
python3 simulation/fly_course.py --course 3 --connect udp:127.0.0.1:14550
```

### Verify accuracy during a flight

```bash
python3 simulation/accuracy_verify.py --connect udp:127.0.0.1:14551
```

Subscribes to `/drone/camera`, runs the full detection pipeline, writes a cross-referenced accuracy report on exit.

### Regenerate the detection map

```bash
python3 simulation/plot_run.py             # latest run
python3 simulation/plot_run.py --run 3    # specific run
```

---

## Technical notes

- ogre2 ignores the camera `<distortion>` block → the render produces a clean pinhole image. Always run with `--no-undistort`. All launchers already do this.
- `gz_env.sh` sets all `GZ_SIM_*` paths. Source it (don't execute it) before running anything manually.
- "ArduPilot controller has reset" a few times at startup is normal. A continuous reset loop is not.
- `eeprom.bin` in the repo root persists `FRAME_CLASS=1` so arming works without wiping params each launch.
- Green buoys use a spring-green emissive (OpenCV hue ~81) to stay inside the HSV green range (75–105).

---

## Legacy

`ucsd_robotx_demo`, `run_end_to_end.sh`, `verify_sim_topics.sh` are the old flat-ocean single-course setup. Kept for reference but superseded by the three-course stack above.
