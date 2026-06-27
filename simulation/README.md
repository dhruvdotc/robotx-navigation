# Gazebo Harmonic RobotX UAV Course

A Gazebo Harmonic scene where an ArduPilot-SITL drone flies a nadir camera over a RobotX-spec navigation channel (3 red/green gates) on an animated ocean. The real `camera_live_feed.py` detector runs against the live camera topic and projects each buoy detection to GPS.

---

## World: `gazebo/worlds/robotx_uav_course.sdf`

**Animated ocean.** The water uses VRX's `coast_waves` Gerstner-wave ocean driven by `libWaveVisual.so`. A `vrx::PublisherPlugin` publishes wavefield parameters on `/vrx/wavefield/parameters`. It is visual only - no physics or collision. A separate invisible `ocean_surface` collision plane at z=0 lets the drone rest and arm before takeoff.

**3 navigation gates.** Red/green Sur-Mark cylinders along +X at Y=0, at East = 10, 25, 40 m. Gate width is 2.5 m (within RobotX spec of 1.8-3.0 m). A scan-the-code `light_buoy` sits at East=50. Green uses a spring-green emissive (~OpenCV hue 81) so the HSV detector's green range (75-99) catches it cleanly without colliding with blue.

**`iris_uav` model.** Stock iris airframe with the ArduPilotPlugin (FDM UDP 9002, lockstep) and a fixed nadir `gimbal_nadir` camera pod at 1920x1080, with calibration-matched intrinsics, publishing on `/drone/camera`.

**GPS datum.** Set to ArduPilot SITL's CMAC home so the SITL home and world origin match.

---

## Prerequisites (Ubuntu 22.04 / WSL Ubuntu-22.04)

- ROS 2 Humble (`/opt/ros/humble`), Gazebo Harmonic (`gz`), `ros_gz_image`
- ArduPilot SITL built at `~/ardupilot`, plus `ardupilot_gazebo` plugin at `~/ardupilot_gazebo`
- VRX built in `~/vrx_ws` (supplies `coast_waves` and the wave plugins). Override with `VRX_GZ=<path>` if installed elsewhere.

`gz_env.sh` is the single source of truth for `GZ_SIM_*` resource and plugin paths (repo models first, then ardupilot_gazebo, then VRX). It is sourced by both launchers - do not set these paths manually.

---

## Run

**Basic launch (one terminal):**

```bash
bash simulation/run_robotx_uav_sitl.sh            # Gazebo GUI + SITL
bash simulation/run_robotx_uav_sitl.sh --headless # headless server + SITL
bash simulation/run_robotx_uav_sitl.sh --no-sitl  # Gazebo only, no SITL
```

SITL MAVLink is at `tcp:127.0.0.1:5760` (FDM UDP 9002). `eeprom.bin` in the repo root persists `FRAME_CLASS=1` so arming works without wiping parameters each launch.

**Full demo - three windows on the WSLg display:**

```bash
bash simulation/run_demo_windows.sh
```

Opens three windows: the Gazebo GUI with the animated ocean, an xterm running the `sim_vehicle.py` / MAVProxy console, and an xterm running `camera_live_feed.py --ros-topic /drone/camera` showing live detection output. A background gz-to-ROS image bridge starts automatically. MAVProxy outputs are exposed at `udp:14550` (fly_course), `udp:14551` (accuracy_verify), `udp:14552` (readiness probe).

When it prints **READY**, start recording and then fly:

```bash
python3 simulation/fly_course.py
```

This runs a 10 second countdown then a scripted GUIDED flight over the 3-gate course.

---

## Verify detection and GPS accuracy

Run alongside a flight to log detections live and write a report on exit:

```bash
python3 simulation/accuracy_verify.py --connect udp:127.0.0.1:14551
```

It subscribes to `/drone/camera`, runs the same pipeline as `camera_live_feed.py`, and reads live pose and attitude over MAVLink. For every level frame it projects detections to local-NED and GPS using the drone's live altitude, logging to `simulation/accuracy_logs/detections_<ts>.csv` in real time.

On flight end (Ctrl-C, `--duration` timeout, or disarm-after-arm) it cross-references the log against the world's ground-truth buoy positions and writes `simulation/accuracy_report_<ts>.md` plus refreshes `simulation/accuracy_report.md` with per-buoy error (m), mean/max error, detection count, and mean confidence.

A 15 second minimum sustained-flight duration is enforced - shorter runs are flagged in the report.

Typical result over the 3-gate course at 10 m: 6/6 colour buoys detected, mean error ~0.15 m. The `light_buoy` is an expected miss (black box, no colour signature from nadir).

---

## Notes

- ogre2 ignores the camera `<distortion>` block ("ImageBrownDistortionModel is not supported in ogre2") so the render is a clean pinhole. Run the detector with `--no-undistort` - both the demo launcher and accuracy_verify already do this.
- Do not reboot the flight controller in place - it breaks gz lockstep. Restart both the Gazebo and SITL processes together.
- Seeing "ArduPilot controller has reset" a couple of times at startup is normal. A continuous reset loop is not.

---

## Legacy

The earlier flat-ocean scene (`ucsd_robotx_demo`, `run_end_to_end.sh`, `verify_sim_topics.sh`, `ros2_bridge/bridge.yaml`) is superseded by this UAV course but kept in the repo for reference.
