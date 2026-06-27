# Gazebo Harmonic RobotX UAV Course (camera + buoys + SITL flight)

A Gazebo Harmonic scene where an ArduPilot-SITL drone flies a nadir camera over a
RobotX-spec navigation channel (3 red/green gates) on an **animated ocean**, and
the real `camera_live_feed.py` detector projects each buoy to GPS from the live
camera topic.

## The world: `gazebo/worlds/robotx_uav_course.sdf`

- **Animated ocean (VRX).** The water is VRX's `coast_waves` Gerstner-wave ocean
  driven by `libWaveVisual.so`; a `vrx::PublisherPlugin` publishes the wavefield
  parameters on `/vrx/wavefield/parameters`. It reads as moving open water with a
  depth gradient and ripple texture — **visual only**, no physics/collision. A
  separate invisible `ocean_surface` collision plane at z=0 lets the drone rest/arm.
- **3 gates** (red/green Sur-Mark cylinders) along +X at Y=0, gates at East = 10,
  25, 40 m; gate width 2.5 m. Plus a scan-the-code `light_buoy` at East=50.
  Green is a spring-green emissive (~OpenCV hue 81) so the HSV detector's green
  range (75–99) catches it without colliding with blue.
- **`iris_uav`** (local model): stock iris airframe + ArduPilotPlugin (FDM UDP
  9002, lockstep) with a fixed nadir `gimbal_nadir` camera pod, 1920×1080 with
  calibration-matched intrinsics, publishing `/drone/camera`.
- GPS datum = ArduPilot SITL's CMAC home, so SITL home matches the world origin.

## Prerequisites (Ubuntu 22.04 / WSL Ubuntu-22.04)

- ROS 2 Humble (`/opt/ros/humble`), Gazebo Harmonic (`gz`), `ros_gz_image`.
- ArduPilot SITL built (`~/ardupilot`) + `ardupilot_gazebo` plugin (`~/ardupilot_gazebo`).
- **VRX** built in `~/vrx_ws` (supplies `coast_waves` + the wave plugins). Override
  the location with `VRX_GZ=...` if installed elsewhere.

`simulation/gz_env.sh` is the single source of truth for the `GZ_SIM_*` resource /
plugin paths (repo models first, then ardupilot_gazebo, then VRX) and is sourced by
both launchers — don't hand-roll the env.

## Run it

**Headless smoke test / scripted flight (one terminal):**

```bash
bash simulation/run_robotx_uav_sitl.sh            # gz GUI + SITL (FDM lockstep)
bash simulation/run_robotx_uav_sitl.sh --headless # gz server-only + SITL
bash simulation/run_robotx_uav_sitl.sh --no-sitl  # gz only
```
SITL MAVLink is at `tcp:127.0.0.1:5760` (FDM UDP 9002). `eeprom.bin` in the repo
root persists `FRAME_CLASS=1` so arming works on non-wipe launches.

**Live demo — three real windows on the WSLg display (for screen recording):**

```bash
bash simulation/run_demo_windows.sh
```
Opens (1) the Gazebo GUI with the animated ocean, (2) an xterm running the genuine
`sim_vehicle.py` / MAVProxy console (real arm/mode/GPS text), and (3) an xterm
running `camera_live_feed.py --ros-topic /drone/camera` with its real `[INFO]`/`[GPS]`
output. It starts a background gz→ROS image bridge and exposes MAVProxy outs:
`udp:14550` (fly_course), `udp:14551` (accuracy_verify), `udp:14552` (readiness probe).
When it prints **READY**, start recording, then fly:

```bash
python3 simulation/fly_course.py          # 10 s countdown, then a cinematic GUIDED flight
```

## Verify detection + GPS accuracy from a flight: `accuracy_verify.py`

Run it alongside a flight (it logs every detection live and writes a timestamped report):

```bash
python3 simulation/accuracy_verify.py --connect udp:127.0.0.1:14551
```
- Subscribes to `/drone/camera`, runs the exact `camera_live_feed.py` pipeline, and
  reads live pose/attitude over MAVLink. For every **level** frame it projects each
  detection to absolute local-NED + GPS using the drone's **live altitude** and logs
  it in real time to `simulation/accuracy_logs/detections_<ts>.csv`.
- On flight end (Ctrl-C / `--duration` / disarm-after-arm) it cross-references the
  log against the world's ground-truth buoy positions and writes
  `simulation/accuracy_report_<ts>.md` (+ refreshes `simulation/accuracy_report.md`):
  per-buoy error (m), mean/max error, detection count + mean confidence.
- A **15 s minimum** sustained-flight duration is enforced (shorter runs are flagged
  in the report, not silently accepted).

Typical result over the 3-gate course at 10 m: 6/6 colour buoys, mean error ~0.15 m,
`light_buoy` an expected miss (black box, no colour from nadir).

## Notes / gotchas

- **ogre2 ignores the camera `<distortion>` block** ("ImageBrownDistortionModel is
  not supported in ogre2") → the render is a clean pinhole, so run the detector with
  `--no-undistort` (both demo launcher and accuracy_verify already do).
- Don't reboot the FC in place — it breaks gz lockstep. Restart both processes.
- "ArduPilot controller has reset" a couple of times at startup is normal; a
  continuous loop is not.

## Legacy

The earlier flat-ocean scene (`ucsd_robotx_demo`, `run_end_to_end.sh`,
`verify_sim_topics.sh`, `ros2_bridge/bridge.yaml`) and its `gz-sim-waves-system`
note are superseded by the UAV course above but kept for reference.
