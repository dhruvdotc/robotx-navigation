# Simulink Bridge — Test Status

Full architecture doc: [docs/11_simulink_sensor_sim.md](../11_simulink_sensor_sim.md)

---

## What has been validated (offline / static)

| Item | Method | Result |
|---|---|---|
| `iris_uav/model.sdf` SDF XML parses clean | `gz sdf --check` (syntax only, no runtime) | ✅ Pass |
| `OdometryPublisher` plugin present in SDF | grep / XML parse | ✅ Present |
| `simulation/ros2_bridge/bridge.yaml` correct YAML + topic names | syntax check | ✅ Pass |
| `run_course.sh` shell syntax | `bash -n` | ✅ Pass |
| Sensor bridge process (`SENSOR_BRIDGE_PID`) tracked and torn down in `cleanup()` | code review | ✅ Present |
| `bridge.yaml` odometry entry: `gz.msgs.Odometry → nav_msgs/msg/Odometry` | code review | ✅ Correct |
| `bridge.yaml` IMU note: gz topic path varies per course world name, `run_course.sh` interpolates at launch | code review | ✅ Correct |

---

## What still needs a live run on Ubuntu

These cannot be checked without the full Gazebo Harmonic + ROS 2 Humble + ArduPilot SITL stack running.

### Layer 1 — Gazebo → ROS 2 bridge

| Test | How to check | Expected |
|---|---|---|
| `/model/iris_uav/odometry` appears | `ros2 topic list \| grep odometry` | Topic present |
| `/model/iris_uav/odometry` publishing at ~30 Hz | `ros2 topic hz /model/iris_uav/odometry` | ~30 Hz |
| `/imu/data` appears | `ros2 topic list \| grep imu` | Topic present |
| `/imu/data` publishing at ~30 Hz | `ros2 topic hz /imu/data` | ~30 Hz |
| Odometry position looks sane (moves as drone flies) | `ros2 topic echo /model/iris_uav/odometry --once` | Non-zero pose, changes over time |

If `/model/iris_uav/odometry` is missing → OdometryPublisher plugin failed to load, check `gz.log` for `[OdometryPublisher]`.

If `/imu/data` is missing but odometry is present → IMU gz topic path is wrong; run `gz topic -l | grep imu` while sim is running to get the actual name and compare to `GZ_IMU_TOPIC` in `run_course.sh`.

```bash
# Run this on Ubuntu after starting any course
bash simulation/run_course.sh --course 1

# In a second terminal
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E "odometry|imu|fix"
ros2 topic hz /model/iris_uav/odometry
ros2 topic hz /imu/data
```

---

### Layer 2 — Simulink (`gps_navsatfix_sim.slx`) connects and publishes

> **Prerequisite:** MATLAB R2022b or R2023a (Humble-compatible). R2025a/R2026a ships with Jazzy support only and will not see the Humble topics. See `11_simulink_sensor_sim.md` compatibility table. Confirm version with Abhishek (abshanka@mathworks.com) before proceeding.

| Test | How to check | Expected |
|---|---|---|
| Simulink opens without errors | `open('gps_navsatfix_sim.slx')` in MATLAB | No missing block errors |
| Subscribe blocks point to correct topic names | Inspect each ROS2Subscribe block | `/model/iris_uav/odometry`, `/imu/data` |
| Simulink runs without timeout errors | `Simulation → Run` while sim is live | No "ROS 2 node failed" errors |
| `/fix` topic appears in ROS | `ros2 topic list \| grep fix` | `/fix` present |
| `/fix` has correct type | `ros2 topic info /fix` | `sensor_msgs/msg/NavSatFix` |
| `/fix` lat/lon tracks drone position | `ros2 topic echo /fix --once` while drone is in flight | Non-zero lat/lon, changes as drone moves |
| Fix type cycles through states | Observe `status.status` field over ~60 s | Mix of 0 (SBAS/single), 1 (SBAS), 2 (GBAS/RTK) values |

---

### Layer 2b — Mock Simulink publisher (Docker-only test, no MATLAB needed)

`simulation/mock_fix_publisher.py` mirrors `gps_navsatfix_sim.slx` exactly:
subscribes to `/model/iris_uav/odometry`, applies the same fix-type state machine
(RTK-float 75%, DGPS 20%, single-point 5%), and publishes noisy `NavSatFix` on `/fix`.

```bash
# Inside the sim container, after run_course.sh --course 1
source /opt/ros/humble/setup.bash
python3 simulation/mock_fix_publisher.py --datum-lat -35.363262 --datum-lon 149.165237
# Then verify:
ros2 topic echo /fix --once
```

Use this to validate Layer 3 without needing MATLAB reachability from Docker.

### Layer 2c — Real MATLAB → Docker (when native Ubuntu isn't available)

DDS multicast doesn't cross the Colima VM boundary. Use the unicast profile in
`docker/fastdds_colima.xml`:

```bash
# 1. Get the Colima VM IP
colima ssh -- ip addr show eth0 | grep "inet "

# 2. Edit docker/fastdds_colima.xml — replace COLIMA_VM_IP with that address

# 3. In MATLAB, before ros2node():
setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '/path/to/docker/fastdds_colima.xml')
setenv('ROS_DOMAIN_ID', '0')
```

Then open `gps_navsatfix_sim.slx` and run — MATLAB will connect to the container's topics.

### Layer 3 — Noisy GPS feeds into buoy pipeline

| Test | How to check | Expected |
|---|---|---|
| `accuracy_verify.py` or `camera_live_feed.py` can subscribe to `/fix` | Requires code addition (see below) | Not yet implemented — `/fix` subscriber not wired in |
| GPS error from Simulink noise model is measurable | Compare `accuracy_report.md` with and without mock noise | Error should increase from baseline 0.16 m mean |
| Noisy GPS via ArduCopter EKF (Option 2 — recommended) | Bridge `/fix` → `GPS_RAW_INT` MAVLink, watch EKF output | EKF absorbs noise; pipeline unchanged |

Layer 3 is entirely **not yet implemented**. The current pipeline reads GPS from MAVLink (`--connect`) and ignores `/fix` entirely. See `11_simulink_sensor_sim.md → Next steps to integrate /fix into the pipeline` for the two implementation options.

---

## Summary

```
Layer 1   Gazebo → ROS 2 bridge        static ✅   live run ❌ (needs Docker build)
Layer 2   Simulink .slx → /fix         static n/a  live run ❌ (needs MATLAB + Docker networking)
Layer 2b  mock_fix_publisher.py        ready ✅    run inside Docker container
Layer 3   /fix → buoy pipeline                      not yet implemented ❌
```

The only thing blocking Layer 1 from being confirmed is SSH access to the Ubuntu machine. Layer 2 additionally needs the correct MATLAB version confirmed. Layer 3 is a code task.
