# Simulink Bridge — Test Status

Full architecture doc: [docs/11_simulink_sensor_sim.md](../11_simulink_sensor_sim.md)

---

## Docker E2E Test Results ✅ — 2026-08-27

All Layers 1 and 2b have been verified end-to-end inside the `robotx-sim` Docker container
(Gazebo Fortress + ROS 2 Humble + `ros_gz_bridge`, ARM64/Colima on Apple Silicon Mac).

**Test command:**
```bash
docker exec robotx-sim bash /ws/robotx-navigation/docker/run_e2e_test.sh
```

**Key result (run `e2e_docker_20260827_045721`):**

| Layer | Check | Result |
|---|---|---|
| 1 | `/model/iris_uav/odometry` topic present | ✅ |
| 1 | IMU topic present (full Gazebo path) | ✅ |
| 1 | Odometry publishing rate | ✅ **49.8 Hz** |
| 2b | `/fix` topic present | ✅ |
| 2b | `/fix` publishing rate | ✅ **4.995 Hz** |
| 2b | `/fix` lat/lon with noise | ✅ `lat: -35.363258, lon: 149.165243` |
| 2b | Fix status field | ✅ `status: 1` (DGPS) |
| 3 | `accuracy_verify.py --use-fix` code wiring | ✅ |

**Notes on Gazebo version:**
`ros-humble-ros-gz-bridge` (v0.244.25) on Ubuntu 22.04 is compiled against
`libignition-msgs8` + `libignition-transport11` (Gazebo **Fortress**).
Using `gz-harmonic` (transport13) caused silent "Unknown message type" errors.
The Docker image and `run_e2e_test.sh` now use `ign sim` (Fortress) with the
`ignition.msgs.*` bridge type strings.

**`IGN_IP=127.0.0.1` required** — forces Gz transport to use loopback unicast
instead of multicast (which Docker blocks by default).

---

## How to run the Docker E2E test

Requires: [Colima](https://github.com/abiosoft/colima) + Docker on macOS (Apple Silicon or x86).

```bash
# 1. Start Colima (if not already running)
colima start --cpu 4 --memory 8

# 2. Build the image (first time only — ~10 min)
colima ssh -- docker build \
  -f /Users/xurui/Downloads/ROBOTX/robotx-navigation/docker/sim.Dockerfile \
  -t robotx-sim \
  /Users/xurui/Downloads/ROBOTX/robotx-navigation

# 3. Start a persistent container with the repo volume-mounted
#    (skip if already running: `docker ps | grep robotx-sim`)
colima ssh -- docker run -d --name robotx-sim \
  -v /Users/xurui/Downloads/ROBOTX/robotx-navigation:/ws/robotx-navigation \
  robotx-sim bash -c 'sleep infinity'

# 4. Run the e2e test (takes ~40 s)
colima ssh -- docker exec robotx-sim \
  bash /ws/robotx-navigation/docker/run_e2e_test.sh
```

Expected output (passing):
```
✅ /model/iris_uav/odometry present
✅ IMU topic present
  Odometry rate: ~100 Hz  (no autopilot = unlocked physics; ~30 Hz on full stack)
✅ /fix topic present
  /fix rate: 5.000 Hz
  latitude: -35.363258   longitude: 149.165243
```

Logs saved to `simulation/sim_tests/e2e_docker_<timestamp>/`.

---

## What has been validated (offline / static)

| Item | Method | Result |
|---|---|---|
| `iris_uav/model.sdf` SDF XML parses clean | `gz sdf --check` (syntax only) | ✅ Pass |
| `OdometryPublisher` plugin present in SDF | grep / XML parse | ✅ Present |
| `simulation/ros2_bridge/bridge.yaml` correct YAML + topic names | syntax check | ✅ Pass |
| `run_course.sh` shell syntax | `bash -n` | ✅ Pass |
| Sensor bridge process tracked and torn down in `cleanup()` | code review | ✅ Present |
| `bridge.yaml` odometry entry: `gz.msgs.Odometry → nav_msgs/msg/Odometry` | code review | ✅ Correct |
| Bridge protocol: use `ignition.msgs.*` with `ros-humble-ros-gz-bridge` | live test | ✅ Confirmed |

---

## Live-run results (Docker, 2026-08-27)

### Layer 1 — Gazebo Fortress → ROS 2 bridge

| Test | Result |
|---|---|
| `/model/iris_uav/odometry` appears | ✅ |
| `/model/iris_uav/odometry` publishing ~30+ Hz | ✅ 49.8 Hz |
| IMU topic appears (full Gz path under `/world/robotx_uav_course/...`) | ✅ |
| Odometry pose is non-zero (drone floating at z=5m) | ✅ |

> **Note on odometry rate:** The minimal Docker test world has no autopilot (no ArduPilot SITL),
> so physics runs unlocked at ~50 Hz. In the full simulation with ArduPilot lock-step, this
> stabilises at 30 Hz as configured.

### Layer 2b — Mock Simulink publisher

| Test | Result |
|---|---|
| `/fix` topic appears | ✅ |
| `/fix` publishing at ~5 Hz | ✅ 4.995 Hz |
| `/fix` lat/lon tracks drone datum position | ✅ |
| Noise model active (RTK-float / DGPS / single-point mix) | ✅ |
| `position_covariance` correctly typed (`float64[9]`) | ✅ (fixed) |

---

## What still needs a live run on Ubuntu (with ArduPilot SITL)

### Layer 1 — Full course world

The Docker test uses a minimal world (`robotx_docker_test.sdf`).
The production world (`robotx_uav_course.sdf`) with `ardupilot_gazebo` and
`iris_with_standoffs` still requires Ubuntu + `ardupilot_gazebo` compiled from source.

```bash
# Run this on Ubuntu after setting up the full stack
bash simulation/run_course.sh --course 1

# In a second terminal
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E "odometry|imu|fix"
ros2 topic hz /model/iris_uav/odometry   # expect ~30 Hz
```

### Layer 2 — Real MATLAB Simulink (`gps_navsatfix_sim.slx`)

| Test | Status |
|---|---|
| Simulink opens without errors | ❌ needs MATLAB + Ubuntu |
| `/fix` publishes from Simulink | ❌ needs MATLAB + Ubuntu |
| Fix type cycles through RTK/DGPS/single | ❌ needs MATLAB |

**MATLAB version note:** R2022b or R2023a required for ROS 2 Humble compatibility.
R2025a/R2026a ships with Jazzy support only.

**Docker → MATLAB networking:** DDS multicast doesn't cross the Colima VM boundary.
Use the unicast profile in `docker/fastdds_colima.xml`:

```bash
# 1. Get Colima VM IP
colima ssh -- ip addr show eth0 | grep "inet "

# 2. Edit docker/fastdds_colima.xml — replace COLIMA_VM_IP

# 3. In MATLAB before ros2node():
setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '/path/to/docker/fastdds_colima.xml')
setenv('ROS_DOMAIN_ID', '0')
```

### Layer 3 — Noisy GPS → buoy pipeline (needs ArduPilot SITL)

The `--use-fix` flag is wired into `accuracy_verify.py` (Layer 3 code ✅).
A full live comparison (noisy fix GPS vs MAVLink GPS error) needs:
1. ArduPilot SITL running and connected
2. Camera feed (real or simulated)
3. Both `--use-fix` and baseline runs compared in `accuracy_report.md`

---

## Summary

```
Layer 1   Gazebo Fortress → ROS 2 bridge     ✅ Docker confirmed (49.8 Hz odometry)
Layer 2   Simulink .slx → /fix               ❌ needs MATLAB + Ubuntu
Layer 2b  mock_fix_publisher.py → /fix        ✅ Docker confirmed (4.995 Hz, GPS with noise)
Layer 3   /fix → accuracy_verify.py          ✅ code wired  ❌ live test needs SITL
```
