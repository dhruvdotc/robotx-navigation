# Simulink GPS/IMU Noise Injection

**Goal:** keep Gazebo as the ground-truth physics engine; use Simulink as a
noise-injection layer that sits between Gazebo's perfect sensor output and
whatever downstream consumer reads GPS or IMU.

Gazebo publishes perfect odometry and IMU data on ROS 2. Simulink (running in
MATLAB on the Mac) subscribes to those topics, applies realistic sensor noise
(fix-type state machine, multipath, drift), and republishes a noisy
`sensor_msgs/NavSatFix` on `/fix`. The buoy-detection pipeline can then
optionally subscribe to `/fix` instead of using the MAVLink GPS fix it currently
reads from ArduPilot.

---

## Architecture

```
Gazebo (ground truth)
  |-- /model/iris_uav/odometry  (nav_msgs/Odometry, 30 Hz, perfect pose)
  |-- /imu/data                 (sensor_msgs/Imu, 30 Hz, perfect IMU)
        |
        v   [ros_gz_bridge parameter_bridge  -- run_course.sh launches this]
        |
ROS 2 topics on the Ubuntu host
        |
        v   [MATLAB Robotics System Toolbox ROS 2 node]
        |
Simulink  gps_navsatfix_sim.slx
  - Subscribes:  /model/iris_uav/odometry  (position truth)
  - Subscribes:  /imu/data                 (orientation truth, optional)
  - State machine: fix type cycles ~75% RTK-float (sigma ~0.02 m),
                   ~20% DGPS (sigma ~0.4 m), ~5% single-point (sigma ~1.5 m)
  - Publishes:   /fix  (sensor_msgs/NavSatFix, noisy)
        |
        v
ROS 2 /fix topic -- any downstream node (accuracy_verify, camera_live_feed)
can subscribe here instead of the ArduPilot MAVLink GPS fix
```

---

## What was changed

### `simulation/gazebo/models/iris_uav/model.sdf`

Added `gz-sim-odometry-publisher-system` plugin (before the closing `</model>`
tag). This makes Gazebo publish the drone's ground-truth pose as a Gazebo
`Odometry` message on `/model/iris_uav/odometry` at 30 Hz. Without this plugin
the topic does not exist and the bridge has nothing to bridge.

### `simulation/ros2_bridge/bridge.yaml`

Added entries for:
- `/model/iris_uav/odometry` (`gz.msgs.Odometry` -> `nav_msgs/msg/Odometry`)
- `/imu/data` (`gz.msgs.IMU` -> `sensor_msgs/msg/Imu`)

The IMU gz-topic path includes the world name, which differs across the three
courses. The YAML entry shows Course 1 for reference; `run_course.sh` passes the
correct interpolated topic name at launch time.

### `simulation/run_course.sh`

Added a second background bridge process (`ros2 run ros_gz_bridge
parameter_bridge`) launched immediately after the image bridge. It bridges
odometry and IMU using the world-name-interpolated IMU topic. `SENSOR_BRIDGE_PID`
is tracked and torn down in `cleanup()` / `kill_all_sim()`.

---

## MATLAB compatibility constraint - IMPORTANT

MATLAB R2025a and R2026a ship with ROS 2 **Jazzy** support only. Our Gazebo
SITL stack runs ROS 2 **Humble**.

| MATLAB release | Supported ROS 2 DDS |
|----------------|---------------------|
| R2022b / R2023a | Humble |
| R2023b / R2024a | Iron |
| R2025a / R2026a | Jazzy |

**Bottom line:** you need an older MATLAB (R2022b or R2023a) to connect to the
Humble topics directly. Alternatively, run a ROS 2 Jazzy container or bridge on
the Mac and republish from there. Verify with Abhishek (abshanka@mathworks.com)
what MATLAB version the `.slx` model was built and tested with before proceeding.

---

## Verifying the new topics (run a simulation first)

```bash
# Start Course 1 in headless mode
bash simulation/run_course.sh --course 1

# In another terminal, source ROS and check topics
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E "odometry|imu|fix"
# Expected:
#   /model/iris_uav/odometry
#   /imu/data

# Spot-check odometry at 30 Hz
ros2 topic hz /model/iris_uav/odometry
ros2 topic echo /model/iris_uav/odometry --once

# Spot-check IMU
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

If `/model/iris_uav/odometry` is missing, the OdometryPublisher plugin did not
load - check `simulation/sim_tests/run_N/gz.log` for `[OdometryPublisher]`
lines. If `/imu/data` is missing but odometry is present, the IMU topic path is
wrong; run `gz topic -l | grep imu` while the sim is running to get the actual
name and update `run_course.sh`'s `GZ_IMU_TOPIC` variable.

---

## Opening and running gps_navsatfix_sim.slx

`gps_navsatfix_sim.slx` lives at the repo root. It was provided by Abhishek
Shankar (abshanka@mathworks.com, MathWorks).

```
1. Open MATLAB (R2022b or R2023a - see compatibility note above)
2. In MATLAB: open('path/to/robotx-navigation/gps_navsatfix_sim.slx')
3. Check the ROS 2 Subscribe blocks:
     - Confirm topic name matches /model/iris_uav/odometry
     - Confirm topic name matches /imu/data  (if used)
   If they differ, double-click the Subscribe block and update the topic name.
4. Start the Gazebo sim (bash simulation/run_course.sh --course 1)
5. Wait for /model/iris_uav/odometry to appear (check with ros2 topic list)
6. In MATLAB: run the Simulink model (Simulation -> Run or Ctrl+T)
7. Check /fix is being published:
     ros2 topic echo /fix --once
```

The model should publish `sensor_msgs/NavSatFix` on `/fix`. The fix type
(STATUS_FIX, STATUS_GBAS_FIX, etc.) in the message will cycle according to
the state machine inside the `.slx`.

---

## Next steps to integrate /fix into the pipeline

Currently `camera_live_feed.py` and `accuracy_verify.py` read the drone's GPS
position from the MAVLink connection (`--connect udp:127.0.0.1:145xx`), not from
a ROS 2 NavSatFix topic. To use the Simulink-noisy GPS instead:

1. **Add a ROS 2 subscriber** to `camera_live_feed.py` for `/fix`
   (`sensor_msgs/msg/NavSatFix`) and use that lat/lon as the origin datum when
   set, falling back to `--origin-lat/lon` when absent.
2. **Or** write a small bridge node that reads `/fix` and rebroadcasts it as a
   MAVLink `GPS_RAW_INT` message so ArduPilot (and thus `camera_live_feed.py`'s
   existing `--connect` path) sees the noisy GPS. This keeps the pipeline
   unchanged and lets Simulink noise propagate through ArduCopter's EKF as well,
   which is more realistic.
3. **Run accuracy_verify.py** in a mode that compares Gazebo ground truth vs.
   Simulink-noisy GPS to quantify how much positioning error the noise model
   introduces.

Option 2 is recommended for a realistic end-to-end test because ArduCopter's
EKF will filter and smooth the noisy GPS, matching what happens in real flight.

---

## File locations

| File | Purpose |
|------|---------|
| `gps_navsatfix_sim.slx` | Simulink GPS noise model (repo root) |
| `simulation/gazebo/models/iris_uav/model.sdf` | OdometryPublisher plugin added |
| `simulation/ros2_bridge/bridge.yaml` | Bridge topic reference (odometry + IMU) |
| `simulation/run_course.sh` | Launches sensor bridge alongside image bridge |
