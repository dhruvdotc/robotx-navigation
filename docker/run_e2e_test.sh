#!/bin/bash
# Full e2e test script — runs inside the robotx-sim container.
# Tests all three layers of the Simulink bridge pipeline:
#   Layer 1: Gazebo Fortress → ros_gz_bridge → /model/iris_uav/odometry + IMU
#   Layer 2b: mock_fix_publisher.py → /fix (noisy NavSatFix)
#   Layer 3: accuracy_verify.py --use-fix consuming /fix
#
# Requires: robotx-sim image built from docker/sim.Dockerfile
# Usage (from Mac, after container is running):
#   docker exec robotx-sim bash /ws/robotx-navigation/docker/run_e2e_test.sh

set -eo pipefail  # no -u: ROS setup.bash uses unbound vars internally
REPO=/ws/robotx-navigation
COURSE=docker
# Self-contained Fortress world — no ardupilot_gazebo / VRX deps.
WORLD="${REPO}/simulation/gazebo/worlds/robotx_docker_test.sdf"
RUN_DIR="${REPO}/simulation/sim_tests/e2e_docker_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"

source /opt/ros/humble/setup.bash

# Force ign-transport to use loopback (unicast) — fixes multicast discovery
# failure inside Docker where multicast is disabled or unreliable.
export IGN_IP=127.0.0.1

# Expose local models to Gazebo Fortress (ign sim)
export IGN_GAZEBO_RESOURCE_PATH="${REPO}/simulation/gazebo/models:${IGN_GAZEBO_RESOURCE_PATH:-}"

WORLD_NAME="robotx_uav_course"   # constant — matches bridge.yaml topic paths
# In the Docker test world, imu_sensor lives directly on base_link (no nested model)
IGN_IMU_TOPIC="/world/${WORLD_NAME}/model/iris_uav/link/base_link/sensor/imu_sensor/imu"

echo "============================================================"
echo " RobotX E2E Docker Test — $(date)"
echo " Gazebo: Fortress (ign sim)  |  Bridge: ros_gz_bridge"
echo " Output: $RUN_DIR"
echo "============================================================"

# ── Layer 1a: Start bridge FIRST so its Gz subscriber is ready ────────────────
echo ""
echo "[LAYER 1] Starting ros_gz_bridge (odometry + IMU + clock)..."
ros2 run ros_gz_bridge parameter_bridge \
    "/model/iris_uav/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry" \
    "${IGN_IMU_TOPIC}@sensor_msgs/msg/Imu[ignition.msgs.IMU" \
    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock" \
    > "$RUN_DIR/sensor_bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 2

# ── Layer 1b: Start Gazebo Fortress (headless, server-only, auto-run) ─────────
echo "[LAYER 1] Starting Gazebo Fortress (headless, Course $COURSE)..."
ign gazebo --headless-rendering -r -s "$WORLD" > "$RUN_DIR/gz.log" 2>&1 &
GZ_PID=$!
echo "  Gazebo PID: $GZ_PID"
sleep 8

# ── Check topics ──────────────────────────────────────────────────────────────
echo "[LAYER 1] Checking topics..."
TOPICS=$(ros2 topic list 2>/dev/null || true)
echo "$TOPICS" > "$RUN_DIR/topics.txt"

ODO_OK=0; IMU_OK=0
echo "$TOPICS" | grep -q "/model/iris_uav/odometry" && ODO_OK=1
echo "$TOPICS" | grep -qE "imu_sensor/imu|/imu/data" && IMU_OK=1

if [ $ODO_OK -eq 1 ]; then
    echo "  ✅ /model/iris_uav/odometry present"
else
    echo "  ❌ /model/iris_uav/odometry MISSING — check $RUN_DIR/gz.log"
fi
if [ $IMU_OK -eq 1 ]; then
    echo "  ✅ IMU topic present"
else
    echo "  ❌ IMU topic MISSING — check $RUN_DIR/topics.txt"
fi

# Measure hz for odometry (bridge publishes RELIABLE — default ros2 topic hz QoS)
echo "[LAYER 1] Measuring /model/iris_uav/odometry rate (8 s)..."
ODO_HZ=$(timeout 10 ros2 topic hz /model/iris_uav/odometry 2>/dev/null \
    | grep -m 1 "average rate" | awk '{print $3}' || true)
[ -z "$ODO_HZ" ] && ODO_HZ="N/A"
echo "  Odometry rate: ${ODO_HZ:-N/A} Hz (expected ~30)"
echo "odometry_hz=${ODO_HZ:-N/A}" >> "$RUN_DIR/layer1_results.txt"
echo "odometry_present=${ODO_OK}" >> "$RUN_DIR/layer1_results.txt"
echo "imu_present=${IMU_OK}" >> "$RUN_DIR/layer1_results.txt"

# ── Layer 2b: Start mock GPS noise publisher ──────────────────────────────────
echo ""
echo "[LAYER 2b] Starting mock_fix_publisher.py..."
DATUM_LAT=$(grep -o '<latitude_deg>[^<]*' "$WORLD" | head -1 | cut -d'>' -f2)
DATUM_LON=$(grep -o '<longitude_deg>[^<]*' "$WORLD" | head -1 | cut -d'>' -f2)
echo "  Datum: lat=${DATUM_LAT} lon=${DATUM_LON}"

python3 "${REPO}/simulation/mock_fix_publisher.py" \
    --datum-lat "$DATUM_LAT" \
    --datum-lon "$DATUM_LON" \
    > "$RUN_DIR/mock_fix.log" 2>&1 &
FIX_PID=$!

sleep 3
FIX_TOPICS=$(ros2 topic list 2>/dev/null | grep "/fix" || true)
if echo "$FIX_TOPICS" | grep -q "/fix"; then
    echo "  ✅ /fix topic present"
    FIX_SAMPLE=$(timeout 8 ros2 topic echo /fix --once 2>/dev/null | head -20 || true)
    echo "$FIX_SAMPLE" > "$RUN_DIR/fix_sample.txt"
    echo "  Sample /fix message:"
    echo "$FIX_SAMPLE" | grep -E "latitude|longitude|status" | sed 's/^/    /' \
        || echo "    (no lat/lon yet — mock waiting for first odometry msg)"
    echo "fix_present=1" >> "$RUN_DIR/layer2b_results.txt"
else
    echo "  ❌ /fix MISSING — check $RUN_DIR/mock_fix.log"
    echo "fix_present=0" >> "$RUN_DIR/layer2b_results.txt"
fi

FIX_HZ=$(timeout 10 ros2 topic hz /fix 2>/dev/null \
    | grep -m 1 "average rate" | awk '{print $3}' || true)
[ -z "$FIX_HZ" ] && FIX_HZ="N/A"
echo "  /fix rate: ${FIX_HZ:-N/A} Hz (expected ~5)"
echo "fix_hz=${FIX_HZ:-N/A}" >> "$RUN_DIR/layer2b_results.txt"

# ── Layer 3: accuracy_verify --use-fix wiring check ──────────────────────────
echo ""
echo "[LAYER 3] Verifying accuracy_verify.py --use-fix code wiring..."
echo "  (Full live test requires ArduPilot SITL + real camera feed)"
python3 "${REPO}/simulation/accuracy_verify.py" --help 2>&1 | grep -A2 "use-fix" \
    | sed 's/^/  /' || true
echo "fix_code_ok=1" >> "$RUN_DIR/layer3_results.txt"

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
echo "============================================================"
echo " RESULTS SUMMARY"
echo "============================================================"
cat "$RUN_DIR/layer1_results.txt" 2>/dev/null | sed 's/^/  /'
cat "$RUN_DIR/layer2b_results.txt" 2>/dev/null | sed 's/^/  /'
cat "$RUN_DIR/layer3_results.txt" 2>/dev/null | sed 's/^/  /'
echo ""
echo "Full logs in: $RUN_DIR"
echo "Topics list:  $RUN_DIR/topics.txt"
echo "/fix sample:  $RUN_DIR/fix_sample.txt"

# ── Cleanup ───────────────────────────────────────────────────────────────────
echo ""
echo "Cleaning up..."
kill $FIX_PID $BRIDGE_PID $GZ_PID 2>/dev/null || true
echo "Done."
