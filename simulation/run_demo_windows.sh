#!/usr/bin/env bash
# RobotX UAV course -- LIVE DEMO launcher (real windows on the WSLg display).
#
# Opens the three REAL things to screen-record, no synthesis/overlays:
#   1. The Gazebo GUI  -- `gz sim -r` (not headless), showing the animated VRX
#      ocean + the gate buoys + the drone.                       [its own window]
#   2. SITL + MAVProxy -- the genuine `sim_vehicle.py` / MAVProxy console; its
#      real arm / mode-change / GPS text as ArduCopter prints it.   [xterm "SITL"]
#   3. camera_live_feed.py --ros-topic /drone/camera -- the actual detector
#      against the live camera topic, its real [INFO]/[GPS] lines.  [xterm "CAMERA"]
#
# A background gz->ROS image bridge feeds (3). SITL exposes MAVProxy outs:
#   udp:127.0.0.1:14550  -> simulation/fly_course.py    (the cinematic flight)
#   udp:127.0.0.1:14551  -> simulation/accuracy_verify.py (Task C logger)
#   udp:127.0.0.1:14552  -> this script's readiness probe (released once GPS is up)
#
# Flow: this script brings the 3 windows up, waits until SITL has a GPS/EKF fix,
# then prints "READY" + the exact flight command. YOU arrange the windows, start
# your screen recording, then run fly_course.py (which gives a 10 s countdown).
#
# Usage:
#   bash simulation/run_demo_windows.sh            # bring the 3 windows up
#   bash simulation/run_demo_windows.sh --fly      # ...and auto-run fly_course after READY
# Env knobs: ROS_SETUP (default /opt/ros/humble/setup.bash), ALTITUDE_M (10).
#
# Ctrl-C this launcher to tear everything down.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${REPO_ROOT}/simulation/gazebo/worlds/robotx_uav_course.sdf"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ALTITUDE_M="${ALTITUDE_M:-10}"

# shellcheck source=gz_env.sh
source "${REPO_ROOT}/simulation/gz_env.sh"
SIM_VEHICLE="${ARDUPILOT}/Tools/autotest/sim_vehicle.py"

AUTO_FLY=0
for arg in "$@"; do
  case "$arg" in
    --fly) AUTO_FLY=1 ;;
    -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown arg: $arg (try --help)" >&2; exit 2 ;;
  esac
done

# --- Preflight ---------------------------------------------------------------
need() { command -v "$1" >/dev/null 2>&1 || { echo "ERROR: '$1' not found ($2)." >&2; exit 1; }; }
need gz "Gazebo Harmonic"
need xterm "apt install xterm"
[ -n "${DISPLAY:-}" ] || { echo "ERROR: \$DISPLAY is empty; need a WSLg/X display." >&2; exit 1; }
[ -f "$WORLD" ] || { echo "ERROR: world not found: $WORLD" >&2; exit 1; }
[ -f "$SIM_VEHICLE" ] || { echo "ERROR: sim_vehicle.py not at $SIM_VEHICLE (set ARDUPILOT)." >&2; exit 1; }
[ -f "$ROS_SETUP" ] || { echo "ERROR: ROS setup not at $ROS_SETUP (set ROS_SETUP)." >&2; exit 1; }
[ -f "${VRX_GZ}/lib/libWaveVisual.so" ] || { echo "ERROR: VRX ocean missing (set VRX_GZ)." >&2; exit 1; }

XTERM=(xterm -fa Monospace -fs 11)
GZ_PID=""; BRIDGE_PID=""; SITL_XTERM_PID=""; CAM_XTERM_PID=""
cleanup() {
  echo; echo "Tearing down demo..."
  [ -n "$BRIDGE_PID" ]      && kill "$BRIDGE_PID"      2>/dev/null || true
  [ -n "$SITL_XTERM_PID" ]  && kill "$SITL_XTERM_PID"  2>/dev/null || true
  [ -n "$CAM_XTERM_PID" ]   && kill "$CAM_XTERM_PID"   2>/dev/null || true
  [ -n "$GZ_PID" ]          && kill "$GZ_PID"          2>/dev/null || true
  pkill -f "gz sim.*robotx_uav_course" 2>/dev/null || true
  pkill -f "ros_gz_image image_bridge /drone/camera" 2>/dev/null || true
  pkill -f "arducopter.*-I0" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "=== RobotX UAV course: LIVE DEMO windows ==="
echo "World: $WORLD"

# --- Window 1: Gazebo GUI (real ocean) --------------------------------------
echo "[1/3] Launching Gazebo GUI (gz sim -r)..."
gz sim -v3 -r "$WORLD" >"${REPO_ROOT}/simulation/.demo_gz.log" 2>&1 &
GZ_PID=$!

echo "      waiting for /drone/camera to publish..."
ready=0
for _ in $(seq 1 120); do
  if gz topic -l 2>/dev/null | grep -qx "/drone/camera"; then ready=1; break; fi
  if ! kill -0 "$GZ_PID" 2>/dev/null; then echo "ERROR: gz exited (see simulation/.demo_gz.log)." >&2; exit 1; fi
  sleep 1
done
[ "$ready" -eq 1 ] && echo "      Gazebo up: /drone/camera publishing." || echo "      WARN: camera topic not seen; continuing."

# --- Background: gz -> ROS image bridge (feeds camera_live_feed) --------------
echo "      starting gz->ROS image bridge (background)..."
bash -c "source '$ROS_SETUP'; export GZ_VERSION=harmonic; \
  exec ros2 run ros_gz_image image_bridge /drone/camera" \
  >"${REPO_ROOT}/simulation/.demo_bridge.log" 2>&1 &
BRIDGE_PID=$!

# --- Window 2: SITL + MAVProxy (real console) --------------------------------
# CWD must be the repo root so SITL reuses the repo-root eeprom.bin where
# FRAME_CLASS=1 is persisted (else arming fails "Check frame class"). No -w.
# `env -u DISPLAY` keeps sim_vehicle from spawning a SEPARATE xterm for the
# ArduCopter binary (it falls back to running it inline, logging to
# /tmp/ArduCopter.log) -- so MAVProxy is the ONLY thing in this window, giving
# exactly the three windows to record. MAVProxy is a text console; no DISPLAY needed.
echo "[2/3] Launching SITL + MAVProxy xterm..."
SITL_CMD="cd '$REPO_ROOT' && echo '=== SITL + MAVProxy (real ArduCopter console) ===' && \
  env -u DISPLAY python3 '$SIM_VEHICLE' -v ArduCopter -f gazebo-iris --model JSON --no-rebuild -I0 \
    --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14552"
"${XTERM[@]}" -T "SITL + MAVProxy console" -geometry 104x30+10+20 -e bash -lc "$SITL_CMD" &
SITL_XTERM_PID=$!

# --- Window 3: camera_live_feed.py against the live ROS topic -----------------
echo "[3/3] Launching camera_live_feed.py xterm (live /drone/camera)..."
CAM_CMD="cd '$REPO_ROOT' && source '$ROS_SETUP' && \
  echo '=== camera_live_feed.py --ros-topic /drone/camera (real detector) ===' && \
  python3 camera_live_feed.py --ros-topic /drone/camera --no-undistort --altitude-m '$ALTITUDE_M'"
"${XTERM[@]}" -T "camera_live_feed.py  [INFO]/[GPS]" -geometry 104x30+760+20 -e bash -lc "$CAM_CMD" &
CAM_XTERM_PID=$!

# --- Wait for SITL GPS/EKF readiness via the 14552 probe ---------------------
echo
echo "Waiting for SITL to get a GPS/EKF fix (this is the genuine ArduCopter boot)..."
python3 - <<'PY' || echo "WARN: readiness probe timed out; check the SITL window manually."
import time
from pymavlink import mavutil
m = mavutil.mavlink_connection("udp:127.0.0.1:14552")
m.wait_heartbeat(timeout=90)
deadline = time.time() + 120
while time.time() < deadline:
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
    if msg and msg.lat != 0:
        print(f"   SITL READY: GPS lat={msg.lat/1e7:.6f} lon={msg.lon/1e7:.6f}")
        break
PY

echo
echo "============================================================"
echo " ALL THREE WINDOWS ARE UP AND READY."
echo "   1) Gazebo GUI window  (animated ocean + gates + drone)"
echo "   2) 'SITL + MAVProxy console' xterm   (arm/mode/GPS text)"
echo "   3) 'camera_live_feed.py' xterm       ([INFO]/[GPS] lines)"
echo
echo " NOW: arrange the windows, START YOUR SCREEN RECORDING, then fly:"
echo "     python3 simulation/fly_course.py            # 10 s countdown, then flies"
echo "   (optional, in a 4th terminal, to log + verify accuracy -- Task C):"
echo "     python3 simulation/accuracy_verify.py --connect udp:127.0.0.1:14551"
echo "============================================================"

if [ "$AUTO_FLY" -eq 1 ]; then
  echo "--fly: auto-launching fly_course.py in 5 s (its own 10 s countdown follows)..."
  sleep 5
  python3 "${REPO_ROOT}/simulation/fly_course.py" --connect udp:127.0.0.1:14550 || true
fi

echo "Launcher holding (gz + bridge). Ctrl-C to tear everything down."
wait "$GZ_PID"
