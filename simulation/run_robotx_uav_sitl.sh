#!/usr/bin/env bash
# Launch the RobotX UAV course (Gazebo Harmonic) + ArduPilot SITL (lockstep FDM).
#
# The whole point of this script is to encode the GZ_SIM_RESOURCE_PATH so the
# world's model:// includes resolve without fat-fingering:
#   - simulation/gazebo/models (THIS repo)  -> iris_uav, gimbal_nadir   [MUST be first]
#   - ardupilot_gazebo/models               -> iris_with_standoffs
#   - ardupilot_gazebo/worlds               -> (reference worlds)
#
# Usage:
#   bash simulation/run_robotx_uav_sitl.sh            # gz GUI + SITL
#   bash simulation/run_robotx_uav_sitl.sh --headless # gz server-only + SITL
#   bash simulation/run_robotx_uav_sitl.sh --no-sitl  # gz only (no flight stack)
#   bash simulation/run_robotx_uav_sitl.sh --wipe     # SITL with -w (re-wipes EEPROM;
#                                                       reintroduces FRAME_CLASS=0 arm bug)
#
# Connect a GCS/script to SITL MAVLink at tcp:127.0.0.1:5760. FDM is UDP 9002.
# Stop with Ctrl-C (both gz and SITL are torn down by the trap).
#
# No -u: ROS/ArduPilot setup scripts reference vars that may be unset.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${REPO_ROOT}/simulation/gazebo/worlds/robotx_uav_course.sdf"

# Toolchains live OUTSIDE the repo (override via env if relocated).
ARDUPILOT_GAZEBO="${ARDUPILOT_GAZEBO:-$HOME/ardupilot_gazebo}"
ARDUPILOT="${ARDUPILOT:-$HOME/ardupilot}"
SIM_VEHICLE="${ARDUPILOT}/Tools/autotest/sim_vehicle.py"

# --- Gazebo environment (the resource paths this script exists to get right) ---
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ARDUPILOT_GAZEBO}/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_RESOURCE_PATH="${REPO_ROOT}/simulation/gazebo/models:${ARDUPILOT_GAZEBO}/models:${ARDUPILOT_GAZEBO}/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

HEADLESS=0
NO_SITL=0
WIPE=""
for arg in "$@"; do
  case "$arg" in
    --headless) HEADLESS=1 ;;
    --no-sitl)  NO_SITL=1 ;;
    --wipe)     WIPE="-w" ;;
    -h|--help)  grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown arg: $arg (try --help)" >&2; exit 2 ;;
  esac
done

# --- Preflight: fail loudly with a fix hint instead of a cryptic gz error ---
[ -f "$WORLD" ] || { echo "ERROR: world not found: $WORLD" >&2; exit 1; }
[ -d "${ARDUPILOT_GAZEBO}/models/iris_with_standoffs" ] || {
  echo "ERROR: iris_with_standoffs not in ${ARDUPILOT_GAZEBO}/models (set ARDUPILOT_GAZEBO)." >&2; exit 1; }
ls "${ARDUPILOT_GAZEBO}/build/"*.so >/dev/null 2>&1 || {
  echo "ERROR: ardupilot_gazebo plugins (*.so) not built in ${ARDUPILOT_GAZEBO}/build." >&2; exit 1; }
if [ "$NO_SITL" -eq 0 ]; then
  [ -f "$SIM_VEHICLE" ] || { echo "ERROR: sim_vehicle.py not at $SIM_VEHICLE (set ARDUPILOT)." >&2; exit 1; }
fi

echo "=== RobotX UAV course: SITL + Gazebo ==="
echo "World : $WORLD"
echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"

GZ_PID=""
SITL_PID=""
cleanup() {
  echo "Tearing down..."
  [ -n "$SITL_PID" ] && kill "$SITL_PID" 2>/dev/null || true
  [ -n "$GZ_PID" ] && kill "$GZ_PID" 2>/dev/null || true
  pkill -f "gz sim.*robotx_uav_course" 2>/dev/null || true
  pkill -f "arducopter.*-I0" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# --- Launch Gazebo first: the ArduPilotPlugin must be listening before SITL connects ---
GZ_FLAGS="-v4 -r"
[ "$HEADLESS" -eq 1 ] && GZ_FLAGS="$GZ_FLAGS -s"
echo "Launching: gz sim $GZ_FLAGS <world>"
# shellcheck disable=SC2086
gz sim $GZ_FLAGS "$WORLD" &
GZ_PID=$!

echo "Waiting for /drone/camera to come up..."
ready=0
for _ in $(seq 1 90); do
  if gz topic -l 2>/dev/null | grep -qx "/drone/camera"; then ready=1; break; fi
  if ! kill -0 "$GZ_PID" 2>/dev/null; then echo "ERROR: gz sim exited during startup." >&2; exit 1; fi
  sleep 1
done
[ "$ready" -eq 1 ] && echo "Gazebo up: /drone/camera publishing." || echo "WARN: /drone/camera not seen yet; continuing."

if [ "$NO_SITL" -eq 1 ]; then
  echo "--no-sitl: Gazebo only. Ctrl-C to stop."
  wait "$GZ_PID"
  exit 0
fi

# --- Launch ArduPilot SITL (gazebo-iris JSON FDM, lockstep). --no-mavproxy: connect your own GCS. ---
echo "Launching SITL (ArduCopter, gazebo-iris). MAVLink -> tcp:127.0.0.1:5760"
python3 "$SIM_VEHICLE" -v ArduCopter -f gazebo-iris --model JSON \
  --no-mavproxy --no-rebuild -I0 $WIPE &
SITL_PID=$!

echo "Running. Ctrl-C to stop both."
wait "$SITL_PID"
