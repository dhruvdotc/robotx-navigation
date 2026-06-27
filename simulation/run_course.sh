#!/usr/bin/env bash
# run_course.sh - Run any RobotX UAV course in headless or 4-window visual mode.
#
# HEADLESS MODE (default):
#   bash simulation/run_course.sh --course 1
#   bash simulation/run_course.sh --course 2
#   bash simulation/run_course.sh --course 3
#
#   No windows open. Progress is printed to the terminal as a percentage,
#   with a milestone line each time a waypoint is reached and a status line
#   every 5 seconds while flying between waypoints.
#
# VISUAL MODE (4 windows):
#   bash simulation/run_course.sh --course 1 --visual
#   bash simulation/run_course.sh --course 2 --visual
#   bash simulation/run_course.sh --course 3 --visual
#
#   Opens four separate windows:
#     1. Gazebo 3D view   -- animated VRX ocean + buoys + drone
#     2. SITL console     -- ArduCopter arm / mode / GPS log (tail of gz.log)
#     3. Camera detector  -- camera_live_feed.py text output + OpenCV window
#     4. GPS coordinates  -- live lat / lon / alt AGL / speed / mode
#
# After EITHER run completes (or Ctrl-C), all outputs are saved to:
#   simulation/sim_tests/run_N/
#     detections.csv       per-frame buoy GPS projections
#     accuracy_report.md   cross-referenced vs ground-truth positions
#     summary.json         machine-readable metrics
#     gz.log               Gazebo + SITL stdout/stderr
#     fly.log              fly_course.py output (headless) or echoed (visual)
#     verify.log           accuracy_verify.py output
#     camera.log           camera_live_feed.py output (visual mode)
#     gps.log              GPS display stream (visual mode)
#     map.png              top-down detection diagram
#
# Options:
#   --course 1|2|3   Select course world and flight path (default: 1)
#   --visual         Open 4 display windows (requires WSLg / X11 DISPLAY)
#   --no-fly         Start sim only; fly manually with fly_course.py
#   --speed N        Transit speed m/s (default: 1.5)
#
# Env overrides: ARDUPILOT, ARDUPILOT_GAZEBO, VRX_GZ, ROS_SETUP, ALTITUDE_M
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ALTITUDE_M="${ALTITUDE_M:-10}"
SIM_TESTS_DIR="${REPO_ROOT}/simulation/sim_tests"

# shellcheck source=gz_env.sh
source "${REPO_ROOT}/simulation/gz_env.sh"
SIM_VEHICLE="${ARDUPILOT}/Tools/autotest/sim_vehicle.py"

VISUAL=0
AUTO_FLY=1
COURSE=1
SPEED=1.5

while [[ $# -gt 0 ]]; do
  case "$1" in
    --course)    COURSE="$2";            shift 2 ;;
    --course=*)  COURSE="${1#--course=}"; shift ;;
    --visual)    VISUAL=1;               shift ;;
    --no-fly)    AUTO_FLY=0;             shift ;;
    --speed)     SPEED="$2";             shift 2 ;;
    --speed=*)   SPEED="${1#--speed=}";  shift ;;
    -h|--help)   grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown arg: $1 (try --help)" >&2; exit 2 ;;
  esac
done

# --------------------------------------------------------------------------- #
# Course configuration
# --------------------------------------------------------------------------- #
case "$COURSE" in
  1) WORLD="${REPO_ROOT}/simulation/gazebo/worlds/robotx_uav_course.sdf"
     COURSE_NAME="Course 1: Straight Navigation Channel"
     TOTAL_WP=4 ;;
  2) WORLD="${REPO_ROOT}/simulation/gazebo/worlds/course_2_search_field.sdf"
     COURSE_NAME="Course 2: Open Water Survey (Lawnmower)"
     TOTAL_WP=6 ;;
  3) WORLD="${REPO_ROOT}/simulation/gazebo/worlds/course_3_dogleg.sdf"
     COURSE_NAME="Course 3: L-Shaped Dogleg"
     TOTAL_WP=6 ;;
  *) echo "ERROR: --course must be 1, 2, or 3" >&2; exit 2 ;;
esac

# --------------------------------------------------------------------------- #
# Run directory (auto-increments: run_1, run_2, ...)
# --------------------------------------------------------------------------- #
mkdir -p "$SIM_TESTS_DIR"
RUN_N=1
for d in "${SIM_TESTS_DIR}"/run_*/; do
  [ -d "$d" ] || continue
  n="${d%/}"; n="${n##*run_}"
  [[ "$n" =~ ^[0-9]+$ ]] && (( n + 1 > RUN_N )) && RUN_N=$(( n + 1 ))
done
RUN_DIR="${SIM_TESTS_DIR}/run_${RUN_N}"
mkdir -p "$RUN_DIR"

MODE_STR="headless"
[ "$VISUAL" -eq 1 ] && MODE_STR="visual (4 windows)"

echo "================================================================"
echo " RobotX sim: run_${RUN_N}  |  ${COURSE_NAME}"
echo " Mode   : ${MODE_STR}"
echo " Output : ${RUN_DIR}"
echo "================================================================"
echo

# --------------------------------------------------------------------------- #
# Preflight checks
# --------------------------------------------------------------------------- #
[ -f "$WORLD" ]       || { echo "ERROR: world not found: $WORLD" >&2; exit 1; }
[ -f "$SIM_VEHICLE" ] || { echo "ERROR: sim_vehicle.py not at $SIM_VEHICLE" >&2; exit 1; }
[ -f "$ROS_SETUP" ]   || { echo "ERROR: ROS setup not at $ROS_SETUP" >&2; exit 1; }
[ -f "${VRX_GZ}/lib/libWaveVisual.so" ] || {
  echo "ERROR: VRX ocean missing. Set VRX_GZ to your vrx_gz install." >&2; exit 1; }
if [ "$VISUAL" -eq 1 ]; then
  command -v xterm >/dev/null 2>&1 || { echo "ERROR: xterm not found (apt install xterm)." >&2; exit 1; }
  [ -n "${DISPLAY:-}" ] || { echo "ERROR: \$DISPLAY empty; need WSLg/X11 for visual mode." >&2; exit 1; }
fi

# --------------------------------------------------------------------------- #
# PIDs to clean up on exit
# --------------------------------------------------------------------------- #
GZ_PID=""; SITL_PID=""; BRIDGE_PID=""
FLY_PID=""; VERIFY_PID=""
SITL_XT_PID=""; CAM_XT_PID=""; GPS_XT_PID=""

ts() { date '+%H:%M:%S'; }
pmsg() { echo "[$(ts)] $*"; }

cleanup() {
  echo
  pmsg "Tearing down run_${RUN_N}..."
  for pid_var in GPS_XT_PID CAM_XT_PID SITL_XT_PID FLY_PID VERIFY_PID BRIDGE_PID SITL_PID GZ_PID; do
    eval pid="\${${pid_var}}"
    [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
  done
  pkill -f "gz sim.*$(basename "$WORLD" .sdf)"    2>/dev/null || true
  pkill -f "ros_gz_image image_bridge /drone"     2>/dev/null || true
  pkill -f "arducopter.*-I0"                      2>/dev/null || true
  pkill -f "gps_display.py"                       2>/dev/null || true

  # Stamp run metadata into summary.json if it was written
  local sjson="${RUN_DIR}/summary.json"
  if [ -f "$sjson" ]; then
    python3 - "$sjson" "$RUN_N" "$RUN_DIR" "$COURSE" "$COURSE_NAME" "$MODE_STR" <<'PY'
import json, sys, os
path, run_n, run_dir, course_n, course_name, mode = \
    sys.argv[1], int(sys.argv[2]), sys.argv[3], int(sys.argv[4]), sys.argv[5], sys.argv[6]
with open(path) as f:
    d = json.load(f)
d.update(run=run_n, course=course_n, course_name=course_name, mode=mode)
for fname in os.listdir(run_dir):
    if fname.endswith(".mp4"):
        d.setdefault("files", {})["recording"] = fname
    elif fname == "gz.log":
        d.setdefault("files", {})["gz_log"] = fname
with open(path, "w") as f:
    json.dump(d, f, indent=2)
print(f"[INFO] summary.json updated: run={run_n}, mode={mode}")
PY
  fi

  # Generate detection map
  pmsg "Generating detection map (map.png)..."
  if python3 "${REPO_ROOT}/simulation/plot_run.py" "$RUN_DIR" 2>/dev/null; then
    pmsg "map.png written."
  else
    pmsg "WARN: plot_run.py failed (matplotlib installed?)."
  fi

  echo
  echo "================================================================"
  echo " run_${RUN_N} complete. Outputs in:"
  echo "   ${RUN_DIR}"
  echo "================================================================"
  ls -lh "$RUN_DIR" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# --------------------------------------------------------------------------- #
# 1. Gazebo
# --------------------------------------------------------------------------- #
GZ_LOG="${RUN_DIR}/gz.log"
if [ "$VISUAL" -eq 1 ]; then
  pmsg "[1/4] Starting Gazebo (GUI window)..."
  gz sim -v3 -r "$WORLD" >"$GZ_LOG" 2>&1 &
else
  pmsg "[1/4] Starting Gazebo (headless server)..."
  gz sim -v3 -s -r "$WORLD" >"$GZ_LOG" 2>&1 &
fi
GZ_PID=$!

pmsg "      Waiting for /drone/camera topic..."
ready=0
for _ in $(seq 1 120); do
  if gz topic -l 2>/dev/null | grep -qx "/drone/camera"; then ready=1; break; fi
  if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "ERROR: Gazebo exited early (see $GZ_LOG)." >&2; exit 1
  fi
  sleep 1
done
[ "$ready" -eq 1 ] \
  && pmsg "      Gazebo up: /drone/camera publishing." \
  || pmsg "      WARN: camera topic not seen; continuing anyway."

# --------------------------------------------------------------------------- #
# 2. SITL
# --------------------------------------------------------------------------- #
pmsg "[2/4] Starting ArduPilot SITL..."
cd "$REPO_ROOT"
env -u DISPLAY python3 "$SIM_VEHICLE" \
  -v ArduCopter -f gazebo-iris --model JSON --no-rebuild -I0 \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14552 \
  >>"$GZ_LOG" 2>&1 &
SITL_PID=$!

# --------------------------------------------------------------------------- #
# 3. gz -> ROS image bridge
# --------------------------------------------------------------------------- #
pmsg "[3/4] Starting gz->ROS image bridge..."
bash -c "source '${ROS_SETUP}'; export GZ_VERSION=harmonic; \
  exec ros2 run ros_gz_image image_bridge /drone/camera" \
  >>"$GZ_LOG" 2>&1 &
BRIDGE_PID=$!

# --------------------------------------------------------------------------- #
# 4. Visual mode: open the 4 xterm windows
# --------------------------------------------------------------------------- #
if [ "$VISUAL" -eq 1 ]; then
  XTERM=(xterm -fa "Monospace" -fs 11)

  pmsg "[4/4] Opening display windows..."

  # Window: SITL + Gazebo log (tail gz.log so the xterm shows live SITL output)
  "${XTERM[@]}" -T "SITL + Gazebo log" -geometry 110x32+10+20 \
    -e bash -c "tail -n 60 -f '${GZ_LOG}'" &
  SITL_XT_PID=$!
  pmsg "      Window 2: SITL console (SITL + Gazebo log)"

  # Window: camera_live_feed.py (opens its own OpenCV detection window)
  CAM_CMD="cd '${REPO_ROOT}' && source '${ROS_SETUP}' && \
    echo '=== camera_live_feed.py  [nadir camera] ===' && \
    python3 camera_live_feed.py \
      --ros-topic /drone/camera \
      --no-undistort \
      --altitude-m '${ALTITUDE_M}' 2>&1 | tee '${RUN_DIR}/camera.log'"
  "${XTERM[@]}" -T "Camera Detector" -geometry 110x28+750+20 \
    -e bash -lc "$CAM_CMD" &
  CAM_XT_PID=$!
  pmsg "      Window 3: camera_live_feed.py (+ OpenCV detection window)"

  # Window: live GPS coordinates
  "${XTERM[@]}" -T "Live GPS Coords" -geometry 70x16+10+560 \
    -e bash -c "python3 '${REPO_ROOT}/simulation/gps_display.py' 2>&1 | tee '${RUN_DIR}/gps.log'" &
  GPS_XT_PID=$!
  pmsg "      Window 4: GPS display (lat / lon / alt / speed)"
fi

# --------------------------------------------------------------------------- #
# Wait for SITL GPS/EKF fix
# --------------------------------------------------------------------------- #
echo
pmsg "Waiting for SITL GPS/EKF fix (this takes 30-90 s)..."
python3 - <<'PY'
import sys, time
from pymavlink import mavutil
try:
    m = mavutil.mavlink_connection("udp:127.0.0.1:14552")
    m.wait_heartbeat(timeout=90)
    deadline = time.time() + 120
    while time.time() < deadline:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
        if msg and msg.lat != 0:
            print(f"   SITL READY: GPS lat={msg.lat/1e7:.6f}  lon={msg.lon/1e7:.6f}")
            sys.exit(0)
    print("WARN: GPS readiness timed out.", file=sys.stderr)
except Exception as e:
    print(f"WARN: readiness probe failed: {e}", file=sys.stderr)
PY

# --------------------------------------------------------------------------- #
# Start accuracy_verify.py (background; writes detections.csv + report)
# --------------------------------------------------------------------------- #
echo
pmsg "Starting accuracy_verify.py (background detection logger -> ${RUN_DIR})..."
bash -c "source '${ROS_SETUP}'; python3 '${REPO_ROOT}/simulation/accuracy_verify.py' \
  --world '${WORLD}' \
  --connect udp:127.0.0.1:14551 \
  --out-dir '${RUN_DIR}' \
  --report-dir '${RUN_DIR}' \
  --summary-json '${RUN_DIR}/summary.json'" \
  >"${RUN_DIR}/verify.log" 2>&1 &
VERIFY_PID=$!

# --------------------------------------------------------------------------- #
# Auto-fly (or wait for manual flight)
# --------------------------------------------------------------------------- #
FLY_LOG="${RUN_DIR}/fly.log"

if [ "$AUTO_FLY" -eq 0 ]; then
  echo
  pmsg "Sim running. Fly manually when ready:"
  pmsg "  python3 simulation/fly_course.py --course ${COURSE} --connect udp:127.0.0.1:14550"
  pmsg "Ctrl-C to stop and save outputs."
  wait "$GZ_PID" 2>/dev/null || true

elif [ "$VISUAL" -eq 1 ]; then
  # ---- VISUAL AUTO-FLY ---------------------------------------------------- #
  echo
  echo "================================================================"
  echo " ALL WINDOWS ARE UP:"
  echo "   1. Gazebo 3D view        (drone + buoys + animated ocean)"
  echo "   2. SITL console xterm    (arm / mode / GPS log)"
  echo "   3. Camera detector xterm (detections + OpenCV window)"
  echo "   4. GPS coords xterm      (live lat / lon / alt / speed)"
  echo
  echo " Auto-fly starts in 10 s -- arrange your windows now."
  echo "================================================================"
  echo
  sleep 10

  pmsg "Auto-fly: ${COURSE_NAME}"
  python3 "${REPO_ROOT}/simulation/fly_course.py" \
    --connect udp:127.0.0.1:14550 \
    --course "${COURSE}" \
    --speed "${SPEED}" \
    --countdown 0 \
    2>&1 | tee "$FLY_LOG"

  pmsg "Flight complete. Waiting for accuracy_verify to finish..."
  sleep 3
  kill "$VERIFY_PID" 2>/dev/null || true
  wait "$VERIFY_PID" 2>/dev/null || true

else
  # ---- HEADLESS AUTO-FLY with PROGRESS ------------------------------------ #
  echo
  pmsg "Auto-fly starting in 5 s..."
  sleep 5

  python3 "${REPO_ROOT}/simulation/fly_course.py" \
    --connect udp:127.0.0.1:14550 \
    --course "${COURSE}" \
    --speed "${SPEED}" \
    --countdown 0 \
    >"$FLY_LOG" 2>&1 &
  FLY_PID=$!

  echo
  pmsg "=== FLIGHT IN PROGRESS | ${COURSE_NAME} | ${TOTAL_WP} waypoints ==="
  echo

  LAST_REACHED=0
  LAST_STATUS_TIME=0

  while kill -0 "$FLY_PID" 2>/dev/null; do
    NOW_TS=$(date +%s)
    NOW_STR=$(ts)

    REACHED=$(grep -c "reached .* -- held" "$FLY_LOG" 2>/dev/null || echo 0)
    PCT=$(( REACHED * 100 / TOTAL_WP ))

    # Print a milestone line for each newly completed waypoint
    while [ "$LAST_REACHED" -lt "$REACHED" ]; do
      LAST_REACHED=$(( LAST_REACHED + 1 ))
      LABEL=$(grep "reached .* -- held" "$FLY_LOG" 2>/dev/null \
              | sed -n "${LAST_REACHED}p" \
              | sed 's/.*reached //' | sed 's/ -- held.*//' || true)
      echo "[${NOW_STR}] ${PCT}% | waypoint ${LAST_REACHED}/${TOTAL_WP} reached: ${LABEL}"
      LAST_STATUS_TIME=0  # trigger immediate status print after milestone
    done

    # Print a status line every 5 seconds between milestones
    if [ $(( NOW_TS - LAST_STATUS_TIME )) -ge 5 ]; then
      LAST_STATUS_TIME=$NOW_TS
      if grep -q "FLIGHT START" "$FLY_LOG" 2>/dev/null; then
        CURRENT=$(grep "\-> .* (N=" "$FLY_LOG" 2>/dev/null | tail -1 \
                  | sed 's/.*-> //' | sed 's/ (N=.*//' || true)
        echo "[${NOW_STR}] ${PCT}% | ${LAST_REACHED}/${TOTAL_WP} done | flying to: ${CURRENT:-?}"
      elif grep -q "ARMED" "$FLY_LOG" 2>/dev/null; then
        echo "[${NOW_STR}]   0% | armed -- climbing to altitude..."
      else
        echo "[${NOW_STR}]   0% | waiting for GPS / arming..."
      fi
    fi

    sleep 2
  done

  wait "$FLY_PID" || true
  echo
  pmsg "Flight complete. Waiting for accuracy_verify to finish..."
  sleep 3
  kill "$VERIFY_PID" 2>/dev/null || true
  wait "$VERIFY_PID" 2>/dev/null || true
fi
