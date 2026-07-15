#!/usr/bin/env bash
# run_sim_test.sh - Run a full RobotX UAV course sim test and collect all outputs.
#
# Creates simulation/sim_tests/run_N/ and writes everything from the run there:
#   detections.csv        - per-frame buoy detections with GPS estimates
#   accuracy_report.md    - cross-referenced vs ground-truth buoy positions
#   summary.json          - machine-readable metrics (duration, errors, buoys found)
#   gz.log                - Gazebo stdout/stderr
#   map.png               - top-down detection diagram vs ground truth
#   recording.mp4         - screen recording (--gui + ffmpeg, optional)
#
# A minimum 15-second sustained flight is enforced. Runs that don't meet it are
# flagged in the report but the folder is still saved so nothing is silently lost.
#
# Usage:
#   bash simulation/run_sim_test.sh --course 1   # Course 1: straight nav channel (default)
#   bash simulation/run_sim_test.sh --course 2   # Course 2: open-water survey lawnmower
#   bash simulation/run_sim_test.sh --course 3   # Course 3: L-shaped dogleg
#   bash simulation/run_sim_test.sh --gui        # show Gazebo window (for recording)
#   bash simulation/run_sim_test.sh --no-fly     # launch sim but don't auto-fly
#   bash simulation/run_sim_test.sh --no-record  # skip screen recording even if ffmpeg available
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

HEADLESS=1
AUTO_FLY=1
RECORD=1
COURSE=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --course)   COURSE="$2"; shift 2 ;;
    --course=*) COURSE="${1#--course=}"; shift ;;
    --gui)       HEADLESS=0; shift ;;
    --no-fly)    AUTO_FLY=0; shift ;;
    --no-record) RECORD=0; shift ;;
    -h|--help)   grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown arg: $1 (try --help)" >&2; exit 2 ;;
  esac
done

# Select world file based on course number
case "$COURSE" in
  1) WORLD="${REPO_ROOT}/simulation/gazebo/worlds/robotx_uav_course.sdf"
     COURSE_NAME="Course 1: Straight Navigation Channel" ;;
  2) WORLD="${REPO_ROOT}/simulation/gazebo/worlds/course_2_search_field.sdf"
     COURSE_NAME="Course 2: Open Water Survey (Lawnmower)" ;;
  3) WORLD="${REPO_ROOT}/simulation/gazebo/worlds/course_3_dogleg.sdf"
     COURSE_NAME="Course 3: L-Shaped Dogleg" ;;
  *) echo "ERROR: --course must be 1, 2, or 3" >&2; exit 2 ;;
esac

# --------------------------------------------------------------------------- #
# Determine run number
# --------------------------------------------------------------------------- #
mkdir -p "$SIM_TESTS_DIR"
RUN_N=1
for d in "${SIM_TESTS_DIR}"/run_*/; do
  [ -d "$d" ] || continue
  n="${d%/}"; n="${n##*run_}"
  [[ "$n" =~ ^[0-9]+$ ]] && (( n+1 > RUN_N )) && RUN_N=$((n+1))
done
RUN_DIR="${SIM_TESTS_DIR}/run_${RUN_N}"
mkdir -p "$RUN_DIR"

echo "=== RobotX sim test: run_${RUN_N} | ${COURSE_NAME} ==="
echo "World:      $WORLD"
echo "Output dir: $RUN_DIR"

# --------------------------------------------------------------------------- #
# Preflight checks
# --------------------------------------------------------------------------- #
[ -f "$WORLD" ]       || { echo "ERROR: world not found: $WORLD" >&2; exit 1; }
[ -f "$SIM_VEHICLE" ] || { echo "ERROR: sim_vehicle.py not at $SIM_VEHICLE (set ARDUPILOT)." >&2; exit 1; }
[ -f "$ROS_SETUP" ]   || { echo "ERROR: ROS setup not at $ROS_SETUP (set ROS_SETUP)." >&2; exit 1; }
[ -f "${VRX_GZ}/lib/libWaveVisual.so" ] || {
  echo "ERROR: VRX ocean missing. Set VRX_GZ to your vrx_gz install." >&2; exit 1; }

# --------------------------------------------------------------------------- #
# Cleanup on exit
# --------------------------------------------------------------------------- #
GZ_PID=""; SITL_PID=""; BRIDGE_PID=""; FLY_PID=""; VERIFY_PID=""; FFMPEG_PID=""
cleanup() {
  echo; echo "[run_${RUN_N}] Tearing down..."
  [ -n "$FFMPEG_PID" ]  && kill "$FFMPEG_PID"  2>/dev/null || true
  [ -n "$VERIFY_PID" ]  && kill "$VERIFY_PID"  2>/dev/null || true
  [ -n "$FLY_PID" ]     && kill "$FLY_PID"     2>/dev/null || true
  [ -n "$BRIDGE_PID" ]  && kill "$BRIDGE_PID"  2>/dev/null || true
  [ -n "$SITL_PID" ]    && kill "$SITL_PID"    2>/dev/null || true
  [ -n "$GZ_PID" ]      && kill "$GZ_PID"      2>/dev/null || true
  pkill -f "gz sim.*robotx_uav_course"          2>/dev/null || true
  pkill -f "ros_gz_image image_bridge /drone"   2>/dev/null || true
  pkill -f "arducopter.*-I0"                    2>/dev/null || true

  # Stamp the run number into the summary.json if it was written
  local sjson="${RUN_DIR}/summary.json"
  if [ -f "$sjson" ]; then
    python3 - "$sjson" "$RUN_N" "$RUN_DIR" "$COURSE" "$COURSE_NAME" <<'PY'
import json, sys, os
path, run_n, run_dir, course_n, course_name = sys.argv[1], int(sys.argv[2]), sys.argv[3], int(sys.argv[4]), sys.argv[5]
with open(path) as f:
    d = json.load(f)
d["run"] = run_n
d["course"] = course_n
d["course_name"] = course_name
# record files that exist in the run dir
for fname in os.listdir(run_dir):
    if fname.endswith(".mp4"):
        d.setdefault("files", {})["recording"] = fname
    elif fname == "gz.log":
        d.setdefault("files", {})["gz_log"] = fname
with open(path, "w") as f:
    json.dump(d, f, indent=2)
print(f"[INFO] summary.json updated with run={run_n}")
PY
  fi

  # Generate the detection map diagram
  if python3 "${REPO_ROOT}/simulation/plot_run.py" "$RUN_DIR" 2>/dev/null; then
    echo "[run_${RUN_N}] map.png written."
  else
    echo "[run_${RUN_N}] WARN: plot_run.py failed (matplotlib installed?)"
  fi

  echo "[run_${RUN_N}] Done. Outputs in: $RUN_DIR"
  ls -lh "$RUN_DIR" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# --------------------------------------------------------------------------- #
# 1. Gazebo
# --------------------------------------------------------------------------- #
GZ_LOG="${RUN_DIR}/gz.log"
if [ "$HEADLESS" -eq 1 ]; then
  echo "[1/4] Starting Gazebo (headless)..."
  gz sim -v3 -s -r "$WORLD" >"$GZ_LOG" 2>&1 &
else
  echo "[1/4] Starting Gazebo (GUI)..."
  gz sim -v3 -r "$WORLD" >"$GZ_LOG" 2>&1 &
fi
GZ_PID=$!

echo "      waiting for /drone/camera topic..."
ready=0
for _ in $(seq 1 120); do
  if gz topic -l 2>/dev/null | grep -qx "/drone/camera"; then ready=1; break; fi
  if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "ERROR: Gazebo exited (see $GZ_LOG)." >&2; exit 1
  fi
  sleep 1
done
[ "$ready" -eq 1 ] && echo "      Gazebo up: /drone/camera publishing." \
  || echo "      WARN: camera topic not seen; continuing anyway."

# --------------------------------------------------------------------------- #
# 2. SITL
# --------------------------------------------------------------------------- #
echo "[2/4] Starting ArduPilot SITL..."
cd "$REPO_ROOT"
env -u DISPLAY python3 "$SIM_VEHICLE" \
  -v ArduCopter -f gazebo-iris --model JSON --no-rebuild -I0 \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14552 \
  >>"$GZ_LOG" 2>&1 &
SITL_PID=$!

# --------------------------------------------------------------------------- #
# 3. gz -> ROS bridge
# --------------------------------------------------------------------------- #
echo "[3/4] Starting gz->ROS image bridge..."
bash -c "source '${ROS_SETUP}'; export GZ_VERSION=harmonic; \
  exec ros2 run ros_gz_image image_bridge /drone/camera" \
  >>"$GZ_LOG" 2>&1 &
BRIDGE_PID=$!

# --------------------------------------------------------------------------- #
# 4. Optional screen recording
# --------------------------------------------------------------------------- #
if [ "$RECORD" -eq 1 ] && [ "$HEADLESS" -eq 0 ] && command -v ffmpeg &>/dev/null && [ -n "${DISPLAY:-}" ]; then
  echo "      starting screen recording -> ${RUN_DIR}/recording.mp4"
  # Short sleep so Gazebo window is up before recording starts
  sleep 3
  RES=$(xdpyinfo 2>/dev/null | awk '/dimensions/{print $2; exit}')
  RES="${RES:-1920x1080}"
  ffmpeg -loglevel warning -f x11grab -r 30 -s "$RES" -i "${DISPLAY}.0+0,0" \
    -c:v libx264 -preset fast -crf 23 \
    "${RUN_DIR}/recording.mp4" >>"$GZ_LOG" 2>&1 &
  FFMPEG_PID=$!
  echo "      recording started (${RES}, pid ${FFMPEG_PID})."
else
  [ "$RECORD" -eq 0 ] && echo "      screen recording skipped (--no-record)."
  [ "$HEADLESS" -eq 1 ] && echo "      screen recording skipped (headless mode)."
fi

# --------------------------------------------------------------------------- #
# Wait for SITL GPS fix
# --------------------------------------------------------------------------- #
echo
echo "Waiting for SITL GPS/EKF fix..."
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
            print(f"   SITL READY: GPS lat={msg.lat/1e7:.6f} lon={msg.lon/1e7:.6f}")
            sys.exit(0)
    print("WARN: GPS readiness timed out.", file=sys.stderr)
except Exception as e:
    print(f"WARN: readiness probe failed: {e}", file=sys.stderr)
PY

# --------------------------------------------------------------------------- #
# 4. accuracy_verify (foreground - this drives the flight duration gate)
# --------------------------------------------------------------------------- #
VERIFY_OUT="${RUN_DIR}/detections.csv"
VERIFY_REPORT="${RUN_DIR}/accuracy_report.md"
VERIFY_JSON="${RUN_DIR}/summary.json"

echo
echo "[4/4] Starting accuracy_verify.py (connect udp:127.0.0.1:14551)..."
echo "      Outputs -> $RUN_DIR"

# Launch fly_course if requested (after a brief delay for accuracy_verify to connect)
if [ "$AUTO_FLY" -eq 1 ]; then
  (sleep 5; python3 "${REPO_ROOT}/simulation/fly_course.py" \
    --connect udp:127.0.0.1:14550 --countdown 5 --course "${COURSE}") &
  FLY_PID=$!
  echo "      fly_course.py scheduled (5s delay, then 5s countdown)."
fi

bash -c "source '${ROS_SETUP}'; python3 '${REPO_ROOT}/simulation/accuracy_verify.py' \
  --connect udp:127.0.0.1:14551 \
  --out-dir '${RUN_DIR}' \
  --report-dir '${RUN_DIR}' \
  --summary-json '${VERIFY_JSON}'" \
  2>&1 | tee "${RUN_DIR}/verify.log"

echo
echo "============================================================"
echo " run_${RUN_N} complete. All outputs in:"
echo "   $RUN_DIR"
echo "============================================================"
