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
#   every few seconds while flying between waypoints.
#
# VISUAL MODE (4 windows):
#   bash simulation/run_course.sh --course 1 --visual
#   bash simulation/run_course.sh --course 2 --visual
#   bash simulation/run_course.sh --course 3 --visual
#
#   Opens four separate windows:
#     1. Gazebo 3D view   -- animated VRX ocean + buoys + drone
#     2. SITL console     -- ArduCopter / MAVProxy arm / mode / GPS log
#     3. Camera detector  -- camera_live_feed.py text output + OpenCV window
#     4. GPS coordinates  -- live lat / lon / alt AGL / speed / mode
#
# After EITHER run completes (or Ctrl-C), all outputs are saved to:
#   simulation/sim_tests/run_N/
#     detections.csv       per-frame buoy GPS projections
#     accuracy_report.md   cross-referenced vs ground-truth positions
#     summary.json         machine-readable metrics
#     gz.log               Gazebo (+ image bridge) stdout/stderr
#     sitl.log             ArduPilot SITL + MAVProxy console
#     fly.log              fly_course.py output
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
#
# NOTE: this script deliberately does NOT use `set -e`. It is a long-running
# orchestrator full of background jobs, monitoring loops and `grep` calls (grep
# exits non-zero on no-match), so `set -e` causes surprise exits. Errors that
# matter are checked explicitly. Ctrl-C is handled by a trap that tears the
# whole simulation down cleanly (see kill_all_sim / cleanup below).
set -o pipefail

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
    --course)    COURSE="$2";             shift 2 ;;
    --course=*)  COURSE="${1#--course=}"; shift ;;
    --visual)    VISUAL=1;                shift ;;
    --no-fly)    AUTO_FLY=0;              shift ;;
    --speed)     SPEED="$2";              shift 2 ;;
    --speed=*)   SPEED="${1#--speed=}";   shift ;;
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
     WORLD_NAME="robotx_uav_course"
     TOTAL_WP=4 ;;
  2) WORLD="${REPO_ROOT}/simulation/gazebo/worlds/course_2_search_field.sdf"
     COURSE_NAME="Course 2: Open Water Survey (Lawnmower)"
     WORLD_NAME="course_2_search_field"
     TOTAL_WP=6 ;;
  3) WORLD="${REPO_ROOT}/simulation/gazebo/worlds/course_3_dogleg.sdf"
     COURSE_NAME="Course 3: L-Shaped Dogleg"
     WORLD_NAME="course_3_dogleg"
     TOTAL_WP=6 ;;
  *) echo "ERROR: --course must be 1, 2, or 3" >&2; exit 2 ;;
esac
# /world/<name>/stats is published by the Gazebo physics server itself once the
# world has finished loading (all plugins, incl. ArduPilotPlugin's FDM port,
# are up). A stale image_bridge cannot fake it -- unlike /drone/camera.
GZ_READY_TOPIC="/world/${WORLD_NAME}/stats"

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

GZ_LOG="${RUN_DIR}/gz.log"
SITL_LOG="${RUN_DIR}/sitl.log"
FLY_LOG="${RUN_DIR}/fly.log"
GPS_READY_FILE="${RUN_DIR}/.gps_ready"

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
# Helpers
# --------------------------------------------------------------------------- #
ts()   { date '+%H:%M:%S'; }
pmsg() { echo "[$(ts)] $*"; }

# Comprehensive SIGKILL net for every process this sim stack can spawn. Used in
# BOTH pre-flight (clear last run's leftovers) and teardown (don't leave any).
#
# This is the safety net that makes runs independent of how the previous one
# ended -- clean exit, Ctrl-C, crash, or kill. It matches by command-line
# substring because children get orphaned (reparented to PID 1) when their
# launcher dies, so tracked PIDs alone are not enough.
#
# IMPORTANT pattern notes:
#  * The image bridge runs as ".../ros_gz_image/image_bridge /drone/camera" --
#    the binary is image_bridge (SLASH before it, not a space). Match the bare
#    word "image_bridge" or it will never be killed (this was the original bug
#    that left orphaned bridges faking /drone/camera readiness forever).
#  * run_course.sh's own command line is "bash .../run_course.sh --course N",
#    which contains none of these patterns, so we never SIGKILL ourselves.
kill_all_sim() {
  pkill -9 -f "arducopter"                  2>/dev/null
  pkill -9 -f "sim_vehicle.py"              2>/dev/null
  pkill -9 -f "mavproxy"                    2>/dev/null
  pkill -9 -f "gz sim"                      2>/dev/null
  pkill -9 -f "simulation/gazebo/worlds"    2>/dev/null
  pkill -9 -f "image_bridge"               2>/dev/null
  pkill -9 -f "accuracy_verify.py"         2>/dev/null
  pkill -9 -f "camera_live_feed.py"        2>/dev/null
  pkill -9 -f "gps_display.py"             2>/dev/null
  pkill -9 -f "fly_course.py"              2>/dev/null
  return 0
}

# --------------------------------------------------------------------------- #
# PIDs to clean up on exit
# --------------------------------------------------------------------------- #
GZ_PID=""; SITL_PID=""; BRIDGE_PID=""
FLY_PID=""; VERIFY_PID=""
SITL_XT_PID=""; CAM_XT_PID=""; GPS_XT_PID=""
CLEANED=0

cleanup() {
  [ "$CLEANED" = 1 ] && return 0
  CLEANED=1
  echo
  pmsg "Tearing down run_${RUN_N}..."

  # 1. Give accuracy_verify a chance to flush its report. It is launched with
  #    `exec` so VERIFY_PID is the python itself -- SIGTERM reaches its handler,
  #    which writes accuracy_report.md + summary.json before exiting.
  if [ -n "$VERIFY_PID" ] && kill -0 "$VERIFY_PID" 2>/dev/null; then
    pmsg "      Asking accuracy_verify to write its report..."
    kill -TERM "$VERIFY_PID" 2>/dev/null || true
    for _ in $(seq 1 12); do
      kill -0 "$VERIFY_PID" 2>/dev/null || break
      sleep 0.5
    done
  fi

  # 2. Terminate tracked PIDs (xterms, fly, gz, sitl, bridge).
  for pid_var in GPS_XT_PID CAM_XT_PID SITL_XT_PID FLY_PID BRIDGE_PID SITL_PID GZ_PID; do
    eval pid="\${${pid_var}:-}"
    [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
  done

  # 3. SIGKILL net for anything orphaned (the real guarantee of a clean slate).
  kill_all_sim
  rm -f "$GPS_READY_FILE" 2>/dev/null || true

  # Stamp run metadata into summary.json if it was written.
  local sjson="${RUN_DIR}/summary.json"
  if [ -f "$sjson" ]; then
    python3 - "$sjson" "$RUN_N" "$RUN_DIR" "$COURSE" "$COURSE_NAME" "$MODE_STR" <<'PY' || true
import json, sys, os
path, run_n, run_dir, course_n, course_name, mode = \
    sys.argv[1], int(sys.argv[2]), sys.argv[3], int(sys.argv[4]), sys.argv[5], sys.argv[6]
with open(path) as f:
    d = json.load(f)
d.update(run=run_n, course=course_n, course_name=course_name, mode=mode)
for fname in os.listdir(run_dir):
    if fname.endswith(".mp4"):
        d.setdefault("files", {})["recording"] = fname
with open(path, "w") as f:
    json.dump(d, f, indent=2)
print(f"[INFO] summary.json updated: run={run_n}, mode={mode}")
PY
  fi

  # Generate the detection map.
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

# On Ctrl-C / TERM: tear down and exit (do NOT let the script resume after the
# handler). The EXIT trap also calls cleanup, but the CLEANED guard makes it a
# no-op the second time.
on_signal() { echo; pmsg "Signal received -- stopping."; cleanup; exit 130; }
trap on_signal INT TERM
trap cleanup EXIT

# --------------------------------------------------------------------------- #
# Pre-flight: clear every leftover from previous runs, then confirm the gz bus
# is actually clean before starting a fresh world.
# --------------------------------------------------------------------------- #
pmsg "Pre-flight: clearing stale simulation processes..."
kill_all_sim

pmsg "      Waiting for the gz bus to clear (stale /drone/camera to vanish)..."
cleared=0
for _ in $(seq 1 15); do
  if ! gz topic -l 2>/dev/null | grep -q "/drone/camera"; then cleared=1; break; fi
  sleep 1
done
if [ "$cleared" -eq 1 ]; then
  pmsg "      Bus clean."
else
  pmsg "      WARN: /drone/camera still present; a bridge may be stuck. Continuing."
fi
sleep 1   # let UDP sockets fully release

# --------------------------------------------------------------------------- #
# 1. Gazebo
# --------------------------------------------------------------------------- #
if [ "$VISUAL" -eq 1 ]; then
  pmsg "[1/4] Starting Gazebo (GUI window)..."
  gz sim -v3 -r "$WORLD" >"$GZ_LOG" 2>&1 &
else
  pmsg "[1/4] Starting Gazebo (headless server)..."
  gz sim -v3 -s -r "$WORLD" >"$GZ_LOG" 2>&1 &
fi
GZ_PID=$!

pmsg "      Waiting for Gazebo physics server (${GZ_READY_TOPIC}; world load ~30-90 s)..."
GZ_START=$(date +%s)
ready=0
for _ in $(seq 1 150); do
  if gz topic -l 2>/dev/null | grep -qx "${GZ_READY_TOPIC}"; then ready=1; break; fi
  if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "ERROR: Gazebo exited early (see $GZ_LOG)." >&2; exit 1
  fi
  sleep 1
done
GZ_ELAPSED=$(( $(date +%s) - GZ_START ))
if [ "$ready" -eq 1 ]; then
  pmsg "      Gazebo physics server up in ${GZ_ELAPSED}s."
else
  pmsg "      WARN: ${GZ_READY_TOPIC} not seen after ${GZ_ELAPSED}s; continuing anyway."
fi

# Confirm the nadir camera sensor is actually rendering before we wire up the
# bridge + detector (this only appears now that the bus is clean).
pmsg "      Waiting for /drone/camera sensor..."
for _ in $(seq 1 30); do
  gz topic -l 2>/dev/null | grep -qx "/drone/camera" && break
  sleep 1
done

# --------------------------------------------------------------------------- #
# 2. ArduPilot SITL  (own log so the SITL window shows only SITL/MAVProxy text)
# --------------------------------------------------------------------------- #
pmsg "[2/4] Starting ArduPilot SITL..."
cd "$REPO_ROOT"
# --mavproxy-args=--daemon is CRITICAL here. sim_vehicle.py launches MAVProxy,
# whose interactive console reads stdin. Because this whole command is a
# background job in a non-interactive script (job control off), bash redirects
# its stdin from /dev/null -- which delivers instant EOF. Without --daemon,
# MAVProxy treats that EOF as "quit", unloads every module and exits the moment
# it reaches the MAV> prompt; sim_vehicle then declares "MAVProxy exited" and
# tears the SITL stack down, so nothing ever heartbeats on 14550-14552 and the
# GPS-fix probe aborts the run. --daemon runs MAVProxy with no interactive
# shell (it never reads stdin) while still forwarding to every --out target.
env -u DISPLAY python3 "$SIM_VEHICLE" \
  -v ArduCopter -f gazebo-iris --model JSON --no-rebuild -I0 \
  --mavproxy-args="--daemon" \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14552 \
  >"$SITL_LOG" 2>&1 &
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
# 4. Visual mode: open the display windows
# --------------------------------------------------------------------------- #
XTERM=(xterm -fa "Monospace" -fs 11)
if [ "$VISUAL" -eq 1 ]; then
  pmsg "[4/4] Opening display windows..."

  # Window 2: SITL + MAVProxy console (tail its dedicated log).
  "${XTERM[@]}" -T "SITL + MAVProxy console" -geometry 110x32+10+20 \
    -e bash -c "echo '=== ArduPilot SITL + MAVProxy ==='; tail -n 200 -f '${SITL_LOG}'" &
  SITL_XT_PID=$!
  pmsg "      Window 2: SITL console"

  # Window 3: camera_live_feed.py (opens its own OpenCV detection window).
  CAM_CMD="cd '${REPO_ROOT}' && source '${ROS_SETUP}' && \
    echo '=== camera_live_feed.py  [nadir camera] ===' && \
    python3 camera_live_feed.py \
      --ros-topic /drone/camera \
      --no-undistort \
      --altitude-m '${ALTITUDE_M}' 2>&1 | tee '${RUN_DIR}/camera.log'"
  "${XTERM[@]}" -T "Camera Detector" -geometry 110x28+750+20 \
    -e bash -lc "$CAM_CMD" &
  CAM_XT_PID=$!
  pmsg "      Window 3: camera_live_feed.py (+ OpenCV window)"
  pmsg "      Window 4: GPS display opens after GPS fix..."
fi

# --------------------------------------------------------------------------- #
# Wait for SITL GPS/EKF fix. The probe binds udp:14552, so the GPS-display
# window (which also binds 14552) must open only AFTER this exits.
# --------------------------------------------------------------------------- #
echo
pmsg "Waiting for SITL GPS/EKF fix (this takes 30-90 s)..."
rm -f "$GPS_READY_FILE" 2>/dev/null || true
python3 - "$GPS_READY_FILE" <<'PY'
import sys, time
from pymavlink import mavutil
ready_file = sys.argv[1]
try:
    m = mavutil.mavlink_connection("udp:127.0.0.1:14552")
    if m.wait_heartbeat(timeout=90) is None:
        print("ERROR: no SITL heartbeat in 90 s -- SITL likely never connected to",
              "the Gazebo FDM (UDP 9002). Check sitl.log / the SITL window.",
              file=sys.stderr)
        sys.exit(1)
    deadline = time.time() + 120
    while time.time() < deadline:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
        if msg and msg.lat != 0:
            print(f"   SITL READY: GPS lat={msg.lat/1e7:.6f}  lon={msg.lon/1e7:.6f}")
            open(ready_file, "w").close()
            sys.exit(0)
    print("ERROR: heartbeat OK but GPS position never arrived in 120 s.",
          file=sys.stderr)
    sys.exit(1)
except SystemExit:
    raise
except Exception as e:
    print(f"ERROR: readiness probe failed: {e}", file=sys.stderr)
    sys.exit(1)
PY
if [ ! -f "$GPS_READY_FILE" ]; then
  pmsg "ABORT: SITL never got a GPS fix -- nothing to fly."
  pmsg "       See ${SITL_LOG} (or the SITL window) for the ArduCopter error."
  exit 1
fi
rm -f "$GPS_READY_FILE" 2>/dev/null || true

# GPS display window (visual): now safe to bind 14552.
if [ "$VISUAL" -eq 1 ]; then
  "${XTERM[@]}" -T "Live GPS Coords" -geometry 70x16+10+560 \
    -e bash -c "python3 '${REPO_ROOT}/simulation/gps_display.py' 2>&1 | tee '${RUN_DIR}/gps.log'" &
  GPS_XT_PID=$!
  pmsg "      Window 4: GPS display (lat / lon / alt / speed)"
fi

# --------------------------------------------------------------------------- #
# accuracy_verify.py -- background detection logger. `exec` makes VERIFY_PID the
# python itself, so the cleanup SIGTERM reaches its report-writing handler.
# --------------------------------------------------------------------------- #
echo
pmsg "Starting accuracy_verify.py (detection logger -> ${RUN_DIR})..."
bash -c "source '${ROS_SETUP}'; exec python3 '${REPO_ROOT}/simulation/accuracy_verify.py' \
  --world '${WORLD}' \
  --connect udp:127.0.0.1:14551 \
  --out-dir '${RUN_DIR}' \
  --report-dir '${RUN_DIR}' \
  --summary-json '${RUN_DIR}/summary.json'" \
  >"${RUN_DIR}/verify.log" 2>&1 &
VERIFY_PID=$!

# --------------------------------------------------------------------------- #
# Headless progress monitor: tail fly.log and print a percentage + status.
# fly_course.py markers we key off (note: --countdown 0 means it never prints
# "FLIGHT START", so we detect flight state from these instead):
#   "GPS/position OK"          -> got GPS fix
#   "ARMED"                    -> armed
#   "-> <label> (N=..,E=..)"   -> transiting to a waypoint
#   "reached <label> -- held"  -> a waypoint is complete (counts toward %)
# --------------------------------------------------------------------------- #
monitor_headless() {
  pmsg "=== FLIGHT IN PROGRESS | ${COURSE_NAME} | ${TOTAL_WP} waypoints ==="
  echo
  local last_reached=0 last_status=0 reached pct now_ts now_str label current

  while kill -0 "$FLY_PID" 2>/dev/null; do
    now_ts=$(date +%s); now_str=$(ts)

    reached=$(grep -c "reached .* -- held" "$FLY_LOG" 2>/dev/null) || reached=0
    [[ "$reached" =~ ^[0-9]+$ ]] || reached=0
    pct=$(( reached * 100 / TOTAL_WP ))

    # Milestone line for each newly completed waypoint.
    while [ "$last_reached" -lt "$reached" ]; do
      last_reached=$(( last_reached + 1 ))
      label=$(grep "reached .* -- held" "$FLY_LOG" 2>/dev/null \
              | sed -n "${last_reached}p" | sed 's/.*reached //; s/ -- held.*//')
      echo "[${now_str}] $(printf '%3d' "$pct")% | waypoint ${last_reached}/${TOTAL_WP} reached: ${label}"
      last_status=0   # force an immediate status line after a milestone
    done

    # Status line every ~4 s between milestones.
    if [ $(( now_ts - last_status )) -ge 4 ]; then
      last_status=$now_ts
      if grep -qE "(reached .* -- held|-> .* \(N=)" "$FLY_LOG" 2>/dev/null; then
        current=$(grep -e "-> " "$FLY_LOG" 2>/dev/null | tail -1 | sed 's/.*-> //; s/ (N=.*//')
        echo "[${now_str}] $(printf '%3d' "$pct")% | ${last_reached}/${TOTAL_WP} done | flying to: ${current:-?}"
      elif grep -q "ARMED" "$FLY_LOG" 2>/dev/null; then
        echo "[${now_str}]   0% | armed -- climbing to altitude..."
      elif grep -q "GPS/position OK" "$FLY_LOG" 2>/dev/null; then
        echo "[${now_str}]   0% | GPS fix -- arming..."
      else
        echo "[${now_str}]   0% | waiting for GPS / arming..."
      fi
    fi
    sleep 2
  done
  echo
  pmsg "100% | flight finished."
}

# --------------------------------------------------------------------------- #
# Auto-fly (or hand off to manual flight)
# --------------------------------------------------------------------------- #
if [ "$AUTO_FLY" -eq 0 ]; then
  echo
  pmsg "Sim running. Fly manually when ready:"
  pmsg "  python3 simulation/fly_course.py --course ${COURSE} --connect udp:127.0.0.1:14550"
  pmsg "Ctrl-C to stop and save outputs."
  wait "$GZ_PID" 2>/dev/null || true
  exit 0
fi

if [ "$VISUAL" -eq 1 ]; then
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
    --connect udp:127.0.0.1:14550 --course "${COURSE}" \
    --speed "${SPEED}" --countdown 0 2>&1 | tee "$FLY_LOG"
else
  echo
  pmsg "Auto-fly starting in 5 s..."
  sleep 5
  python3 "${REPO_ROOT}/simulation/fly_course.py" \
    --connect udp:127.0.0.1:14550 --course "${COURSE}" \
    --speed "${SPEED}" --countdown 0 >"$FLY_LOG" 2>&1 &
  FLY_PID=$!
  echo
  monitor_headless
  wait "$FLY_PID" 2>/dev/null || true
fi

# Flight done: let accuracy_verify flush, then cleanup() (via EXIT) tears down.
pmsg "Flight complete. Finalizing accuracy report..."
sleep 3
# cleanup() (EXIT trap) handles report flush, teardown, map and summary.
exit 0
