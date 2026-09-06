#!/usr/bin/env python3
"""
Close the loop between gps_sim_navtoolbox.slx and ArduPilot SITL.

Subscribes to /fix (sensor_msgs/NavSatFix, published live by the Simulink GPS
noise model against real Gazebo odometry) and injects it into ArduPilot SITL
as a MAVLink GPS_INPUT message.

STATUS (see ../simulink/README.md for full detail): this mechanism is verified
working end to end at the SITL level, as of 2026-09-05. GPS_INPUT messages are
accepted as a healthy fix (the vehicle arms successfully on injected data
alone), AND EKF-level fusion into GLOBAL_POSITION_INT is confirmed: running
`--diag-offset-test` against a freshly-built arducopter SITL (--model +,
GPS1_TYPE=14) injected a 5.00m northward ramp and GLOBAL_POSITION_INT tracked
5.01m north / 0.00m east -- a clean 100% match. Earlier testing had reported
this as unconfirmed, but that test never actually moved the vehicle (or the
injected fix) -- it armed a stationary Iris on the ground and watched
GLOBAL_POSITION_INT sit at home, which is exactly what you'd see whether
fusion worked or not; it had no power to distinguish the two. The
previously-recorded theory that --model JSON's FDM ground-truth channel was
silently overriding the GPS-driven estimate doesn't hold up against the
ArduPilot source (AP_GPS_MAV, the backend selected by GPS1_TYPE=14, reads only
from injected GPS_INPUT messages, and EK3_SRC1_POSXY defaults to GPS) and is
now further contradicted by direct measurement. Retired.

Still open: this confirms the GPS_INPUT -> EKF3 mechanism in isolation, not
yet the full Gazebo odometry -> Simulink noise -> /fix -> EKF pipeline with
the vehicle actually flying a course.

Use `--diag-offset-test` (below) to re-check this mechanism any time the
GPS_INPUT path changes: it drives a slow, deliberate position ramp straight
into SITL and checks whether GLOBAL_POSITION_INT follows it. That's a real
test regardless of whether Gazebo, the ROS 2 bridge, or Simulink are even
running.

Requires SITL configured with GPS1_TYPE=14 (MAV) -- note the "1", the older
"GPS_TYPE" name only exists as a load-time conversion alias for old saved
parameter files, not for live PARAM_SET over MAVLink. Preferred way to set
this: bake it in at boot via --add-param-file=gps_mav_params.parm (see the
sibling file in this folder) rather than a live PARAM_SET/reboot -- rebooting
SITL alone (without restarting Gazebo too) desyncs the ardupilot_gazebo
lockstep loop and silently stalls odometry entirely. If GPS1_TYPE is already
set at boot, pass --skip-reboot.

Usage (inside the sim container, with Simulink already publishing /fix,
and SITL launched with --add-param-file=gps_mav_params.parm):
    python3 simulink/fix_to_gps_input.py --connect tcp:127.0.0.1:5760 --skip-reboot

Usage (diagnostic -- SITL only, no Gazebo/ROS2/Simulink needed):
    python3 simulink/fix_to_gps_input.py --connect tcp:127.0.0.1:5760 \\
        --skip-reboot --diag-offset-test
"""

from __future__ import annotations

import argparse
import datetime
import math
import time

from pymavlink import mavutil

try:
    # Only needed for the /fix-relay mode. The --diag-offset-test diagnostic
    # is pure MAVLink and must work without ROS 2 installed at all.
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix, NavSatStatus
except ImportError:
    rclpy = None
    Node = object

GPS_TYPE_MAV = 14

# GPS_INPUT ignore_flags bits (skip fields we don't have trustworthy values for)
IGNORE_VEL_HORIZ = 8
IGNORE_VEL_VERT = 16
IGNORE_SPEED_ACCURACY = 32
NO_VELOCITY_DATA = IGNORE_VEL_HORIZ | IGNORE_VEL_VERT | IGNORE_SPEED_ACCURACY

# Matches the <spherical_coordinates> datum baked into every course world
# (simulation/gazebo/worlds/*.sdf) and the DATUM in build_gps_model.m.
DEFAULT_DATUM_LAT = -35.363262
DEFAULT_DATUM_LON = 149.165237

METERS_PER_DEG_LAT = 111320.0


def gps_time_now() -> tuple[int, int]:
    """Return (week, time_of_week_ms) since the GPS epoch (1980-01-06)."""
    gps_epoch = datetime.datetime(1980, 1, 6)
    now = datetime.datetime.utcnow()
    delta = now - gps_epoch
    week = int(delta.days / 7)
    tow_ms = int((delta.total_seconds() - week * 604800) * 1000)
    return week, tow_ms


def ensure_gps_type_mav(m, force_reboot: bool) -> None:
    """Set GPS_TYPE=14 (MAV), optionally rebooting SITL so it takes effect.

    Fire-and-forget param_set (matches fly_course.py's pattern) -- SITL's
    parameter read-back protocol isn't reliably answering requests from a
    client that isn't itself streaming heartbeats, so we don't try to verify
    the current value first.
    """
    print("[fix_to_gps_input] Setting GPS1_TYPE=14 (MAV)...")
    m.mav.param_set_send(
        m.target_system, m.target_component, b"GPS1_TYPE",
        float(GPS_TYPE_MAV), mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    time.sleep(1)
    if not force_reboot:
        return
    print("[fix_to_gps_input] Rebooting SITL so GPS_TYPE takes effect...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
        1, 0, 0, 0, 0, 0, 0,
    )
    m.close()
    print("[fix_to_gps_input] Waiting for SITL to come back up...")
    time.sleep(10)


def run_diag_offset_test(
    connect_str: str,
    gps_id: int,
    skip_reboot: bool,
    duration_s: float,
    offset_m: float,
    rate_hz: float,
    start_lat: float,
    start_lon: float,
    start_alt: float,
) -> int:
    """Definitively answer "does GPS_INPUT actually reach the EKF?" without
    Gazebo, ROS 2, or Simulink -- just SITL.

    Injects a slow, deliberate northward position ramp (small enough to stay
    well inside EKF3's default glitch/innovation gates) *with matching
    velocity* (sidesteps the separate question of whether the ignore_flags
    velocity path is trustworthy -- see module docstring) and polls
    GLOBAL_POSITION_INT to see whether ArduPilot's own believed position
    follows it. A vehicle sitting still with a frozen readout proves nothing
    either way; a *moving* injected fix that the EKF does or doesn't follow
    is an actual answer.
    """
    print(f"[diag] Connecting to SITL at {connect_str}...")
    m = mavutil.mavlink_connection(connect_str, source_system=1, source_component=195)
    m.wait_heartbeat(timeout=15)
    print("[diag] Heartbeat received")

    ensure_gps_type_mav(m, force_reboot=not skip_reboot)
    if not skip_reboot:
        m = mavutil.mavlink_connection(connect_str, source_system=1, source_component=195)
        m.wait_heartbeat(timeout=20)

    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 4, 1,
    )

    def send_heartbeat():
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def inject(north_m: float, vn: float):
        dlat = north_m / METERS_PER_DEG_LAT
        week, tow_ms = gps_time_now()
        m.mav.gps_input_send(
            int(time.time() * 1e6), gps_id, 0,  # ignore_flags=0: velocity IS trustworthy here
            tow_ms, week, 3,  # fix_type=3 (3D fix)
            int((start_lat + dlat) * 1e7), int(start_lon * 1e7), float(start_alt),
            1.0, 1.0,          # hdop, vdop
            vn, 0.0, 0.0,      # vn, ve, vd
            0.2,               # speed_accuracy
            1.0, 2.0,          # h_acc, v_acc
            10,                # satellites_visible
        )

    def poll_position():
        last = None
        deadline = time.time() + 2.0
        while time.time() < deadline:
            msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
            if msg:
                last = msg
        return last

    # Settle at the start position for 3s and warm up the heartbeat/GPS stream
    # before touching anything, then record the true baseline.
    print(f"[diag] Warming up at start position ({start_lat:.7f}, {start_lon:.7f})...")
    t_end = time.time() + 3.0
    next_hb = 0.0
    while time.time() < t_end:
        if time.time() >= next_hb:
            send_heartbeat()
            next_hb = time.time() + 1.0
        inject(0.0, 0.0)
        time.sleep(1.0 / rate_hz)

    baseline = poll_position()
    if baseline is None:
        print("[diag] FAIL: no GLOBAL_POSITION_INT received at all -- GPS/EKF isn't even "
              "reporting a position. Check GPS1_TYPE=14 took effect and the vehicle isn't "
              "still waiting on a first fix.")
        return 1
    base_lat, base_lon = baseline.lat / 1e7, baseline.lon / 1e7
    print(f"[diag] Baseline EKF position: lat={base_lat:.7f} lon={base_lon:.7f}")

    print(f"[diag] Ramping a {offset_m:.1f}m northward offset over {duration_s:.1f}s "
          f"(with matching velocity)...")
    vn = offset_m / duration_s
    steps = int(duration_s * rate_hz)
    t_start = time.time()
    for i in range(steps + 1):
        now = time.time()
        if now >= next_hb:
            send_heartbeat()
            next_hb = now + 1.0
        elapsed = now - t_start
        north_m = min(offset_m, vn * elapsed)
        inject(north_m, vn if north_m < offset_m else 0.0)
        m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)  # drain, avoid buffer buildup
        time.sleep(1.0 / rate_hz)

    # Hold at the final offset briefly so the EKF has time to converge before
    # we read it back.
    t_end = time.time() + 2.0
    while time.time() < t_end:
        if time.time() >= next_hb:
            send_heartbeat()
            next_hb = time.time() + 1.0
        inject(offset_m, 0.0)
        time.sleep(1.0 / rate_hz)

    final = poll_position()
    if final is None:
        print("[diag] FAIL: GLOBAL_POSITION_INT stopped arriving during the ramp.")
        return 1
    final_lat, final_lon = final.lat / 1e7, final.lon / 1e7
    actual_north_m = (final_lat - base_lat) * METERS_PER_DEG_LAT
    actual_east_m = (final_lon - base_lon) * METERS_PER_DEG_LAT * math.cos(math.radians(base_lat))

    print(f"[diag] Final EKF position:    lat={final_lat:.7f} lon={final_lon:.7f}")
    print(f"[diag] Injected offset: {offset_m:.2f}m north")
    print(f"[diag] EKF-believed displacement: {actual_north_m:.2f}m north, {actual_east_m:.2f}m east")

    frac = actual_north_m / offset_m if offset_m else 0.0
    if frac > 0.5:
        print(f"[diag] PASS: EKF tracked {frac*100:.0f}% of the injected offset -- "
              f"GPS_INPUT is reaching GLOBAL_POSITION_INT.")
        return 0
    elif frac > 0.05:
        print(f"[diag] AMBIGUOUS: EKF tracked only {frac*100:.0f}% of the injected offset. "
              f"Fusion may be happening but heavily damped/gated -- check EK3_POS_I_GATE, "
              f"EK3_GLITCH_RAD, EK3_POSNE_M_NSE, or re-run with a longer --diag-duration.")
        return 2
    else:
        print(f"[diag] FAIL: EKF did not move (tracked {frac*100:.0f}% of the injected offset) -- "
              f"GPS_INPUT is not reaching the position estimate. Check GPS1_TYPE=14 is actually "
              f"set (not just GPS_TYPE), EK3_SRC1_POSXY is GPS (default), and that fix_type>=3 "
              f"is being accepted (watch for GPS glitch / pre-arm messages in STATUSTEXT).")
        return 1


class FixToGpsInput(Node):
    def __init__(self, connect_str: str, gps_id: int, skip_reboot: bool = False, try_arm: bool = False):
        super().__init__("fix_to_gps_input")
        self._gps_id = gps_id

        print(f"[fix_to_gps_input] Connecting to SITL at {connect_str}...")
        m = mavutil.mavlink_connection(connect_str, source_system=1, source_component=195)
        m.wait_heartbeat(timeout=15)
        print("[fix_to_gps_input] Heartbeat received")

        ensure_gps_type_mav(m, force_reboot=not skip_reboot)

        if skip_reboot:
            self._mav = m
        else:
            # Reconnect after the reboot
            self._mav = mavutil.mavlink_connection(connect_str, source_system=1, source_component=195)
            self._mav.wait_heartbeat(timeout=20)
        print("[fix_to_gps_input] SITL ready, relaying /fix -> GPS_INPUT")

        # ArduPilot's external-sensor drivers generally expect a connected GCS
        # to be actively present (streaming heartbeats), same as any real GCS.
        self._hb_timer = self.create_timer(1.0, self._send_heartbeat)

        # Request position telemetry back so we can prove the EKF is actually
        # fusing our injected GPS_INPUT, not just accepting it silently.
        self._mav.mav.request_data_stream_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 4, 1,
        )
        self._poll_timer = self.create_timer(0.1, self._poll_position)
        self._last_pos_log = 0.0

        self._count = 0
        self._sub = self.create_subscription(NavSatFix, "/fix", self._on_fix, 10)

        if try_arm:
            self.create_timer(8.0, self._try_arm_once)
            self._armed_attempted = False

    def _try_arm_once(self) -> None:
        if getattr(self, "_armed_attempted", True):
            return
        self._armed_attempted = True
        print("[fix_to_gps_input] Attempting to arm (triggers full pre-arm GPS check)...")
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0,
        )
        deadline = time.time() + 5
        while time.time() < deadline:
            ack = self._mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                print(f"[fix_to_gps_input] Arm COMMAND_ACK result: {ack.result}")
                return
        print("[fix_to_gps_input] No arm ACK received within timeout")
        # Drain any STATUSTEXT messages that explain a pre-arm failure reason
        deadline = time.time() + 2
        while time.time() < deadline:
            st = self._mav.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
            if st:
                print(f"[fix_to_gps_input] STATUSTEXT: {st.text}")

    def _send_heartbeat(self) -> None:
        self._mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )

    def _poll_position(self) -> None:
        msg = self._mav.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg and time.time() - self._last_pos_log > 5.0:
            self._last_pos_log = time.time()
            self.get_logger().info(
                f"ArduPilot EKF believes: lat={msg.lat/1e7:.7f} lon={msg.lon/1e7:.7f} "
                f"alt={msg.alt/1000:.2f}m"
            )

    def _on_fix(self, msg: NavSatFix) -> None:
        status = msg.status.status
        if status == NavSatStatus.STATUS_NO_FIX:
            fix_type = 0
        elif status == NavSatStatus.STATUS_FIX:
            fix_type = 3
        elif status == NavSatStatus.STATUS_SBAS_FIX:
            fix_type = 4  # DGPS
        elif status == NavSatStatus.STATUS_GBAS_FIX:
            fix_type = 5  # RTK float
        else:
            fix_type = 3

        cov = msg.position_covariance
        h_acc = cov[0] ** 0.5 if cov[0] > 0 else 1.0
        v_acc = cov[8] ** 0.5 if cov[8] > 0 else 3.0

        week, tow_ms = gps_time_now()

        self._mav.mav.gps_input_send(
            int(time.time() * 1e6),
            self._gps_id,
            NO_VELOCITY_DATA,
            tow_ms,
            week,
            fix_type,
            int(msg.latitude * 1e7),
            int(msg.longitude * 1e7),
            float(msg.altitude),
            1.0,   # hdop
            1.0,   # vdop
            0.0, 0.0, 0.0,  # vn, ve, vd (ignored, see NO_VELOCITY_DATA)
            0.5,   # speed_accuracy (ignored)
            h_acc,
            v_acc,
            10,    # satellites_visible
        )
        self._count += 1
        if self._count % 25 == 0:
            self.get_logger().info(
                f"Injected {self._count} GPS_INPUT messages "
                f"(fix_type={fix_type}, h_acc={h_acc:.3f}m)"
            )


def main() -> None:
    ap = argparse.ArgumentParser(description="Inject Simulink /fix into ArduPilot SITL as GPS_INPUT")
    ap.add_argument("--connect", default="tcp:127.0.0.1:5760")
    ap.add_argument("--gps-id", type=int, default=0)
    ap.add_argument("--skip-reboot", action="store_true",
                     help="Skip the GPS_TYPE reboot cycle (use when it's already set -- "
                          "rebooting SITL alone stalls the Gazebo lockstep loop, so only "
                          "reboot right after a fresh Gazebo+SITL restart, never standalone)")
    ap.add_argument("--try-arm", action="store_true",
                     help="Attempt to arm once, 8s after startup, to trigger ArduCopter's "
                          "full pre-arm GPS check (diagnostic only)")
    ap.add_argument("--diag-offset-test", action="store_true",
                     help="Skip the /fix relay entirely. Instead inject a synthetic "
                          "position ramp and check whether GLOBAL_POSITION_INT follows "
                          "it, to definitively answer whether GPS_INPUT is reaching the "
                          "EKF. Needs only SITL running -- no Gazebo/ROS2/Simulink.")
    ap.add_argument("--diag-duration", type=float, default=15.0,
                     help="Ramp duration in seconds (diag mode only)")
    ap.add_argument("--diag-offset-m", type=float, default=5.0,
                     help="Total northward offset in meters to ramp to (diag mode only)")
    ap.add_argument("--diag-rate-hz", type=float, default=5.0,
                     help="Injection rate in Hz, matches the Simulink model's /fix rate (diag mode only)")
    ap.add_argument("--diag-start-lat", type=float, default=DEFAULT_DATUM_LAT)
    ap.add_argument("--diag-start-lon", type=float, default=DEFAULT_DATUM_LON)
    ap.add_argument("--diag-start-alt", type=float, default=0.0)
    args, _ = ap.parse_known_args()

    if args.diag_offset_test:
        import sys
        sys.exit(run_diag_offset_test(
            args.connect, args.gps_id, args.skip_reboot,
            args.diag_duration, args.diag_offset_m, args.diag_rate_hz,
            args.diag_start_lat, args.diag_start_lon, args.diag_start_alt,
        ))

    if rclpy is None:
        raise SystemExit(
            "rclpy/sensor_msgs not available -- the /fix relay mode needs ROS 2. "
            "Use --diag-offset-test if you only want to test GPS_INPUT -> EKF fusion."
        )

    rclpy.init()
    node = FixToGpsInput(args.connect, args.gps_id, skip_reboot=args.skip_reboot, try_arm=args.try_arm)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
