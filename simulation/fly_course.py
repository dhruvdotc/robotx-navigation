#!/usr/bin/env python3
"""Scripted cinematic GUIDED flight down the RobotX UAV course (ArduCopter SITL).

Connects over MAVLink (default udp:127.0.0.1:14550, i.e. MAVProxy's GCS out),
arms, takes off to a fixed altitude, then dollies EAST along the channel
centreline, pausing LEVEL over each gate so the nadir camera gets clean frames.
The same flight is what camera_live_feed.py and accuracy_verify.py observe.

Encodes the SITL gotchas learned for this world (see project memory):
  * Arm by polling COMMAND_ACK == ACCEPTED, NOT the heartbeat armed bit; the
    first arm right after GPS fix often returns FAILED while EKF settles -> retry.
  * Always MAV_CMD_NAV_TAKEOFF and confirm its ACK before waiting on altitude.
  * WP_YAW_BEHAVIOR=0 so GUIDED position targets don't auto-yaw the nose (which
    would break the nadir image-right=East mapping).
  * Slow cinematic dolly = stream incremental position setpoints advancing East
    at a fixed m/s (WPNAV_SPEED over MAVLink didn't reliably slow GUIDED).
  * Never reboot the FC in place (breaks gz lockstep).

Gazebo +X = NED East, +Y = North; gates sit at East = 10, 25, 40 (m), Y = 0.
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

# MAV_FRAME_LOCAL_NED position-only mask: ignore vx..az, force, yaw, yaw_rate.
POS_TYPE_MASK = 0b0000111111111000


def log(msg: str) -> None:
    print(f"[FLY {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# Preset waypoints for each course. Each entry is (north_m, east_m, label).
COURSE_WAYPOINTS = {
    1: [
        # Course 1: straight East navigation channel (3 gates + light buoy)
        (0.0,  10.0, "gate 1"),
        (0.0,  25.0, "gate 2"),
        (0.0,  40.0, "gate 3"),
        (0.0,  50.0, "light buoy"),
    ],
    2: [
        # Course 2: lawnmower sweep over 60x30 m open-water survey field.
        # Three E-W strips at N=-15, 0, +15 ensure every scattered buoy is
        # overflown within ~8 m of nadir at 10 m AGL (60-deg FOV).
        (- 15.0,  0.0,  "strip 1 start"),
        (- 15.0, 60.0,  "strip 1 end"),
        (   0.0, 60.0,  "strip 2 start"),
        (   0.0,  0.0,  "strip 2 end"),
        (+ 15.0,  0.0,  "strip 3 start"),
        (+ 15.0, 60.0,  "strip 3 end"),
    ],
    3: [
        # Course 3: L-shaped dogleg -- East then North.
        # Leg 1: fly East along N=0 centreline past gates 1 and 2.
        (0.0,  10.0, "gate 1"),
        (0.0,  25.0, "gate 2"),
        (0.0,  35.0, "corner turn"),
        # Leg 2: fly North along E=35 centreline past gates 3 and 4.
        (15.0, 35.0, "gate 3"),
        (30.0, 35.0, "gate 4"),
        (42.0, 35.0, "light buoy"),
    ],
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--connect", default="udp:127.0.0.1:14550",
                   help="MAVLink endpoint (MAVProxy GCS out by default).")
    p.add_argument("--course", type=int, default=1, choices=[1, 2, 3],
                   help="Course preset: 1=nav channel, 2=search field lawnmower, "
                        "3=L-shaped dogleg. Overrides --gates if set.")
    p.add_argument("--altitude-m", type=float, default=10.0)
    p.add_argument("--speed", type=float, default=1.5, help="Transit speed (m/s).")
    p.add_argument("--gates", default=None,
                   help="Override waypoints: comma-separated 'north:east' pairs, "
                        "e.g. '0:10,0:25'. Overrides --course.")
    p.add_argument("--hover-s", type=float, default=4.0,
                   help="Level hover seconds at each waypoint (for clean nadir frames).")
    p.add_argument("--countdown", type=int, default=10,
                   help="Seconds of countdown before the flight begins (0 to skip).")
    p.add_argument("--rtl", action="store_true", help="RTL at the end instead of LAND.")
    p.add_argument("--arm-timeout", type=float, default=90.0)
    return p.parse_args()


def wait_command_ack(m, command, timeout=5.0):
    """Return the MAV_RESULT for `command`, or None on timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
        if ack and ack.command == command:
            return ack.result
    return None


def set_param(m, name, value, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode(), float(value), ptype)
    log(f"set {name} = {value}")


def set_mode(m, mode_name):
    mode_id = m.mode_mapping()[mode_name]
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)


def wait_gps(m, timeout=120.0):
    log("waiting for GPS fix / EKF position...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
        if msg and msg.lat != 0:
            log(f"GPS/position OK (lat={msg.lat/1e7:.6f}, lon={msg.lon/1e7:.6f}, "
                f"alt={msg.relative_alt/1000:.1f} m)")
            return True
    return False


def arm(m, timeout):
    log("arming (retrying until ACCEPTED; first tries often fail while EKF settles)...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        result = wait_command_ack(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 3.0)
        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            log("ARMED (COMMAND_ACK ACCEPTED).")
            return True
        log(f"arm not accepted yet (result={result}); retrying...")
        time.sleep(2.0)
    return False


def takeoff(m, alt):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    result = wait_command_ack(m, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 5.0)
    if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        log(f"TAKEOFF not accepted (result={result}); aborting.")
        return False
    log(f"takeoff accepted; climbing to {alt:.1f} m...")
    deadline = time.time() + 60
    while time.time() < deadline:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
        if msg:
            rel = msg.relative_alt / 1000.0
            if rel >= alt * 0.95:
                log(f"reached {rel:.2f} m.")
                return True
    return False


def goto(m, north, east, alt, settle=0.0):
    """Send a position setpoint (NED, alt up) and optionally hold it `settle` s."""
    end = time.time() + max(settle, 0.0)
    while True:
        m.mav.set_position_target_local_ned_send(
            0, m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, POS_TYPE_MASK,
            north, east, -alt, 0, 0, 0, 0, 0, 0, 0, 0)
        if time.time() >= end:
            return
        time.sleep(0.1)


def fly_waypoints(m, waypoints, altitude, speed, hover_s):
    """Fly through a list of (north, east, label) waypoints at constant altitude."""
    north, east = 0.0, 0.0
    step = speed * 0.2
    for (tgt_n, tgt_e, label) in waypoints:
        dist = math.hypot(tgt_n - north, tgt_e - east)
        log(f"-> {label} (N={tgt_n:.1f}, E={tgt_e:.1f}), dist={dist:.1f} m")
        # interpolate in small steps for smooth flight
        while math.hypot(tgt_n - north, tgt_e - east) > step:
            dn = tgt_n - north
            de = tgt_e - east
            mag = math.hypot(dn, de)
            north += step * dn / mag
            east  += step * de / mag
            goto(m, north, east, altitude, settle=0.2)
        # snap to target and hover
        north, east = tgt_n, tgt_e
        goto(m, north, east, altitude, settle=hover_s)
        log(f"   reached {label} -- held {hover_s:.1f}s.")


def main() -> int:
    args = parse_args()

    # Resolve waypoints: manual override > course preset
    if args.gates is not None:
        waypoints = []
        for pair in args.gates.split(","):
            pair = pair.strip()
            if ":" in pair:
                n, e = pair.split(":", 1)
                waypoints.append((float(n), float(e), f"N{n} E{e}"))
            else:
                waypoints.append((0.0, float(pair), f"E={pair}"))
    else:
        waypoints = COURSE_WAYPOINTS[args.course]

    log(f"Course {args.course}: {len(waypoints)} waypoints.")

    log(f"connecting to {args.connect} ...")
    m = mavutil.mavlink_connection(args.connect)
    m.wait_heartbeat()
    log(f"heartbeat from system {m.target_system} component {m.target_component}.")

    if args.countdown > 0:
        log(f"FLIGHT STARTS IN {args.countdown}s -- start your screen recording NOW.")
        for s in range(args.countdown, 0, -1):
            print(f"   ... flight in {s:2d}s", flush=True)
            time.sleep(1.0)
        log("=== FLIGHT START ===")

    if not wait_gps(m):
        log("no GPS fix; aborting.")
        return 1

    set_param(m, "WP_YAW_BEHAVIOR", 0)
    time.sleep(0.5)
    set_mode(m, "GUIDED")
    time.sleep(1.0)

    if not arm(m, args.arm_timeout):
        log("could not arm; aborting.")
        return 1
    if not takeoff(m, args.altitude_m):
        log("takeoff failed; aborting.")
        return 1

    fly_waypoints(m, waypoints, args.altitude_m, args.speed, args.hover_s)

    log("course complete.")
    if args.rtl:
        set_mode(m, "RTL"); log("RTL.")
    else:
        set_mode(m, "LAND"); log("LAND.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
