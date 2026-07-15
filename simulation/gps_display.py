#!/usr/bin/env python3
"""Live GPS + flight status display for the 4-window visual mode.

Connects to udp:127.0.0.1:14552 and overwrites a compact status panel once
per second. Designed to run in its own xterm window alongside Gazebo, SITL,
and the camera detector.
"""
import math
import sys
import time

from pymavlink import mavutil


HEADER = (
    "+----------------------------------------------------------+\n"
    "|       RobotX UAV  --  Live GPS & Flight Status          |\n"
    "+----------------------------------------------------------+"
)
N_ROWS = 7  # status rows printed each cycle (must match rows in _print_state)


def _print_state(state: dict, first: bool) -> None:
    if not first:
        sys.stdout.write(f"\033[{N_ROWS}A")  # move cursor up N rows to overwrite

    spd_h = math.hypot(state["vx"], state["vy"])
    armed = "ARMED   " if state["armed"] else "disarmed"
    ts = time.strftime("%H:%M:%S")

    lines = [
        f"  [{ts}]",
        f"  Status  : {armed:<10}  Mode: {state['mode']:<12}",
        f"  Lat     : {state['lat']:+.7f}",
        f"  Lon     : {state['lon']:+.7f}",
        f"  Alt AGL : {state['rel_alt']:6.1f} m",
        f"  Speed   : {spd_h:.2f} m/s horiz   {state['vz']:+.2f} m/s vert",
        "",
    ]
    for line in lines:
        print(f"\r\033[K{line}")
    sys.stdout.flush()


def main() -> None:
    print(HEADER)
    print("  Connecting to SITL on udp:127.0.0.1:14552 ...")
    sys.stdout.flush()

    try:
        m = mavutil.mavlink_connection("udp:127.0.0.1:14552")
    except Exception as exc:
        print(f"  ERROR: {exc}", file=sys.stderr)
        sys.exit(1)

    m.wait_heartbeat()
    print("  Connected. Waiting for GPS fix...\n")
    # Print blank placeholder rows so the first overwrite has lines to replace.
    for _ in range(N_ROWS):
        print()

    state: dict = dict(lat=0.0, lon=0.0, rel_alt=0.0,
                       vx=0.0, vy=0.0, vz=0.0, mode="INIT", armed=False)
    last_print = 0.0
    first = True

    while True:
        msg = m.recv_match(
            type=["GLOBAL_POSITION_INT", "HEARTBEAT"],
            blocking=True, timeout=0.5)

        if msg is not None:
            t = msg.get_type()
            if t == "GLOBAL_POSITION_INT":
                state.update(
                    lat=msg.lat / 1e7,
                    lon=msg.lon / 1e7,
                    rel_alt=msg.relative_alt / 1000.0,
                    vx=msg.vx / 100.0,
                    vy=msg.vy / 100.0,
                    vz=msg.vz / 100.0,
                )
            elif t == "HEARTBEAT":
                state["armed"] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                for name, num in m.mode_mapping().items():
                    if num == msg.custom_mode:
                        state["mode"] = name
                        break

        now = time.time()
        if now - last_print >= 1.0:
            _print_state(state, first)
            last_print = now
            first = False


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n  GPS display stopped.")
