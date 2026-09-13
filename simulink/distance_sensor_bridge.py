#!/usr/bin/env python3
"""
Close the loop between rangefinder_sim.slx and ArduPilot SITL.

Subscribes to /rangefinder (sensor_msgs/Range, published by the Simulink
rangefinder noise model against real Gazebo odometry) and injects it into
ArduPilot SITL as a MAVLink DISTANCE_SENSOR message.

STATUS: UNTESTED end to end (built alongside rangefinder_sim.slx, which is
itself untested -- see that file's header and simulink/README.md). The
message field values and pymavlink call signature below are written against
MAVLink's common.xml DISTANCE_SENSOR definition and cross-checked against
ArduPilot's own handler (~/ardupilot/libraries/AP_RangeFinder/AP_RangeFinder_MAVLink.cpp)
during authoring, but this has not been run live. If distance_sensor_send()
raises a TypeError about argument count/order, check pymavlink's generated
dialect (dialects/v20/common.py in your pymavlink install) for the exact
signature your pymavlink version expects -- MAVLink dialect codegen has
changed field ordering across versions before.

Requires SITL configured with RNGFND1_TYPE=10 (MAVLink) and RNGFND1_ORIENT=25
(facing down) -- bake both in at boot via
--add-param-file=rangefinder_mav_params.parm, same reasoning as
gps_mav_params.parm (rebooting SITL alone desyncs the Gazebo lockstep loop).
If RNGFND1_TYPE is already set at boot, pass --skip-reboot.

Usage (Simulink already publishing /rangefinder, SITL launched with
--add-param-file=rangefinder_mav_params.parm):
    python3 simulink/distance_sensor_bridge.py --connect tcp:127.0.0.1:5760 --skip-reboot
"""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

RNGFND_TYPE_MAV = 10
RNGFND_ORIENT_DOWN = 25  # ROTATION_PITCH_270, must match RNGFND1_ORIENT exactly
MAV_DISTANCE_SENSOR_LASER = 0


def ensure_rngfnd_type_mav(m, force_reboot: bool) -> None:
    """Set RNGFND1_TYPE=10 (MAVLink), optionally rebooting SITL so it takes effect."""
    print("[distance_sensor_bridge] Setting RNGFND1_TYPE=10 (MAVLink)...")
    m.mav.param_set_send(
        m.target_system, m.target_component, b"RNGFND1_TYPE",
        float(RNGFND_TYPE_MAV), mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    time.sleep(1)
    print("[distance_sensor_bridge] Setting RNGFND1_ORIENT=25 (down)...")
    m.mav.param_set_send(
        m.target_system, m.target_component, b"RNGFND1_ORIENT",
        float(RNGFND_ORIENT_DOWN), mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    time.sleep(1)
    if not force_reboot:
        return
    print("[distance_sensor_bridge] Rebooting SITL so RNGFND1_TYPE takes effect...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
        1, 0, 0, 0, 0, 0, 0,
    )
    m.close()
    print("[distance_sensor_bridge] Waiting for SITL to come back up...")
    time.sleep(10)


class RangeToDistanceSensor(Node):
    def __init__(self, connect_str: str, skip_reboot: bool = False):
        super().__init__("distance_sensor_bridge")

        print(f"[distance_sensor_bridge] Connecting to SITL at {connect_str}...")
        m = mavutil.mavlink_connection(connect_str, source_system=1, source_component=196)
        m.wait_heartbeat(timeout=15)
        print("[distance_sensor_bridge] Heartbeat received")

        ensure_rngfnd_type_mav(m, force_reboot=not skip_reboot)

        if skip_reboot:
            self._mav = m
        else:
            self._mav = mavutil.mavlink_connection(connect_str, source_system=1, source_component=196)
            self._mav.wait_heartbeat(timeout=20)
        print("[distance_sensor_bridge] SITL ready, relaying /rangefinder -> DISTANCE_SENSOR")

        # Same reasoning as fix_to_gps_input.py: ArduPilot expects a connected
        # GCS to be actively streaming heartbeats.
        self._hb_timer = self.create_timer(1.0, self._send_heartbeat)

        self._count = 0
        self._sub = self.create_subscription(Range, "/rangefinder", self._on_range, 10)

    def _send_heartbeat(self) -> None:
        self._mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )

    def _on_range(self, msg: Range) -> None:
        current_cm = int(max(msg.range, 0.0) * 100)
        min_cm = int(max(msg.min_range, 0.0) * 100)
        max_cm = int(max(msg.max_range, 0.0) * 100)

        # sensor_msgs/Range has no quality field (see build_rangefinder_model.m's
        # header), so infer it from the model's own dropout convention: a
        # reading pinned at max_range means "no valid return." MAVLink's
        # signal_quality is 0-100 (0=unknown, 1=invalid, 100=perfect) --
        # leaving it at the send() default of 0 ("unknown") was confirmed live
        # to leave ArduPilot's rangefinder marked present-but-unhealthy in
        # SYS_STATUS, even with continuously fresh, in-range readings.
        signal_quality = 1 if current_cm >= max_cm else 95

        self._mav.mav.distance_sensor_send(
            int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
            min_cm,
            max_cm,
            current_cm,
            MAV_DISTANCE_SENSOR_LASER,
            0,                       # id -- must match RNGFND1_ADDR if that's ever set (default 0/any)
            RNGFND_ORIENT_DOWN,      # must match RNGFND1_ORIENT or the reading is silently dropped
            0,                       # covariance (unknown)
            signal_quality=signal_quality,
        )
        self._count += 1
        if self._count % 25 == 0:
            self.get_logger().info(
                f"Injected {self._count} DISTANCE_SENSOR messages "
                f"(current={current_cm}cm)"
            )


def main() -> None:
    ap = argparse.ArgumentParser(description="Inject Simulink /rangefinder into ArduPilot SITL as DISTANCE_SENSOR")
    ap.add_argument("--connect", default="tcp:127.0.0.1:5760")
    ap.add_argument("--skip-reboot", action="store_true",
                     help="Skip the RNGFND1_TYPE reboot cycle (use when it's already set at boot -- "
                          "rebooting SITL alone stalls the Gazebo lockstep loop, so only reboot "
                          "right after a fresh Gazebo+SITL restart, never standalone)")
    args = ap.parse_args()

    rclpy.init()
    node = RangeToDistanceSensor(args.connect, skip_reboot=args.skip_reboot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
