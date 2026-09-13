#!/usr/bin/env python3
"""
Close the loop between optical_flow_sim.slx and ArduPilot SITL.

Subscribes to /optical_flow (geometry_msgs/TwistStamped, published by the
Simulink optical-flow noise model against real Gazebo odometry -- see
build_optical_flow_model.m for why TwistStamped and what its fields mean)
and injects it into ArduPilot SITL as a MAVLink OPTICAL_FLOW message.

STATUS: UNTESTED end to end (built alongside optical_flow_sim.slx, which is
itself untested -- see that file's header and simulink/README.md).

A real wrinkle found while writing this, worth recording: pymavlink's
OPTICAL_FLOW message has two ways to carry flow data --
  - flow_rate_x/flow_rate_y (float, rad/s): the modern, physically-meaningful
    fields. AP_OpticalFlow_MAV::handle_msg (confirmed by reading
    ~/ardupilot/libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp) prefers
    these whenever they're nonzero.
  - flow_x/flow_y (int16, legacy "dpix" units tied to a specific sensor's
    pixel resolution/focal length): used only as a fallback when flow_rate
    is exactly zero.
This model computes real angular flow rate in rad/s, so flow_rate_x/y is the
only representation that means anything here -- there is no principled way
to backfill flow_x/y in legacy dpix units without picking an arbitrary
sensor calibration constant, so this script deliberately does NOT try; if
flow_rate can't be sent, it fails loudly instead of silently sending
meaningless legacy-unit numbers dressed up as real data.

Whether flow_rate_x/y are actually sendable depends on pymavlink having
auto-upgraded the connection to a MAVLink 2.0 dialect by the time this script
calls optical_flow_send() -- confirmed by inspecting the installed pymavlink
(2026-09-13): a connection's .mav object only exposes flow_rate_x/y as
keyword args on optical_flow_send() AFTER it has seen a MAVLink2 heartbeat
from the peer (before that, .mav defaults to a v1.0-only dialect with no
flow_rate fields at all). fix_to_gps_input.py's working GPS_INPUT injection
already relies on the same auto-upgrade (GPS_INPUT doesn't exist in v1.0
either), so this should hold here too -- but it was not possible to verify
against a live SITL in this session (see simulink/README.md), hence the
explicit TypeError guard below rather than assuming silently.

Requires SITL configured with FLOW_TYPE=5 (MAVLink) -- bake in at boot via
--add-param-file=optical_flow_mav_params.parm. If already set, --skip-reboot.

Usage:
    python3 simulink/optical_flow_bridge.py --connect tcp:127.0.0.1:5760 --skip-reboot
"""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

FLOW_TYPE_MAV = 5


def ensure_flow_type_mav(m, force_reboot: bool) -> None:
    """Set FLOW_TYPE=5 (MAVLink), optionally rebooting SITL so it takes effect."""
    print("[optical_flow_bridge] Setting FLOW_TYPE=5 (MAVLink)...")
    m.mav.param_set_send(
        m.target_system, m.target_component, b"FLOW_TYPE",
        float(FLOW_TYPE_MAV), mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    time.sleep(1)
    if not force_reboot:
        return
    print("[optical_flow_bridge] Rebooting SITL so FLOW_TYPE takes effect...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
        1, 0, 0, 0, 0, 0, 0,
    )
    m.close()
    print("[optical_flow_bridge] Waiting for SITL to come back up...")
    time.sleep(10)


class FlowToOpticalFlow(Node):
    def __init__(self, connect_str: str, skip_reboot: bool = False):
        super().__init__("optical_flow_bridge")

        print(f"[optical_flow_bridge] Connecting to SITL at {connect_str}...")
        m = mavutil.mavlink_connection(connect_str, source_system=1, source_component=197)
        m.wait_heartbeat(timeout=15)
        print("[optical_flow_bridge] Heartbeat received (connection should now be MAVLink 2.0)")

        ensure_flow_type_mav(m, force_reboot=not skip_reboot)

        if skip_reboot:
            self._mav = m
        else:
            self._mav = mavutil.mavlink_connection(connect_str, source_system=1, source_component=197)
            self._mav.wait_heartbeat(timeout=20)
        print("[optical_flow_bridge] SITL ready, relaying /optical_flow -> OPTICAL_FLOW")

        self._hb_timer = self.create_timer(1.0, self._send_heartbeat)

        self._count = 0
        self._warned_no_flow_rate = False
        self._sub = self.create_subscription(TwistStamped, "/optical_flow", self._on_flow, 10)

    def _send_heartbeat(self) -> None:
        self._mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )

    def _on_flow(self, msg: TwistStamped) -> None:
        flow_rate_x = float(msg.twist.linear.x)
        flow_rate_y = float(msg.twist.linear.y)
        quality = int(max(0, min(255, msg.twist.angular.z)))  # see build_optical_flow_model.m header

        try:
            self._mav.mav.optical_flow_send(
                int(time.time() * 1e6),  # time_usec
                0,                       # sensor_id
                0, 0,                    # flow_x, flow_y (legacy, deliberately unset -- see module docstring)
                0.0, 0.0,                # flow_comp_m_x, flow_comp_m_y (unused by AP_OpticalFlow_MAV)
                quality,
                0.0,                     # ground_distance (0 = "let the rangefinder/baro supply this")
                flow_rate_x=flow_rate_x,
                flow_rate_y=flow_rate_y,
            )
        except TypeError:
            if not self._warned_no_flow_rate:
                self._warned_no_flow_rate = True
                self.get_logger().error(
                    "optical_flow_send() on this connection does not accept flow_rate_x/flow_rate_y "
                    "-- the connection likely hasn't upgraded to a MAVLink 2.0 dialect object yet, or "
                    "this pymavlink version's generated wrapper doesn't forward those fields. See this "
                    "script's module docstring. Refusing to send legacy flow_x/flow_y in fabricated "
                    "units instead of failing loudly."
                )
            return

        self._count += 1
        if self._count % 25 == 0:
            self.get_logger().info(
                f"Injected {self._count} OPTICAL_FLOW messages "
                f"(flow_rate=({flow_rate_x:.3f}, {flow_rate_y:.3f}) rad/s, quality={quality})"
            )


def main() -> None:
    ap = argparse.ArgumentParser(description="Inject Simulink /optical_flow into ArduPilot SITL as OPTICAL_FLOW")
    ap.add_argument("--connect", default="tcp:127.0.0.1:5760")
    ap.add_argument("--skip-reboot", action="store_true",
                     help="Skip the FLOW_TYPE reboot cycle (use when it's already set at boot -- "
                          "rebooting SITL alone stalls the Gazebo lockstep loop, so only reboot "
                          "right after a fresh Gazebo+SITL restart, never standalone)")
    args = ap.parse_args()

    rclpy.init()
    node = FlowToOpticalFlow(args.connect, skip_reboot=args.skip_reboot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
