#!/usr/bin/env python3
"""
Mock Simulink GPS noise publisher — mirrors gps_navsatfix_sim.slx behaviour.

Subscribes to /model/iris_uav/odometry (nav_msgs/Odometry, ground truth from
Gazebo's OdometryPublisher plugin), converts to lat/lon, injects a realistic
GPS noise model, and republishes as sensor_msgs/NavSatFix on /fix at 5 Hz.

Fix-type state machine (matches .slx model):
  ~75 % of time  → STATUS_GBAS_FIX  (RTK-float,   sigma ~0.02 m)
  ~20 % of time  → STATUS_SBAS_FIX  (DGPS,         sigma ~0.40 m)
  ~5  % of time  → STATUS_FIX       (single-point, sigma ~1.50 m)

Usage (inside the sim container, after sourcing ROS):
    python3 simulation/mock_fix_publisher.py \
        --datum-lat -35.363262 \
        --datum-lon  149.165237

Run alongside run_course.sh so /model/iris_uav/odometry is being published.
The published /fix topic can then be consumed by accuracy_verify.py or
camera_live_feed.py in place of the MAVLink GPS fix.
"""

from __future__ import annotations

import argparse
import math
import random
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus


# GPS noise state machine parameters (matches gps_navsatfix_sim.slx)
_STATES = [
    # (status_code, sigma_m, weight)
    (NavSatStatus.STATUS_GBAS_FIX, 0.02, 0.75),   # RTK-float
    (NavSatStatus.STATUS_SBAS_FIX, 0.40, 0.20),   # DGPS
    (NavSatStatus.STATUS_FIX,      1.50, 0.05),   # single-point
]
_STATE_WEIGHTS = [s[2] for s in _STATES]
# Average state duration in seconds before transitioning
_MEAN_DURATION = 8.0

EARTH_R = 6_371_000.0  # metres


def ned_to_latlon(north_m: float, east_m: float,
                  datum_lat: float, datum_lon: float) -> tuple[float, float]:
    d_lat = north_m / EARTH_R
    d_lon = east_m / (EARTH_R * math.cos(math.radians(datum_lat)))
    return datum_lat + math.degrees(d_lat), datum_lon + math.degrees(d_lon)


class MockFixPublisher(Node):
    def __init__(self, datum_lat: float, datum_lon: float, pub_hz: float):
        super().__init__("mock_fix_publisher")
        self._datum_lat = datum_lat
        self._datum_lon = datum_lon

        # Latest odometry
        self._lock = threading.Lock()
        self._north = 0.0
        self._east = 0.0
        self._have_odom = False

        # Fix-type state machine
        self._state_idx = 0
        self._state_until = time.monotonic() + random.expovariate(1.0 / _MEAN_DURATION)

        self._sub = self.create_subscription(
            Odometry, "/model/iris_uav/odometry", self._odom_cb, 10)

        self._pub = self.create_publisher(NavSatFix, "/fix", 10)
        self._timer = self.create_timer(1.0 / pub_hz, self._publish)

        self.get_logger().info(
            f"mock_fix_publisher ready — datum ({datum_lat:.6f}, {datum_lon:.6f})"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self._north = msg.pose.pose.position.x
            self._east  = msg.pose.pose.position.y
            self._have_odom = True

    def _next_state(self) -> int:
        """Pick a new random fix-type state, different from the current one."""
        choices = [i for i in range(len(_STATES)) if i != self._state_idx]
        weights = [_STATE_WEIGHTS[i] for i in choices]
        total = sum(weights)
        r = random.random() * total
        acc = 0.0
        for i, w in zip(choices, weights):
            acc += w
            if r <= acc:
                return i
        return choices[-1]

    def _publish(self) -> None:
        now = time.monotonic()
        if now >= self._state_until:
            self._state_idx = self._next_state()
            self._state_until = now + random.expovariate(1.0 / _MEAN_DURATION)

        status_code, sigma_m, _ = _STATES[self._state_idx]

        with self._lock:
            north = self._north
            east  = self._east
            have  = self._have_odom

        if not have:
            return  # wait for first odometry message

        lat, lon = ned_to_latlon(north, east, self._datum_lat, self._datum_lon)

        # Add Gaussian noise proportional to current fix quality
        lat += random.gauss(0, sigma_m / EARTH_R * (180 / math.pi))
        lon += random.gauss(0, sigma_m / (EARTH_R * math.cos(math.radians(lat)))
                                       * (180 / math.pi))

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.status.status  = status_code
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude  = lat
        msg.longitude = lon
        msg.altitude  = 0.0   # AGL altitude not needed for 2D buoy projection
        # Diagonal covariance: sigma^2 in lat, lon, alt (metres^2)
        cov = sigma_m ** 2
        msg.position_covariance = [cov, 0, 0,  0, cov, 0,  0, 0, cov * 4]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._pub.publish(msg)


def main() -> None:
    ap = argparse.ArgumentParser(description="Mock Simulink GPS noise publisher")
    ap.add_argument("--datum-lat", type=float, default=-35.363262,
                    help="Gazebo world origin latitude (default: ArduPilot SITL home)")
    ap.add_argument("--datum-lon", type=float, default=149.165237,
                    help="Gazebo world origin longitude")
    ap.add_argument("--hz", type=float, default=5.0,
                    help="Publish rate for /fix (Hz, default 5)")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = MockFixPublisher(args.datum_lat, args.datum_lon, args.hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
