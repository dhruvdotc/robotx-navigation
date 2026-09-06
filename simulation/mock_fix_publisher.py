#!/usr/bin/env python3
"""
Mock Simulink GPS noise publisher — mirrors gps_navsatfix_sim.slx behaviour.

Subscribes to /model/iris_uav/odometry (nav_msgs/Odometry, ground truth from
Gazebo's OdometryPublisher plugin), converts to lat/lon, injects a realistic
GPS noise model, and republishes as sensor_msgs/NavSatFix on /fix at 5 Hz.

Fix-type state machine (matches .slx model), used in open water:
  ~75 % of time  → STATUS_GBAS_FIX  (RTK-float,   sigma ~0.02 m)
  ~20 % of time  → STATUS_SBAS_FIX  (DGPS,         sigma ~0.40 m)
  ~5  % of time  → STATUS_FIX       (single-point, sigma ~1.50 m)

Realism improvement (2026-09-05): the weights above flip near a real course
buoy/gate -- physical multipath and partial sky occlusion from a ~0.5m
structure at GPS-antenna height is a real effect, and the RobotX courses are
dense with exactly this kind of structure. Within --degrade-radius (default
8m) of the nearest buoy/gate (from the actual world-file positions for
--course 1/2/3), the weights invert to ~70% single-point / 25% DGPS / 5%
RTK-float. This replaces the purely time-based Markov dwell with a spatial
trigger tied to real course geometry -- see simulink/README.md for the
live-flight verification plot. Disable with --no-proximity-degrade to get the
old purely time-based behaviour back.

Usage (inside the sim container, after sourcing ROS):
    python3 simulation/mock_fix_publisher.py \
        --datum-lat -35.363262 \
        --datum-lon  149.165237 \
        --course 1

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
    # (status_code, sigma_m)
    (NavSatStatus.STATUS_GBAS_FIX, 0.02),   # RTK-float
    (NavSatStatus.STATUS_SBAS_FIX, 0.40),   # DGPS
    (NavSatStatus.STATUS_FIX,      1.50),   # single-point
]
# Open-water weights (ambient, matches gps_navsatfix_sim.slx) vs. near-buoy
# weights (real multipath/occlusion off a ~0.5m structure at antenna height).
_WEIGHTS_OPEN_WATER = [0.75, 0.20, 0.05]
_WEIGHTS_NEAR_BUOY  = [0.05, 0.25, 0.70]
# Average state duration in seconds before transitioning
_MEAN_DURATION = 8.0

EARTH_R = 6_371_000.0  # metres

# Real task buoy/gate (north_m, east_m) positions, read off each course's
# world SDF <model><pose> (decorative debris models d1_*/d2_*/d3_* excluded --
# those are scenery, not RobotX task markers a real GPS would multipath off
# at typical mission altitude/range).
COURSE_BUOYS = {
    1: [  # simulation/gazebo/worlds/robotx_uav_course.sdf
        (1.25, 10.0), (-1.25, 10.0),
        (1.25, 25.0), (-1.25, 25.0),
        (1.25, 40.0), (-1.25, 40.0),
        (0.0, 50.0),  # light_buoy
    ],
    2: [  # simulation/gazebo/worlds/course_2_search_field.sdf
        (10.0, 8.0), (-8.0, 24.0), (7.0, 42.0),      # green1-3
        (-11.0, 14.0), (6.0, 31.0), (-5.0, 48.0),    # red1-3
        (2.0, 55.0),                                  # light_buoy
    ],
    3: [  # simulation/gazebo/worlds/course_3_dogleg.sdf
        (1.25, 10.0), (-1.25, 10.0),
        (1.25, 25.0), (-1.25, 25.0),
        (0.0, 35.0),                    # corner_post
        (15.0, 36.25), (15.0, 33.75),
        (30.0, 36.25), (30.0, 33.75),
        (42.0, 35.0),                    # light_buoy
    ],
}


def nearest_buoy_dist(north: float, east: float, buoys: list[tuple[float, float]]) -> float:
    if not buoys:
        return math.inf
    return min(math.hypot(north - bn, east - be) for bn, be in buoys)


def ned_to_latlon(north_m: float, east_m: float,
                  datum_lat: float, datum_lon: float) -> tuple[float, float]:
    d_lat = north_m / EARTH_R
    d_lon = east_m / (EARTH_R * math.cos(math.radians(datum_lat)))
    return datum_lat + math.degrees(d_lat), datum_lon + math.degrees(d_lon)


class MockFixPublisher(Node):
    def __init__(self, datum_lat: float, datum_lon: float, pub_hz: float,
                 buoys: list[tuple[float, float]] | None = None,
                 degrade_radius: float = 8.0):
        super().__init__("mock_fix_publisher")
        self._datum_lat = datum_lat
        self._datum_lon = datum_lon
        self._buoys = buoys or []
        self._degrade_radius = degrade_radius

        # Latest odometry
        self._lock = threading.Lock()
        self._north = 0.0
        self._east = 0.0
        self._have_odom = False

        # Fix-type state machine. NOT seeded here -- see _publish(): seeding
        # the dwell timer at node-construction time was a real bug (found
        # 2026-09-05 chasing a spurious vertical spike at the very start of
        # every live-flight plot). There's a multi-second gap between this
        # node starting and real odometry/motion actually beginning (Gazebo
        # bridge connect + arm + takeoff), and _MEAN_DURATION's exponential
        # distribution has real probability mass well under 8s -- so the
        # initial dwell had often already "expired" before the vehicle ever
        # moved, firing one or more meaningless transitions while stationary
        # at the start position and bunching several different sigma values
        # onto the same (east=0) x-coordinate. Anchoring the timer to the
        # first real odometry sample instead removes that dependency on
        # incidental ROS/Gazebo startup timing entirely.
        self._state_idx = 0
        self._state_until = None

        self._sub = self.create_subscription(
            Odometry, "/model/iris_uav/odometry", self._odom_cb, 10)

        self._pub = self.create_publisher(NavSatFix, "/fix", 10)
        self._timer = self.create_timer(1.0 / pub_hz, self._publish)

        self.get_logger().info(
            f"mock_fix_publisher ready — datum ({datum_lat:.6f}, {datum_lon:.6f}), "
            f"{len(self._buoys)} buoys tracked for proximity degradation "
            f"(radius={degrade_radius:.1f}m)"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self._north = msg.pose.pose.position.y  # Gazebo ENU (heading_deg=0): y=North
            self._east  = msg.pose.pose.position.x  # Gazebo ENU (heading_deg=0): x=East
            self._have_odom = True

    def _next_state(self, near_buoy: bool) -> int:
        """Pick a new random fix-type state, different from the current one."""
        weights = _WEIGHTS_NEAR_BUOY if near_buoy else _WEIGHTS_OPEN_WATER
        choices = [i for i in range(len(_STATES)) if i != self._state_idx]
        cweights = [weights[i] for i in choices]
        total = sum(cweights)
        r = random.random() * total
        acc = 0.0
        for i, w in zip(choices, cweights):
            acc += w
            if r <= acc:
                return i
        return choices[-1]

    def _publish(self) -> None:
        with self._lock:
            north = self._north
            east  = self._east
            have  = self._have_odom

        if not have:
            return  # wait for first odometry message

        dist = nearest_buoy_dist(north, east, self._buoys)
        near_buoy = dist <= self._degrade_radius

        now = time.monotonic()
        if self._state_until is None:
            # First real sample -- anchor the dwell timer here, not at node
            # construction (see the comment in __init__).
            self._state_until = now + random.expovariate(1.0 / _MEAN_DURATION)
        if now >= self._state_until:
            self._state_idx = self._next_state(near_buoy)
            self._state_until = now + random.expovariate(1.0 / _MEAN_DURATION)

        status_code, base_sigma_m = _STATES[self._state_idx]
        # Continuous proximity multiplier on top of the discrete state's base
        # sigma: real multipath/occlusion severity scales with proximity to a
        # structure, not with a random multi-second dwell timer. Without this,
        # the label (state) can only re-roll every ~_MEAN_DURATION seconds, so
        # on a short, buoy-dense course a handful of unlucky/lucky draws can
        # swamp the proximity signal -- confirmed empirically 2026-09-05 (see
        # simulink/README.md): mean h_acc near buoys came out *better* than
        # open water in a single ~67s Course-1 flight, purely from transition-
        # timing variance, despite the weight bias correctly firing at the
        # right moments. This multiplier makes the degradation immediate and
        # deterministic in distance, independent of state-machine timing.
        closeness = max(0.0, min(1.0, (self._degrade_radius - dist) / self._degrade_radius))
        sigma_m = base_sigma_m * (1.0 + 6.0 * closeness)

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
        msg.position_covariance = [float(cov), 0.0, 0.0,  0.0, float(cov), 0.0,  0.0, 0.0, float(cov * 4)]
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
    ap.add_argument("--course", type=int, default=1, choices=[1, 2, 3],
                    help="Course number, selects the real buoy/gate positions "
                         "for proximity-based fix degradation (default 1)")
    ap.add_argument("--degrade-radius", type=float, default=8.0,
                    help="Distance in metres from the nearest buoy/gate within "
                         "which fix quality is biased toward degraded states")
    ap.add_argument("--no-proximity-degrade", action="store_true",
                    help="Disable buoy-proximity degradation; revert to the "
                         "original purely time-based Markov dwell everywhere")
    args, _ = ap.parse_known_args()

    buoys = [] if args.no_proximity_degrade else COURSE_BUOYS[args.course]

    rclpy.init()
    node = MockFixPublisher(args.datum_lat, args.datum_lon, args.hz,
                             buoys=buoys, degrade_radius=args.degrade_radius)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
