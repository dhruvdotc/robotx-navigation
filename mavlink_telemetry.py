#!/usr/bin/env python3
"""Shared live-drone-pose MAVLink telemetry reader.

Used by both simulation/accuracy_verify.py and camera_live_feed.py so a
detection's pixel-based ground offset is added to the drone's ACTUAL position
at the instant of that frame, not a single fixed point recorded at startup.
That's what makes monocular pixel-to-GPS reprojection track a moving drone
correctly instead of silently drifting as the drone flies away from wherever
it happened to be when the script launched.
"""

from __future__ import annotations

import threading


class Telemetry:
    """Background MAVLink listener exposing the drone's latest local-NED
    position, attitude, and global lat/lon/altitude via snapshot()."""

    def __init__(self, endpoint: str):
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.endpoint = endpoint
        self.lock = threading.Lock()
        self.north = self.east = self.down = 0.0
        self.vx = self.vy = 0.0
        self.roll = self.pitch = 0.0
        self.lat = self.lon = self.rel_alt = 0.0
        self.have_pose = False
        self.armed = False
        self.was_armed = False
        self.stop = False
        self.m = None

    def start(self) -> int:
        self.m = self.mavutil.mavlink_connection(self.endpoint)
        self.m.wait_heartbeat()
        # Ask for position + attitude streams (harmless if MAVProxy already did).
        for stream in (self.mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                       self.mavutil.mavlink.MAV_DATA_STREAM_EXTRA1):
            self.m.mav.request_data_stream_send(
                self.m.target_system, self.m.target_component, stream, 10, 1)
        threading.Thread(target=self._loop, daemon=True).start()
        return self.m.target_system

    def _loop(self) -> None:
        while not self.stop:
            msg = self.m.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE", "GLOBAL_POSITION_INT", "HEARTBEAT"],
                blocking=True, timeout=1.0)
            if msg is None:
                continue
            t = msg.get_type()
            with self.lock:
                if t == "LOCAL_POSITION_NED":
                    self.north, self.east, self.down = msg.x, msg.y, msg.z
                    self.vx, self.vy = msg.vx, msg.vy
                    self.have_pose = True
                elif t == "ATTITUDE":
                    self.roll, self.pitch = msg.roll, msg.pitch
                elif t == "GLOBAL_POSITION_INT":
                    self.lat, self.lon = msg.lat / 1e7, msg.lon / 1e7
                    self.rel_alt = msg.relative_alt / 1000.0
                elif t == "HEARTBEAT":
                    self.armed = bool(msg.base_mode &
                                      self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if self.armed:
                        self.was_armed = True

    def snapshot(self) -> dict:
        with self.lock:
            return dict(north=self.north, east=self.east, down=self.down,
                        vx=self.vx, vy=self.vy, roll=self.roll, pitch=self.pitch,
                        lat=self.lat, lon=self.lon, rel_alt=self.rel_alt,
                        have_pose=self.have_pose, armed=self.armed,
                        was_armed=self.was_armed)
