#!/usr/bin/env python3
"""Detection + GPS accuracy verification for a RobotX UAV course flight.

Runs LIVE during a flight (system python, ROS 2 sourced):
  * subscribes to the drone nadir camera (/drone/camera) and runs the EXACT
    camera_live_feed.py detection + nadir-projection pipeline,
  * reads the drone's live pose/attitude over MAVLink (pymavlink),
  * for every LEVEL frame, projects each detection to a ground point using the
    drone's LIVE altitude, adds the drone's world position, and logs the absolute
    estimate (local NED metres + GPS) to a timestamped CSV in real time.

When the flight ends (Ctrl-C, --duration elapsed, or disarm-after-arm) it
cross-references the logged detections against the ground-truth buoy positions
parsed straight from the world .sdf, and writes a timestamped Markdown report:
per-buoy error (m), mean/max error, detection count and mean confidence.

A minimum sustained flight duration (default 15 s) is enforced so a single frame
grab cannot masquerade as a verified flight -- a shorter run is flagged in the
report instead of being silently accepted.

Ground truth uses the world's own frame: Gazebo +X = NED East, +Y = North, so a
buoy at sdf <pose>X Y Z> sits at local NED (north=Y, east=X). The drone's MAVLink
home is the world datum, so drone LOCAL_POSITION_NED + the projected pixel offset
gives the buoy's absolute local NED directly -- no hand-placed hover assumptions.
"""
import argparse
import csv
import json
import math
import os
import re
import signal
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
import camera_live_feed as clf  # noqa: E402  (the real pipeline, reused verbatim)
from color_utils import load_color_ranges  # noqa: E402


# --------------------------------------------------------------------------- #
# Ground truth from the world file
# --------------------------------------------------------------------------- #
def parse_world(world_path):
    """Return (buoys, datum). buoys: list of dicts {name,color,north,east}."""
    with open(world_path, encoding="utf-8") as f:
        text = f.read()

    datum = {"lat": 0.0, "lon": 0.0}
    mlat = re.search(r"<latitude_deg>([-\d.]+)</latitude_deg>", text)
    mlon = re.search(r"<longitude_deg>([-\d.]+)</longitude_deg>", text)
    if mlat:
        datum["lat"] = float(mlat.group(1))
    if mlon:
        datum["lon"] = float(mlon.group(1))

    buoys = []
    # Each buoy is a <model name="..."> ... <pose>X Y Z r p y</pose>. Grab the
    # first pose inside each model block.
    for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', text, re.DOTALL):
        name, body = m.group(1), m.group(2)
        color = None
        if "green" in name:
            color = "green"
        elif "red" in name:
            color = "red"
        elif "light_buoy" in name:
            color = "light"  # black scan-the-code box, no colour from nadir
        if color is None:
            continue
        pose = re.search(r"<pose>\s*([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)", body)
        if not pose:
            continue
        x, y = float(pose.group(1)), float(pose.group(2))
        buoys.append({"name": name, "color": color, "north": y, "east": x})
    return buoys, datum


# --------------------------------------------------------------------------- #
# MAVLink telemetry (background thread)
# --------------------------------------------------------------------------- #
class Telemetry:
    def __init__(self, endpoint):
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

    def start(self):
        self.m = self.mavutil.mavlink_connection(self.endpoint)
        self.m.wait_heartbeat()
        # Ask for position + attitude streams (harmless if MAVProxy already did).
        for stream in (self.mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                       self.mavutil.mavlink.MAV_DATA_STREAM_EXTRA1):
            self.m.mav.request_data_stream_send(
                self.m.target_system, self.m.target_component, stream, 10, 1)
        threading.Thread(target=self._loop, daemon=True).start()
        return self.m.target_system

    def _loop(self):
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

    def snapshot(self):
        with self.lock:
            return dict(north=self.north, east=self.east, down=self.down,
                        vx=self.vx, vy=self.vy, roll=self.roll, pitch=self.pitch,
                        lat=self.lat, lon=self.lon, rel_alt=self.rel_alt,
                        have_pose=self.have_pose, armed=self.armed,
                        was_armed=self.was_armed)


# --------------------------------------------------------------------------- #
# Detection node
# --------------------------------------------------------------------------- #
def make_args(alt, no_undistort):
    from types import SimpleNamespace
    return SimpleNamespace(
        width=1920, height=1080, det_width=960, det_height=540, altitude_m=alt,
        target_diameter_m=0.32, kernel_size=5, roi_margin=0.10, min_circularity=0.35,
        min_color_ratio=0.12, track_gate_px=70.0, max_track_missed=8,
        fx_px=None, fy_px=None, cx_px=None, cy_px=None, no_undistort=no_undistort)


def build_report(buoys, datum, rows, started, ended, min_duration, match_radius,
                 csv_path, world_path, report_path, summary_json_path=None):
    duration = ended - started
    n_det = len(rows)
    # Group detections by nearest ground-truth buoy of the same colour.
    per_buoy = {b["name"]: [] for b in buoys}
    unmatched = 0
    for r in rows:
        best, best_d = None, match_radius
        for b in buoys:
            if b["color"] != r["color"]:
                continue
            d = math.hypot(r["est_north"] - b["north"], r["est_east"] - b["east"])
            if d < best_d:
                best_d, best = d, b
        if best is None:
            unmatched += 1
        else:
            per_buoy[best["name"]].append((best_d, r["confidence"]))

    detected = [b for b in buoys if per_buoy[b["name"]]]
    all_errs = [d for v in per_buoy.values() for (d, _) in v]
    mean_err = sum(all_errs) / len(all_errs) if all_errs else float("nan")
    max_err = max(all_errs) if all_errs else float("nan")
    all_conf = [c for v in per_buoy.values() for (_, c) in v]
    mean_conf = sum(all_conf) / len(all_conf) if all_conf else float("nan")

    dur_ok = duration >= min_duration
    lines = []
    lines.append(f"# RobotX UAV Course - Accuracy Verification Report\n")
    lines.append(f"- **Flight / recording timestamp:** {datetime.fromtimestamp(started):%Y-%m-%d %H:%M:%S}")
    lines.append(f"- **Flight duration:** {duration:.1f} s "
                 f"({'OK' if dur_ok else 'TOO SHORT'}; minimum {min_duration:.0f} s)")
    if not dur_ok:
        lines.append(f"- **WARNING:** flight shorter than the {min_duration:.0f}s minimum -- "
                     f"results below are NOT a verified sustained flight (possible single-frame grab).")
    lines.append(f"- **World (ground truth):** `{os.path.relpath(world_path, REPO)}`")
    lines.append(f"- **Detection log:** `{os.path.relpath(csv_path, REPO)}`")
    lines.append(f"- **Datum:** lat {datum['lat']:.6f}, lon {datum['lon']:.6f}")
    lines.append("")
    lines.append("## Summary")
    lines.append("")
    lines.append(f"- Total logged detections: **{n_det}**")
    lines.append(f"- Colour buoys detected: **{len(detected)} / "
                 f"{len([b for b in buoys if b['color'] in ('red','green')])}**")
    lines.append(f"- Mean horizontal error: **{mean_err:.2f} m**")
    lines.append(f"- Max horizontal error: **{max_err:.2f} m**")
    lines.append(f"- Mean detection confidence: **{mean_conf:.2f}**")
    lines.append(f"- Unmatched detections (no buoy within {match_radius:.1f} m): **{unmatched}**")
    lines.append("")
    lines.append("## Per-buoy accuracy")
    lines.append("")
    lines.append("| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |")
    lines.append("|------|--------|--------------|-----------:|-------------:|------------:|----------:|")
    for b in buoys:
        v = per_buoy[b["name"]]
        truth = f"{b['north']:.2f}, {b['east']:.2f}"
        if v:
            errs = [d for (d, _) in v]
            confs = [c for (_, c) in v]
            lines.append(f"| {b['name']} | {b['color']} | {truth} | {len(v)} | "
                         f"{sum(errs)/len(errs):.2f} | {max(errs):.2f} | "
                         f"{sum(confs)/len(confs):.2f} |")
        else:
            note = " (nadir: black box, no colour - expected miss)" if b["color"] == "light" else ""
            lines.append(f"| {b['name']} | {b['color']} | {truth} | 0 | - | - | -{note} |")
    lines.append("")
    lines.append("_Each detection is matched to the nearest same-colour ground-truth buoy "
                 "within the match radius; error is the horizontal distance between the "
                 "projected absolute position and the buoy's true position._")
    lines.append("")

    report = "\n".join(lines)
    os.makedirs(os.path.dirname(os.path.abspath(report_path)), exist_ok=True)
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(report)
    # Also refresh the canonical latest-report path.
    latest = os.path.join(os.path.dirname(report_path), "accuracy_report.md")
    if os.path.abspath(latest) != os.path.abspath(report_path):
        with open(latest, "w", encoding="utf-8") as f:
            f.write(report)

    colour_buoys_total = len([b for b in buoys if b["color"] in ("red", "green")])
    summary = {
        "timestamp": datetime.fromtimestamp(started).isoformat(timespec="seconds"),
        "duration_s": round(duration, 1),
        "duration_ok": dur_ok,
        "n_detections": n_det,
        "colour_buoys_detected": len(detected),
        "colour_buoys_total": colour_buoys_total,
        "mean_error_m": None if math.isnan(mean_err) else round(mean_err, 3),
        "max_error_m": None if math.isnan(max_err) else round(max_err, 3),
        "mean_confidence": None if math.isnan(mean_conf) else round(mean_conf, 3),
        "unmatched_detections": unmatched,
        "per_buoy": [
            {
                "name": b["name"],
                "color": b["color"],
                "true_north_m": b["north"],
                "true_east_m": b["east"],
                "detections": len(per_buoy[b["name"]]),
                "mean_error_m": round(sum(d for d, _ in per_buoy[b["name"]]) / len(per_buoy[b["name"]]), 3)
                    if per_buoy[b["name"]] else None,
                "mean_confidence": round(sum(c for _, c in per_buoy[b["name"]]) / len(per_buoy[b["name"]]), 3)
                    if per_buoy[b["name"]] else None,
            }
            for b in buoys
        ],
        "files": {
            "detections_csv": os.path.basename(csv_path),
            "accuracy_report": os.path.basename(report_path),
        },
    }
    if summary_json_path:
        os.makedirs(os.path.dirname(os.path.abspath(summary_json_path)), exist_ok=True)
        with open(summary_json_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)

    return report, report_path, latest, summary


def _print_ros_help(missing, connect):
    """Explain a missing live-camera dependency (rclpy is part of ROS, not pip)."""
    print(f"\n[ERROR] could not import '{missing}' -- the live camera path needs a "
          "sourced ROS 2 environment.\n"
          "        Run it like:\n"
          "            source /opt/ros/humble/setup.bash\n"
          f"            python3 simulation/accuracy_verify.py --connect {connect}\n"
          "        ROS 2 Humble is already installed here (/opt/ros/humble) -- do NOT "
          "pip install rclpy.\n"
          "        To rebuild a report from an existing log WITHOUT ROS, use:\n"
          "            python3 simulation/accuracy_verify.py --from-csv <detections.csv>",
          file=sys.stderr)


def report_from_csv(args, buoys, datum):
    """OFFLINE: rebuild the report from a logged detections CSV (no ROS/MAVLink).

    The CSV is the one written live during a flight (its est_north_m/est_east_m
    columns already hold the absolute projected positions), so verification can be
    re-run on any machine with just Python -- no rclpy, no telemetry, no sim.
    """
    csv_path = args.from_csv
    if not os.path.isfile(csv_path):
        print(f"[ERROR] --from-csv file not found: {csv_path}", file=sys.stderr)
        return 2
    rows, stamps = [], []
    with open(csv_path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                rows.append(dict(color=r["color"], confidence=float(r["confidence"]),
                                 est_north=float(r["est_north_m"]),
                                 est_east=float(r["est_east_m"])))
                stamps.append(float(r["timestamp"]))
            except (KeyError, ValueError) as exc:
                print(f"[WARN] skipping malformed CSV row ({exc})", file=sys.stderr)
    if not stamps:
        print(f"[ERROR] no usable detection rows in {csv_path}", file=sys.stderr)
        return 2
    started, ended = min(stamps), max(stamps)
    # Tie the report name to the source log (detections_<ts>.csv -> accuracy_report_<ts>.md).
    base = os.path.basename(csv_path)
    ts = base[len("detections_"):-len(".csv")] if base.startswith("detections_") \
        else datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = os.path.join(args.report_dir, f"accuracy_report_{ts}.md")
    summary_json = os.path.join(os.path.dirname(report_path), "summary.json") \
        if args.summary_json is None else args.summary_json
    report, rp, latest, _ = build_report(
        buoys, datum, rows, started, ended, args.min_duration, args.match_radius,
        csv_path, args.world, report_path, summary_json_path=summary_json)
    print("\n" + report)
    print(f"[INFO] offline report from {csv_path}")
    print(f"[INFO] report written: {rp}")
    print(f"[INFO] latest report : {latest}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default=os.path.join(
        REPO, "simulation/gazebo/worlds/robotx_uav_course.sdf"))
    ap.add_argument("--ros-topic", default="/drone/camera")
    ap.add_argument("--connect", default="udp:127.0.0.1:14551",
                    help="MAVLink endpoint for drone telemetry (MAVProxy out #2; "
                         "fly_course.py uses 14550, so this defaults to 14551).")
    ap.add_argument("--min-duration", type=float, default=15.0)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="Auto-stop after this many seconds (0 = run until Ctrl-C / disarm).")
    ap.add_argument("--level-deg", type=float, default=1.5,
                    help="Only log frames with |roll|,|pitch| below this (deg).")
    ap.add_argument("--max-speed", type=float, default=0.5,
                    help="Only log frames with horizontal speed below this (m/s).")
    ap.add_argument("--match-radius", type=float, default=3.0)
    ap.add_argument("--no-undistort", action="store_true", default=True,
                    help="ogre2 renders a clean pinhole; keep distortion off (default).")
    ap.add_argument("--out-dir", default=os.path.join(REPO, "simulation/accuracy_logs"))
    ap.add_argument("--report-dir", default=os.path.join(REPO, "simulation"))
    ap.add_argument("--from-csv", default=None, metavar="PATH",
                    help="OFFLINE mode: skip ROS/MAVLink entirely and rebuild the "
                         "accuracy report from an already-logged detections CSV. "
                         "Needs NO rclpy / ROS -- pure stdlib + the world ground truth.")
    ap.add_argument("--summary-json", default=None, metavar="PATH",
                    help="Write a summary.json alongside the report (default: same dir as report).")
    args = ap.parse_args()

    buoys, datum = parse_world(args.world)
    print(f"[INFO] ground truth: {len(buoys)} buoys from {args.world}")
    for b in buoys:
        print(f"       {b['name']:<14} {b['color']:<6} N={b['north']:+.2f} E={b['east']:+.2f}")

    # Offline path: rebuild a report from a logged CSV with zero ROS dependency.
    if args.from_csv:
        return report_from_csv(args, buoys, datum)

    # Fail fast with a clear hint if the live-camera ROS deps are missing, BEFORE
    # we block waiting for a MAVLink heartbeat.
    import importlib.util
    missing = next((m for m in ("rclpy", "cv_bridge", "sensor_msgs")
                    if importlib.util.find_spec(m) is None), None)
    if missing:
        _print_ros_help(missing, args.connect)
        return 2

    clf.COLOR_RANGES = load_color_ranges(classes_dir=os.path.join(REPO, "captures/classes"))
    calib = clf.load_calibration(os.path.join(REPO, "calibration/camera_intrinsics_latest.json"))

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, f"detections_{ts}.csv")
    report_path = os.path.join(args.report_dir, f"accuracy_report_{ts}.md")

    print(f"[INFO] connecting telemetry on {args.connect} ...")
    tel = Telemetry(args.connect)
    tel.start()
    print("[INFO] telemetry connected.")

    # ROS camera. rclpy ships WITH ROS 2 (it is not a standalone pip package) and
    # only joins the Python path after the ROS env is sourced -- so a bare
    # ModuleNotFoundError almost always means ROS just wasn't sourced, not that it
    # is missing. (Use --from-csv to rebuild a report with no ROS at all.)
    try:
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from sensor_msgs.msg import Image
    except ModuleNotFoundError as exc:  # backstop: find_spec passed but import failed
        tel.stop = True
        _print_ros_help(exc.name, args.connect)
        return 2

    # Per-frame altitude is taken live from telemetry below; 10.0 is just the
    # placeholder used until the first LOCAL_POSITION_NED / GLOBAL_POSITION_INT.
    cl_args = make_args(10.0, args.no_undistort)
    intr = clf.resolve_intrinsics(cl_args, calib, cl_args.width, cl_args.height)
    print(f"[INFO] intrinsics[{intr.source}] fx={intr.fx:.1f}")

    rows = []
    state = {"started": None, "ended": None, "frames": 0, "logged": 0, "skipped": 0}
    csv_file = open(csv_path, "w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(["timestamp", "color", "confidence", "cx", "cy",
                     "off_north_m", "off_east_m", "drone_north_m", "drone_east_m",
                     "alt_m", "roll_deg", "pitch_deg",
                     "est_north_m", "est_east_m", "est_lat", "est_lon"])

    class Det(Node):
        def __init__(self):
            super().__init__("accuracy_verify")
            self.b = CvBridge()
            self.tracks = []
            self.nid = 1
            self.create_subscription(Image, args.ros_topic, self.cb, 10)

        def cb(self, msg):
            snap = tel.snapshot()
            if not snap["have_pose"]:
                return
            try:
                img = self.b.imgmsg_to_cv2(msg, "bgr8")
            except Exception:
                return
            if state["started"] is None and snap["was_armed"]:
                state["started"] = time.time()
                print("[INFO] flight detected (armed) -- logging started.", flush=True)
            state["frames"] += 1

            roll_d, pitch_d = math.degrees(snap["roll"]), math.degrees(snap["pitch"])
            speed = math.hypot(snap["vx"], snap["vy"])
            level = abs(roll_d) < args.level_deg and abs(pitch_d) < args.level_deg
            if not (level and speed < args.max_speed):
                state["skipped"] += 1
                return

            alt = max(snap["rel_alt"], -snap["down"], 0.5)
            cl_args.altitude_m = alt
            ff = clf.apply_clahe_to_v(img)
            fd = cv2.resize(ff, (cl_args.det_width, cl_args.det_height),
                            interpolation=cv2.INTER_AREA)
            hd = cv2.cvtColor(fd, cv2.COLOR_BGR2HSV)
            hf = cv2.cvtColor(ff, cv2.COLOR_BGR2HSV)
            mx, my = int(cl_args.roi_margin * cl_args.det_width), int(cl_args.roi_margin * cl_args.det_height)
            roi = (mx, my, cl_args.det_width - mx, cl_args.det_height - my)
            dets = clf.find_detections(ff, fd, hd, hf, roi, cl_args, intr)
            self.tracks, assigned, self.nid = clf.update_tracks(
                self.tracks, dets, cl_args.track_gate_px, cl_args.max_track_missed, self.nid)

            now = time.time()
            for det, tid, (sx, sy) in assigned:
                n_off, e_off = clf.project_pixel_to_ground_ned(sx, sy, intr, alt)
                est_n = snap["north"] + n_off
                est_e = snap["east"] + e_off
                est_lat, est_lon = clf.ned_to_gps(est_n, est_e, datum["lat"], datum["lon"])
                rows.append(dict(color=det.color, confidence=det.confidence,
                                 est_north=est_n, est_east=est_e))
                writer.writerow([f"{now:.3f}", det.color, f"{det.confidence:.4f}",
                                 f"{sx:.1f}", f"{sy:.1f}", f"{n_off:+.3f}", f"{e_off:+.3f}",
                                 f"{snap['north']:+.3f}", f"{snap['east']:+.3f}", f"{alt:.2f}",
                                 f"{roll_d:+.2f}", f"{pitch_d:+.2f}",
                                 f"{est_n:+.3f}", f"{est_e:+.3f}", f"{est_lat:.8f}", f"{est_lon:.8f}"])
                state["logged"] += 1
                print(f"[LOG] {det.color:<5} conf={det.confidence:.2f} "
                      f"est NED=N{est_n:+.2f} E{est_e:+.2f} (alt {alt:.1f}m, "
                      f"roll{roll_d:+.1f} pitch{pitch_d:+.1f})", flush=True)
            csv_file.flush()

    rclpy.init()
    node = Det()

    stop = {"flag": False}

    def finish(*_):
        stop["flag"] = True

    signal.signal(signal.SIGINT, finish)
    signal.signal(signal.SIGTERM, finish)

    print("[INFO] verifying... fly the course now. Ctrl-C to finish (or it auto-finishes on disarm).",
          flush=True)
    t_launch = time.time()
    try:
        while not stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.2)
            snap = tel.snapshot()
            if args.duration > 0 and state["started"] and (time.time() - state["started"]) >= args.duration:
                print("[INFO] --duration reached; finishing.", flush=True)
                break
            if snap["was_armed"] and not snap["armed"] and state["started"]:
                # disarmed after flying -> flight over
                print("[INFO] vehicle disarmed; finishing.", flush=True)
                break
    finally:
        tel.stop = True
        csv_file.flush()
        csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    started = state["started"] or t_launch
    ended = time.time()
    summary_json = args.summary_json or os.path.join(os.path.dirname(report_path), "summary.json")
    report, rp, latest, _ = build_report(
        buoys, datum, rows, started, ended, args.min_duration, args.match_radius,
        csv_path, args.world, report_path, summary_json_path=summary_json)
    print("\n" + report)
    print(f"[INFO] report written: {rp}")
    print(f"[INFO] latest report : {latest}")
    print(f"[INFO] detection log : {csv_path}")
    print(f"[INFO] summary json  : {summary_json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
