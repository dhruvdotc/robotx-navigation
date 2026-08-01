#!/usr/bin/env python3
"""Stage-A RGB color detection pipeline for USB camera feeds.

Frame sources: live USB camera (default), a single video file
(``--video-path``), or a folder of still images (``--image-dir``).

Pixel detections are projected to a local ground frame (NED metres) and an
absolute GPS coordinate using the camera intrinsics. Intrinsics are resolved
with the precedence:  CLI override  >  calibration file  >  error (no hardcoded fallback).
"""

import argparse
import csv
import glob
import json
import math
import os
import platform
import sys
import time
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np
from color_utils import build_mask, color_normalize, is_off_buoy, load_color_ranges


class TrackFlashState:
    """P2: rolling ON/OFF history of a track, used to tell a flashing beacon from
    a steady one (see docs/10_safe_passage.md). Each frame the tracker records
    ON (the track matched a detection) or OFF (the track is alive but unmatched,
    i.e. the light is dark this frame). The window is measured in FRAMES, not
    seconds, so classification is deterministic offline and framerate-agnostic;
    size it to a few flash periods (1 s on / 1 s off = ~2*fps frames per period).
    """

    def __init__(self, window: int = 60) -> None:
        self.obs: deque[bool] = deque(maxlen=window)

    def observe(self, on: bool) -> None:
        self.obs.append(on)

    def classify(self, min_frames: int, min_toggles: int, solid_ratio: float) -> str:
        n = len(self.obs)
        if n < min_frames:
            return "unknown"
        seq = list(self.obs)
        on_ratio = sum(seq) / n
        toggles = sum(1 for a, b in zip(seq, seq[1:]) if a != b)
        if on_ratio >= solid_ratio and toggles <= 1:
            return "solid"
        if toggles >= min_toggles and 0.2 <= on_ratio <= 0.8:
            return "flashing"
        return "unknown"


@dataclass
class Detection:
    color: str
    confidence: float
    cx_full: float
    cy_full: float
    radius_det: float
    bbox_full: tuple[int, int, int, int]


@dataclass
class Track:
    track_id: int
    color: str
    kf: cv2.KalmanFilter
    missed: int
    flash: TrackFlashState = field(default_factory=TrackFlashState)


@dataclass
class Intrinsics:
    """Resolved camera intrinsics plus the source they came from."""

    fx: float
    fy: float
    cx: float
    cy: float
    dist: np.ndarray  # Brown-Conrady (k1, k2, p1, p2, k3); zeros = pinhole.
    K: np.ndarray
    source: str


COLOR_RANGES: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {}
COLOR_DRAW = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 100, 0),
    "black": (60, 60, 60),  # OFF/unlit Safe-Passage beacon (P1); drawn dark grey
    "unknown": (255, 255, 255),
}

DEFAULT_CALIBRATION = "calibration/camera_intrinsics_latest.json"
# No hardcoded focal-length fallback. To swap cameras:
#   1. Run a checkerboard calibration → produce a new JSON.
#   2. Replace DEFAULT_CALIBRATION path, or pass --calibration-file.
# All GPS projections flow exclusively from that file (or explicit CLI flags).


def make_kalman(init_x: float, init_y: float) -> cv2.KalmanFilter:
    kf = cv2.KalmanFilter(4, 2)
    kf.transitionMatrix = np.array(
        [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32
    )
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 2.0
    kf.errorCovPost = np.eye(4, dtype=np.float32)
    kf.statePost = np.array([[init_x], [init_y], [0], [0]], np.float32)
    return kf


def load_calibration(path: str) -> dict | None:
    """Load checkerboard calibration JSON. Returns None if missing/unreadable."""
    if not path or not os.path.exists(path):
        print(f"[WARN] Calibration file not found: {path}; will use CLI/legacy intrinsics.")
        return None
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        # Touch the keys we rely on so a malformed file fails here, not later.
        _ = data["K"], data["fx"], data["fy"], data["cx"], data["cy"]
        _ = data["distortion"]["coefficients"]
        rms = data.get("calibration", {}).get("rms_reprojection_error", float("nan"))
        print(
            f"[INFO] Loaded camera calibration from {path} "
            f"(fx={data['fx']:.2f}, fy={data['fy']:.2f}, cx={data['cx']:.2f}, "
            f"cy={data['cy']:.2f}, RMS={rms:.3f}px)"
        )
        return data
    except (KeyError, ValueError, OSError) as exc:
        print(f"[WARN] Failed to parse calibration {path}: {exc}; using CLI/legacy intrinsics.")
        return None


def resolve_intrinsics(
    args: argparse.Namespace, calib: dict | None, width: int, height: int
) -> Intrinsics:
    """Resolve camera intrinsics with precedence: CLI override > calibration file > error.

    Plug-and-play camera swap:
      • Replace calibration/camera_intrinsics_latest.json with the new camera's
        checkerboard calibration output, OR
      • Pass --calibration-file /path/to/new_intrinsics.json at the CLI.
      All GPS projections then automatically use the new matrix — no source
      changes required.

    Manual override (pinhole-only, no distortion correction):
      Pass --fx-px and --fy-px. Any missing values are filled from the
      calibration file if one is available; cx/cy default to the image centre.
      Both --fx-px AND --fy-px must be given if no calibration file is present.
    """
    cli_keys = [args.fx_px, args.fy_px, args.cx_px, args.cy_px]
    manual = any(v is not None for v in cli_keys)

    if manual:
        # Fill any unspecified CLI values from the calibration file when available.
        if calib is not None:
            fx = args.fx_px if args.fx_px is not None else float(calib["fx"])
            fy = args.fy_px if args.fy_px is not None else float(calib["fy"])
            cx = args.cx_px if args.cx_px is not None else float(calib["cx"])
            cy = args.cy_px if args.cy_px is not None else float(calib["cy"])
        else:
            missing = [name for name, val in [("--fx-px", args.fx_px), ("--fy-px", args.fy_px)]
                       if val is None]
            if missing:
                print(
                    f"[ERROR] {', '.join(missing)} must be provided when using CLI intrinsics "
                    f"without a calibration file.\n"
                    f"        Provide all focal lengths, or supply --calibration-file.",
                    file=sys.stderr,
                )
                sys.exit(1)
            fx = args.fx_px
            fy = args.fy_px
            cx = args.cx_px if args.cx_px is not None else width / 2.0
            cy = args.cy_px if args.cy_px is not None else height / 2.0
        dist = np.zeros(5, dtype=np.float64)
        source = "cli-manual"
    elif calib is not None:
        fx = float(calib["fx"])
        fy = float(calib["fy"])
        cx = float(calib["cx"])
        cy = float(calib["cy"])
        dist = np.array(calib["distortion"]["coefficients"][:5], dtype=np.float64)
        source = "calibration-file"
        if args.no_undistort:
            dist = np.zeros(5, dtype=np.float64)
            source = "calibration-file(no-undistort)"
    else:
        # No calibration file AND no CLI intrinsics: refuse to guess.
        # Silently wrong focal lengths corrupt every GPS projection.
        print(
            f"[ERROR] Camera calibration is required but could not be loaded.\n"
            f"        To fix:\n"
            f"          • Ensure {DEFAULT_CALIBRATION} exists  "
            f"(for the current camera)\n"
            f"          • Or pass --calibration-file /path/to/intrinsics.json  "
            f"(to use a different camera)\n"
            f"          • Or pass --fx-px <fx> --fy-px <fy>  "
            f"(pinhole-only, skips distortion correction)",
            file=sys.stderr,
        )
        sys.exit(1)

    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    return Intrinsics(fx=fx, fy=fy, cx=cx, cy=cy, dist=dist, K=K, source=source)


def project_pixel_to_ground_ned(
    px: float, py: float, intr: Intrinsics, altitude_m: float, heading_deg: float = 0.0
) -> tuple[float, float]:
    """Project a full-frame pixel to a ground point in local NED metres.

    Assumes a nadir-pointing (straight-down) camera at ``altitude_m`` above a
    flat ground plane, rigidly mounted to the airframe. At ``heading_deg=0``
    (the default, and what the sim always flies with WP_YAW_BEHAVIOR=0):
    image +x -> East, image +y -> South. cv2.undistortPoints removes the
    Brown-Conrady lens distortion and returns normalised image coordinates,
    which scale by altitude to body-relative ground offsets.

    ``heading_deg`` (compass bearing, clockwise from North, matching MAVLink
    yaw) rotates that body-relative offset into true North/East, since a real
    drone doesn't fly yaw-locked like the sim does: image-up is the airframe's
    nose direction (heading), not always geographic North. This still assumes
    perfect nadir (no pitch/roll) -- see docs/07_roadmap.md known bugs.
    """
    pts = np.array([[[float(px), float(py)]]], dtype=np.float64)
    undistorted = cv2.undistortPoints(pts, intr.K, intr.dist)
    x_n = float(undistorted[0, 0, 0])
    y_n = float(undistorted[0, 0, 1])
    forward_m = -y_n * altitude_m  # body-relative: along the nose direction
    right_m = x_n * altitude_m  # body-relative: to the right of the nose

    if heading_deg == 0.0:
        return forward_m, right_m  # north, east (unchanged from before)

    theta = math.radians(heading_deg)
    north = forward_m * math.cos(theta) - right_m * math.sin(theta)
    east = forward_m * math.sin(theta) + right_m * math.cos(theta)
    return north, east


def ned_to_gps(
    north_m: float, east_m: float, origin_lat: float, origin_lon: float
) -> tuple[float, float]:
    """Convert a local NED offset (metres) to absolute lat/lon (deg).

    Equirectangular approximation about the origin. Without live telemetry the
    origin is a configurable placeholder (default 0,0); the meaningful,
    calibration-sensitive quantity is the NED offset in metres.
    """
    earth_r = 6378137.0  # WGS84 equatorial radius (m).
    d_lat = north_m / earth_r
    d_lon = east_m / (earth_r * math.cos(math.radians(origin_lat)))
    return origin_lat + math.degrees(d_lat), origin_lon + math.degrees(d_lon)


def open_camera(camera_index: int, width: int, height: int) -> cv2.VideoCapture:
    system = platform.system().lower()
    if system == "darwin":
        cap = cv2.VideoCapture(camera_index, cv2.CAP_AVFOUNDATION)
    else:
        cap = cv2.VideoCapture(camera_index)

    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    return cap


def find_working_camera(max_index: int, width: int, height: int) -> int | None:
    for index in range(max_index + 1):
        cap = open_camera(index, width, height)
        if not cap.isOpened():
            cap.release()
            continue
        ok, _ = cap.read()
        cap.release()
        if ok:
            return index
    return None


def ros_frame_source(topic: str):
    """Yield (frame_bgr, topic) tuples from a live ROS 2 sensor_msgs/Image topic.

    rclpy is imported lazily here so the script still runs under the project
    .venv (which has no ROS) for camera/video/image-dir sources; only
    --ros-topic requires a sourced ROS 2 environment (system python).

    Image decoding is done manually with numpy instead of cv_bridge: the sim
    camera publishes plain rgb8/bgr8, which is a frombuffer+reshape, and
    cv_bridge's compiled cvtColor2 hard-crashes ("_ARRAY_API not found") when
    a pip NumPy 2.x shadows the NumPy 1.x it was built against - an ABI trap
    this repo hit in practice once torch/ultralytics pulled in NumPy 2.
    """
    try:
        import rclpy
        from sensor_msgs.msg import Image
    except ImportError as exc:
        print(
            f"[ERROR] --ros-topic needs rclpy; run under a sourced ROS 2 "
            f"environment (e.g. `source /opt/ros/humble/setup.bash`). Import failed: {exc}"
        )
        return

    state = {"frame": None, "seq": 0}

    def _decode(msg) -> np.ndarray | None:
        if msg.encoding not in ("rgb8", "bgr8"):
            print(f"[WARN] Unsupported image encoding '{msg.encoding}' "
                  f"(expected rgb8/bgr8); frame dropped.")
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.step
        if buf.size < expected:
            return None
        # Respect row stride (step may exceed width*3 with padding), then crop.
        img = buf[:expected].reshape(msg.height, msg.step)[:, : msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]  # RGB -> BGR for OpenCV
        return np.ascontiguousarray(img)

    def _cb(msg):
        try:
            frame = _decode(msg)
            if frame is not None:
                state["frame"] = frame
                state["seq"] += 1
        except Exception as exc:  # noqa: BLE001 - keep streaming on a bad frame
            print(f"[WARN] Failed to convert ROS image: {exc}")

    rclpy.init()
    node = rclpy.create_node("camera_live_feed")
    node.create_subscription(Image, topic, _cb, 10)
    print(f"[INFO] Subscribed to ROS image topic {topic}; waiting for frames...")
    last_seq = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if state["seq"] != last_seq:
                last_seq = state["seq"]
                yield state["frame"], topic
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def frame_source(args: argparse.Namespace):
    """Yield (frame_bgr, label) tuples from a ROS topic, image dir, video file, or camera."""
    if getattr(args, "ros_topic", None):
        yield from ros_frame_source(args.ros_topic)
        return

    if args.image_dir is not None:
        exts = (".jpg", ".jpeg", ".png", ".bmp")
        paths = sorted(
            p for p in glob.glob(os.path.join(args.image_dir, "*"))
            if p.lower().endswith(exts)
        )
        if not paths:
            print(f"No images found in {args.image_dir} (looked for {exts}).")
            return
        print(f"Reading {len(paths)} image(s) from {args.image_dir}")
        for path in paths:
            frame = cv2.imread(path)
            if frame is None:
                print(f"[WARN] Could not read {path}; skipping.")
                continue
            yield frame, path
        return

    if args.video_path is not None:
        cap = cv2.VideoCapture(args.video_path)
        if not cap.isOpened():
            print(f"Failed to open video {args.video_path}.")
            return
        print(f"Reading video {args.video_path}")
        idx = 0
        try:
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                yield frame, f"{args.video_path}#f{idx}"
                idx += 1
        finally:
            cap.release()
        return

    # Live camera mode.
    if args.camera_index is None:
        print(f"Probing camera indices 0..{args.max_index}...")
        camera_index = find_working_camera(args.max_index, args.width, args.height)
        if camera_index is None:
            print("No working camera stream found.")
            print("On macOS, grant camera permission to your terminal/IDE and retry.")
            return
        print(f"Using camera index: {camera_index}")
    else:
        camera_index = args.camera_index

    cap = open_camera(camera_index, args.width, args.height)
    if not cap.isOpened():
        print(f"Failed to open camera index {camera_index}.")
        return
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Frame read failed. Camera may have disconnected.")
                break
            yield frame, ""
    finally:
        cap.release()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Stage-A object-first + HSV color classification.")
    # Frame source (mutually exclusive in spirit; camera is the default).
    parser.add_argument("--camera-index", type=int, default=None)
    parser.add_argument("--max-index", type=int, default=10)
    parser.add_argument("--video-path", type=str, default=None, help="Read frames from a video file.")
    parser.add_argument("--image-dir", type=str, default=None, help="Read frames from a folder of images.")
    parser.add_argument(
        "--ros-topic", type=str, default=None,
        help="Read frames live from a ROS 2 sensor_msgs/Image topic (needs a sourced ROS env), "
        "e.g. --ros-topic /drone/camera.",
    )
    parser.add_argument(
        "--no-display", "--headless", dest="no_display", action="store_true",
        help="Disable the OpenCV preview window (alias: --headless).",
    )
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--det-width", type=int, default=960)
    parser.add_argument("--det-height", type=int, default=540)
    parser.add_argument("--altitude-m", type=float, default=10.0)
    parser.add_argument(
        "--heading-deg", type=float, default=0.0,
        help="Drone compass heading in degrees, clockwise from North (MAVLink yaw convention). "
        "Rotates the pixel->NED projection accordingly; default 0 matches the sim, which always "
        "flies yaw-locked (WP_YAW_BEHAVIOR=0).",
    )
    # Intrinsics: CLI override > calibration file > error (see resolve_intrinsics).
    parser.add_argument("--fx-px", type=float, default=None)
    parser.add_argument("--fy-px", type=float, default=None)
    parser.add_argument("--cx-px", type=float, default=None)
    parser.add_argument("--cy-px", type=float, default=None)
    parser.add_argument("--calibration-file", type=str, default=DEFAULT_CALIBRATION)
    parser.add_argument("--no-undistort", action="store_true", help="Skip lens distortion correction.")
    parser.add_argument(
        "--origin-lat", "--drone-lat", dest="origin_lat", type=float, default=0.0,
        help="Origin/drone latitude for NED->GPS (alias: --drone-lat).",
    )
    parser.add_argument(
        "--origin-lon", "--drone-lon", dest="origin_lon", type=float, default=0.0,
        help="Origin/drone longitude for NED->GPS (alias: --drone-lon).",
    )
    parser.add_argument(
        "--connect", type=str, default=None,
        help="MAVLink endpoint (e.g. udp:127.0.0.1:14554) for LIVE drone position. When set, "
        "--origin-lat/--origin-lon are treated as the datum/home position and the drone's live "
        "NED offset from it is added to every detection before GPS conversion, so the reported "
        "position tracks a moving drone instead of assuming it never left --origin-lat/--origin-lon. "
        "Omit to keep the previous static-origin behaviour (fine for a stationary test).",
    )
    parser.add_argument("--target-diameter-m", type=float, default=0.32)
    parser.add_argument("--kernel-size", type=int, default=5)
    parser.add_argument("--roi-margin", type=float, default=0.10)
    parser.add_argument("--min-circularity", type=float, default=0.35)
    parser.add_argument("--min-color-ratio", type=float, default=0.12)
    parser.add_argument("--track-gate-px", type=float, default=70.0)
    # P1: OFF/black buoy detection (Safe Passage, docs/10_safe_passage.md). OFF by default.
    parser.add_argument(
        "--detect-off-buoys", action="store_true",
        help="Also detect unlit OFF/BLACK Safe-Passage beacons via a dark-blob "
        "proposal + saturation gate (HSV path only). Adds a 'black' detection class.",
    )
    parser.add_argument("--off-sat-max", type=int, default=60, help="Max median saturation for an OFF buoy blob.")
    parser.add_argument("--off-val-max", type=int, default=80, help="Max median value for an OFF buoy blob.")
    parser.add_argument("--dark-block", type=int, default=51, help="adaptiveThreshold block size for the dark-blob channel (odd).")
    parser.add_argument("--dark-c", type=int, default=10, help="adaptiveThreshold C constant for the dark-blob channel.")
    # P2: flashing vs solid classification (Safe Passage). OFF by default.
    parser.add_argument(
        "--classify-flash", action="store_true",
        help="Classify each track as flashing/solid/unknown from its ON/OFF "
        "history and write it to the flash_state CSV column. Distinguishes a "
        "flashing-BLUE ENTRY beacon from a steady-BLUE EXIT beacon.",
    )
    parser.add_argument("--flash-window", type=int, default=60, help="Flash-state rolling window in FRAMES (~2-3 flash periods).")
    parser.add_argument("--flash-min-frames", type=int, default=30, help="Min observed frames before flash_state leaves 'unknown'.")
    parser.add_argument("--flash-min-toggles", type=int, default=2, help="Min ON/OFF toggles in the window to call a track 'flashing'.")
    parser.add_argument("--flash-solid-ratio", type=float, default=0.85, help="Min on-ratio (with <=1 toggle) to call a track 'solid'.")
    parser.add_argument(
        "--flash-max-missed", type=int, default=25,
        help="Track missed-frame budget when --classify-flash is on; must span a "
        "full ~1 s dark phase so a flashing track survives its OFF interval.",
    )
    parser.add_argument(
        "--yolo-model", type=str, default=None,
        help="Path to a YOLO .pt/.onnx model (see docs/08_annotation_and_training.md). When set, "
        "detection uses this model instead of the two-stage HSV pipeline.",
    )
    parser.add_argument("--yolo-conf", type=float, default=0.25, help="YOLO confidence threshold.")
    parser.add_argument(
        "--yolo-size-gate", action="store_true",
        help="Reject YOLO boxes whose diameter is outside "
        "[--yolo-size-tol-lo, --yolo-size-tol-hi] x expected_d "
        "(expected_d = fx*target_diameter_m/altitude_m). OFF by default: only "
        "valid when --altitude-m and --target-diameter-m match the real capture "
        "geometry, else it rejects genuine buoys (see docs/07_roadmap.md A1).",
    )
    parser.add_argument("--yolo-size-tol-lo", type=float, default=0.5, help="Lower diameter fraction for --yolo-size-gate.")
    parser.add_argument("--yolo-size-tol-hi", type=float, default=2.0, help="Upper diameter fraction for --yolo-size-gate.")
    parser.add_argument(
        "--yolo-min-circularity", type=float, default=0.0,
        help="If >0, reject YOLO boxes whose crop-blob circularity is below this "
        "(reject-only shape gate mirroring the HSV path). OFF (0.0) by default.",
    )
    parser.add_argument(
        "--save-video", action="store_true",
        help="Also write annotated frames to an mp4 in --log-dir (works headless too).",
    )
    parser.add_argument(
        "--gcs-ip", type=str, default=None,
        help="Ground station IP. When set, transmit confirmed detections as MAVLink STATUSTEXT "
        "buoy reports (see mavlink_comms/) to udpout:<ip>:14555.",
    )
    parser.add_argument("--max-track-missed", type=int, default=8)
    parser.add_argument("--log-dir", type=str, default="detection_logs")
    parser.add_argument("--calib-color", type=str, default="red", choices=["red", "green", "blue"])
    return parser.parse_args()


def find_detections(
    frame_full: np.ndarray,
    frame_det: np.ndarray,
    hsv_det: np.ndarray,
    hsv_full: np.ndarray,
    roi: tuple[int, int, int, int],
    args: argparse.Namespace,
    intr: Intrinsics,
) -> list[Detection]:
    detections: list[Detection] = []
    h_det, w_det = frame_det.shape[:2]
    x0, y0, x1, y1 = roi
    expected_d = intr.fx * args.target_diameter_m / max(args.altitude_m, 0.1)
    min_d = 0.5 * expected_d
    max_d = 2.0 * expected_d
    kernel = np.ones((args.kernel_size, args.kernel_size), np.uint8)
    scale_x = frame_full.shape[1] / float(w_det)
    scale_y = frame_full.shape[0] / float(h_det)

    # Stage 1: object detection (color-agnostic).
    gray = cv2.cvtColor(frame_det, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 40, 120)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    mask_roi = np.zeros_like(edges)
    mask_roi[y0:y1, x0:x1] = edges[y0:y1, x0:x1]
    contours, _ = cv2.findContours(mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 8:
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter <= 1e-6:
            continue
        circularity = (4.0 * np.pi * area) / (perimeter * perimeter)
        if circularity < args.min_circularity:
            continue

        (cx_det, cy_det), radius = cv2.minEnclosingCircle(cnt)
        diameter = 2.0 * radius
        if diameter < min_d or diameter > max_d:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        if w <= 0 or h <= 0:
            continue

        x_f = int(max(0, x * scale_x))
        y_f = int(max(0, y * scale_y))
        w_f = int(min(frame_full.shape[1] - x_f, w * scale_x))
        h_f = int(min(frame_full.shape[0] - y_f, h * scale_y))
        if w_f <= 0 or h_f <= 0:
            continue

        hsv_roi = hsv_full[y_f : y_f + h_f, x_f : x_f + w_f]
        roi_area = float(hsv_roi.shape[0] * hsv_roi.shape[1])
        if roi_area <= 0:
            continue

        # Stage 2: color thresholding/classification inside object ROI only.
        best_color = "unknown"
        best_ratio = 0.0
        best_moments = None
        for color, ranges in COLOR_RANGES.items():
            c_mask = build_mask(hsv_roi, ranges)
            c_mask = cv2.morphologyEx(c_mask, cv2.MORPH_OPEN, kernel)
            c_mask = cv2.morphologyEx(c_mask, cv2.MORPH_CLOSE, kernel)
            ratio = float(np.count_nonzero(c_mask)) / roi_area
            if ratio > best_ratio:
                best_ratio = ratio
                best_color = color
                best_moments = cv2.moments(c_mask)

        if best_ratio < args.min_color_ratio:
            continue

        if best_moments is not None and best_moments["m00"] > 0:
            cx_full = x_f + (best_moments["m10"] / best_moments["m00"])
            cy_full = y_f + (best_moments["m01"] / best_moments["m00"])
        else:
            cx_full = cx_det * scale_x
            cy_full = cy_det * scale_y

        size_term = max(0.0, 1.0 - abs(diameter - expected_d) / max(expected_d, 1e-6))
        conf = float(0.45 * size_term + 0.55 * best_ratio)
        detections.append(
            Detection(
                color=best_color,
                confidence=max(0.0, min(1.0, conf)),
                cx_full=float(cx_full),
                cy_full=float(cy_full),
                radius_det=radius,
                bbox_full=(x_f, y_f, w_f, h_f),
            )
        )
    return detections


def find_off_buoys(
    frame_full: np.ndarray,
    frame_det: np.ndarray,
    hsv_full: np.ndarray,
    roi: tuple[int, int, int, int],
    args: argparse.Namespace,
    intr: Intrinsics,
) -> list[Detection]:
    """P1: detect OFF/BLACK Safe-Passage beacons (see docs/10_safe_passage.md).

    An unlit beacon is a dark, low-contrast, colourless blob - it has no colour
    for the HSV path to threshold and often no Canny edge, so the normal
    proposal misses it. This adds a dark-blob proposal (adaptive threshold on
    the value channel) and classifies a candidate as ``black`` only when
    ``is_off_buoy`` confirms it is both dark and unsaturated. Enabled by
    ``--detect-off-buoys``; reject-only w.r.t. the colour path (adds only
    ``black`` detections, never touches red/green/blue).
    """
    detections: list[Detection] = []
    h_det, w_det = frame_det.shape[:2]
    x0, y0, x1, y1 = roi
    expected_d = intr.fx * args.target_diameter_m / max(args.altitude_m, 0.1)
    min_d = 0.5 * expected_d
    max_d = 2.0 * expected_d
    scale_x = frame_full.shape[1] / float(w_det)
    scale_y = frame_full.shape[0] / float(h_det)
    kernel = np.ones((args.kernel_size, args.kernel_size), np.uint8)

    gray = cv2.cvtColor(frame_det, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    # Adaptive (local-mean) threshold flags pixels darker than their
    # neighbourhood, so an unlit buoy pops against the surrounding water even
    # when its absolute brightness varies across the frame. Block size is large
    # vs. a buoy so the local mean is the water, not the buoy itself.
    block = args.dark_block if args.dark_block % 2 == 1 else args.dark_block + 1
    dark = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, block, args.dark_c
    )
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, kernel)
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, kernel)

    mask_roi = np.zeros_like(dark)
    mask_roi[y0:y1, x0:x1] = dark[y0:y1, x0:x1]
    contours, _ = cv2.findContours(mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 8:
            continue
        perimeter = cv2.arcLength(cnt, True)
        if perimeter <= 1e-6:
            continue
        circularity = (4.0 * np.pi * area) / (perimeter * perimeter)
        if circularity < args.min_circularity:
            continue
        (_, _), radius = cv2.minEnclosingCircle(cnt)
        diameter = 2.0 * radius
        if diameter < min_d or diameter > max_d:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        x_f = int(max(0, x * scale_x))
        y_f = int(max(0, y * scale_y))
        w_f = int(min(frame_full.shape[1] - x_f, w * scale_x))
        h_f = int(min(frame_full.shape[0] - y_f, h * scale_y))
        if w_f <= 0 or h_f <= 0:
            continue

        hsv_roi = hsv_full[y_f : y_f + h_f, x_f : x_f + w_f]
        if not is_off_buoy(hsv_roi, args.off_sat_max, args.off_val_max):
            continue

        # More grey (lower median saturation) -> more confident it is unlit.
        s_med = float(np.median(hsv_roi[:, :, 1]))
        conf = float(max(0.0, min(1.0, 1.0 - s_med / max(args.off_sat_max, 1))))
        detections.append(
            Detection(
                color="black",
                confidence=conf,
                cx_full=x_f + w_f / 2.0,
                cy_full=y_f + h_f / 2.0,
                radius_det=radius,
                bbox_full=(x_f, y_f, w_f, h_f),
            )
        )
    return detections


def load_yolo_model(model_path: str):
    """Load a YOLO .pt/.onnx model (Path 2 fine-tune; see
    docs/08_annotation_and_training.md). Lazily imported so the HSV-only path
    still runs without `ultralytics` installed.
    """
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            f"[ERROR] --yolo-model requires the `ultralytics` package "
            f"(pip install ultralytics). Import failed: {exc}"
        ) from exc
    print(f"[INFO] Loading YOLO model: {model_path}")
    return YOLO(model_path)


def yolo_box_circularity(frame_full: np.ndarray, bbox: tuple[int, int, int, int]) -> float:
    """Circularity (4*pi*area / perimeter^2) of the largest saturation blob
    inside a YOLO box - a reject-only shape metric for the YOLO path, scoped to
    the crop the model already found. Mirrors the contour-circularity test the
    HSV path (find_detections) runs, but on the box interior instead of a
    Canny-edge contour. Returns 0.0 when the crop has no usable blob; the caller
    only rejects on it when min_circularity > 0, so a 0.0 is harmless otherwise.
    """
    x, y, w, h = bbox
    if w < 3 or h < 3:
        return 0.0
    crop = frame_full[y : y + h, x : x + w]
    if crop.size == 0:
        return 0.0
    sat = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)[:, :, 1]
    _, mask = cv2.threshold(sat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return 0.0
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    perimeter = cv2.arcLength(c, True)
    if area <= 0 or perimeter <= 1e-6:
        return 0.0
    return float(4.0 * np.pi * area / (perimeter * perimeter))


def find_detections_yolo(
    frame_full: np.ndarray,
    model,
    conf_threshold: float,
    expected_d: float | None = None,
    size_tol: tuple[float, float] | None = None,
    min_circularity: float = 0.0,
) -> list[Detection]:
    """YOLO detection path (Path 2 fine-tune), same Detection shape as the HSV
    path so downstream tracking/CSV/GPS/MAVLink code is shared unchanged.
    Class-id -> color name comes from the model itself (0=red, 1=green,
    2=blue per 01_autolabel.py's training label convention).

    An earlier version of this function added agnostic_nms=True plus a
    crop-based HSV color-verification/correction pass, as a mitigation for a
    since-fixed model that hallucinated phantom red boxes on green objects.
    Measured directly against the honest val set: on the RETRAINED model
    (which genuinely learned red -- recall 1.0, precision 0.897 on real
    ground truth), both of those "fixes" made things worse (agnostic_nms
    dropped red recall to 0.40; the HSV correction dropped it to 0.0, by
    reclassifying genuine tight red boxes as green whenever the crop
    included enough background). Root-causing and retraining fixed the real
    problem; the raw model output is the baseline.

    Optional reject-only post-filters (both OFF unless the caller passes them;
    the CLI only does so for --yolo-size-gate / --yolo-min-circularity):
      * expected-size gate: drop a box whose diameter is outside
        [size_tol[0], size_tol[1]] * expected_d, where
        expected_d = fx * target_diameter_m / altitude_m -- the same formula the
        HSV path already uses in find_detections().
      * circularity gate: drop a box whose crop-blob circularity is below
        min_circularity (see yolo_box_circularity).
    Unlike the reverted 2024 experiment these never *reclassify* a box, only
    drop clear outliers. They default OFF because on this checkout's val set no
    size or shape threshold separates the false positives from the true ones,
    and the size gate at a realistic 10 m AGL would reject most genuine buoys
    (sim balloons render at 55-149 px vs a 42 px expected_d) -- measured, see
    tests/validation/test_yolo_size_gate.py and docs/07_roadmap.md. Turn them
    on only
    when --altitude-m / --target-diameter-m match the real capture geometry.
    """
    result = model(frame_full, conf=conf_threshold, verbose=False)[0]
    boxes = result.boxes
    detections: list[Detection] = []
    if boxes is None:
        return detections
    names = result.names
    size_gate = expected_d is not None and expected_d > 1e-6 and size_tol is not None
    for i in range(len(boxes)):
        conf = float(boxes.conf[i].item())
        cls_id = int(boxes.cls[i].item())
        color = str(names.get(cls_id, "unknown")).lower()
        if color not in COLOR_DRAW:
            color = "unknown"
        x1, y1, x2, y2 = boxes.xyxy[i].tolist()
        x = int(max(0, round(x1)))
        y = int(max(0, round(y1)))
        w = int(max(0, round(x2 - x1)))
        h = int(max(0, round(y2 - y1)))
        if w <= 0 or h <= 0:
            continue

        # Reject-only gates (no-ops unless explicitly enabled by the caller).
        diameter = float(max(w, h))
        if size_gate and not (size_tol[0] * expected_d <= diameter <= size_tol[1] * expected_d):
            continue
        if min_circularity > 0.0 and yolo_box_circularity(frame_full, (x, y, w, h)) < min_circularity:
            continue

        detections.append(
            Detection(
                color=color,
                confidence=conf,
                cx_full=x + w / 2.0,
                cy_full=y + h / 2.0,
                radius_det=max(w, h) / 2.0,
                bbox_full=(x, y, w, h),
            )
        )
    return detections


def update_tracks(
    tracks: list[Track],
    detections: list[Detection],
    gate_px: float,
    max_missed: int,
    next_track_id: int,
    flash_window: int = 60,
) -> tuple[list[Track], list[tuple[Detection, int, tuple[float, float]]], int]:
    assigned = []
    used_dets = set()

    for track in tracks:
        pred = track.kf.predict()
        px, py = float(pred[0, 0]), float(pred[1, 0])
        best_idx = None
        best_dist = float("inf")
        for i, det in enumerate(detections):
            if i in used_dets or det.color != track.color:
                continue
            dist = ((det.cx_full - px) ** 2 + (det.cy_full - py) ** 2) ** 0.5
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        if best_idx is not None and best_dist <= gate_px:
            det = detections[best_idx]
            meas = np.array([[det.cx_full], [det.cy_full]], np.float32)
            corr = track.kf.correct(meas)
            track.missed = 0
            track.flash.observe(True)  # P2: light seen this frame
            used_dets.add(best_idx)
            assigned.append((det, track.track_id, (float(corr[0, 0]), float(corr[1, 0]))))
        else:
            track.missed += 1
            track.flash.observe(False)  # P2: track alive but light dark this frame

    tracks = [t for t in tracks if t.missed <= max_missed]

    for i, det in enumerate(detections):
        if i in used_dets:
            continue
        kf = make_kalman(det.cx_full, det.cy_full)
        track = Track(
            track_id=next_track_id, color=det.color, kf=kf, missed=0,
            flash=TrackFlashState(flash_window),
        )
        track.flash.observe(True)
        tracks.append(track)
        assigned.append((det, track.track_id, (det.cx_full, det.cy_full)))
        next_track_id += 1
    return tracks, assigned, next_track_id


def apply_clahe_to_v(frame: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    v = clahe.apply(v)
    return cv2.cvtColor(cv2.merge((h, s, v)), cv2.COLOR_HSV2BGR)


def calibrate_sv_threshold(frame_det_bgr: np.ndarray, calib_color: str) -> None:
    hsv = cv2.cvtColor(frame_det_bgr, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]
    patch = hsv[h // 2 - 20 : h // 2 + 20, w // 2 - 20 : w // 2 + 20]
    if patch.size == 0:
        return
    s_med = int(np.median(patch[:, :, 1]))
    v_med = int(np.median(patch[:, :, 2]))
    updated = []
    for low, high in COLOR_RANGES[calib_color]:
        new_low = (low[0], max(0, min(low[1], s_med - 20)), max(0, min(low[2], v_med - 20)))
        updated.append((new_low, high))
    COLOR_RANGES[calib_color] = updated
    print(f"Calibrated {calib_color}: S_min={updated[0][0][1]}, V_min={updated[0][0][2]}")


def main() -> int:
    global COLOR_RANGES
    args = parse_args()
    COLOR_RANGES = load_color_ranges(classes_dir="captures/classes")
    os.makedirs(args.log_dir, exist_ok=True)
    csv_path = os.path.join(args.log_dir, "detections.csv")
    csv_exists = os.path.exists(csv_path)

    calib = load_calibration(args.calibration_file)
    intr = resolve_intrinsics(args, calib, args.width, args.height)
    print(
        f"[INFO] Active intrinsics [{intr.source}]: "
        f"fx={intr.fx:.2f} fy={intr.fy:.2f} cx={intr.cx:.2f} cy={intr.cy:.2f} "
        f"dist={np.round(intr.dist, 4).tolist()}"
    )

    file_mode = args.image_dir is not None or args.video_path is not None
    display = not args.no_display and not file_mode
    need_overlay = display or args.save_video

    yolo_model = load_yolo_model(args.yolo_model) if args.yolo_model else None

    transmitter = None
    if args.gcs_ip:
        try:
            from mavlink_comms.transmitter import BuoyMavlinkTransmitter
        except ImportError as exc:
            print(
                f"[ERROR] --gcs-ip requires mavlink_comms + vendored mavcore "
                f"(bash jetson_setup.sh clones it into vendor/mavcore). Import failed: {exc}"
            )
            return 1
        transmitter = BuoyMavlinkTransmitter(connection=f"udpout:{args.gcs_ip}:14555")
        print(f"[INFO] Transmitting buoy reports to udpout:{args.gcs_ip}:14555")

    telemetry = None
    if args.connect:
        try:
            from mavlink_telemetry import Telemetry
        except ImportError as exc:
            print(f"[ERROR] --connect requires pymavlink. Import failed: {exc}")
            return 1
        telemetry = Telemetry(args.connect)
        print(f"[INFO] Connecting live drone telemetry on {args.connect} ...")
        telemetry.start()
        print(f"[INFO] Telemetry connected. --origin-lat/--origin-lon "
              f"({args.origin_lat:.6f}, {args.origin_lon:.6f}) is now the datum; "
              f"each detection uses the drone's LIVE offset from it.")

    video_writer = None  # lazily opened on the first frame (actual frame size may differ from --width/--height)

    tracks = []
    next_track_id = 1
    # P2: a flashing track must outlive its ~1 s dark phase, so widen the missed
    # budget when flash classification is on (else keep the default behaviour).
    flash_max_missed = args.flash_max_missed if args.classify_flash else args.max_track_missed
    window_name = "Stage-A RGB Detection"
    if display:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    frame_count = 0
    detection_count = 0
    gen = frame_source(args)

    with open(csv_path, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if not csv_exists:
            writer.writerow(
                [
                    "timestamp", "image_path", "track_id", "color", "confidence",
                    "cx", "cy", "x", "y", "w", "h",
                    "north_m", "east_m", "lat", "lon", "altitude_m", "intrinsics_source",
                    "flash_state",
                ]
            )

        for frame_full, label in gen:
            frame_count += 1

            if yolo_model is not None:
                # Match training: 00_preprocess_training_data.py's default
                # (non---skip-normalize) path runs every training image
                # through color_normalize() (CLAHE-YUV -> gray-world WB ->
                # unsharp) before the model ever sees it (confirmed in
                # results_sim_courses_v2/README.md). Feeding it a raw frame
                # here would be a distribution shift vs. what it learned, so
                # normalize a copy for detection only; frame_full stays raw
                # for display/recording/GPS projection (geometry is unchanged).
                # Optional reject-only gates (default OFF -> identical to the
                # raw-model baseline). expected_d matches the HSV path's formula.
                expected_d_yolo = intr.fx * args.target_diameter_m / max(args.altitude_m, 0.1)
                size_tol = (args.yolo_size_tol_lo, args.yolo_size_tol_hi) if args.yolo_size_gate else None
                detections = find_detections_yolo(
                    color_normalize(frame_full),
                    yolo_model,
                    args.yolo_conf,
                    expected_d=expected_d_yolo if args.yolo_size_gate else None,
                    size_tol=size_tol,
                    min_circularity=args.yolo_min_circularity,
                )
            else:
                frame_full = apply_clahe_to_v(frame_full)
                frame_det = cv2.resize(frame_full, (args.det_width, args.det_height), interpolation=cv2.INTER_AREA)
                hsv_det = cv2.cvtColor(frame_det, cv2.COLOR_BGR2HSV)
                hsv_full = cv2.cvtColor(frame_full, cv2.COLOR_BGR2HSV)

                margin_x = int(args.roi_margin * args.det_width)
                margin_y = int(args.roi_margin * args.det_height)
                roi = (margin_x, margin_y, args.det_width - margin_x, args.det_height - margin_y)

                detections = find_detections(frame_full, frame_det, hsv_det, hsv_full, roi, args, intr)

                if args.detect_off_buoys:
                    # P1: add OFF/black buoys. A blob only becomes an off_det
                    # after is_off_buoy() confirms the ROI is dark AND
                    # unsaturated, which a genuinely lit colour buoy never is -
                    # so where the two overlap, the saturation-verified BLACK is
                    # the trustworthy call and the (weak) colour detection is
                    # dropped. This runs only under --detect-off-buoys, so the
                    # default pipeline is unchanged.
                    off_dets = find_off_buoys(frame_full, frame_det, hsv_full, roi, args, intr)
                    if off_dets:
                        def _same_object(cd: Detection, od: Detection) -> bool:
                            cx, cy, cw, ch = cd.bbox_full
                            ox, oy, ow, oh = od.bbox_full
                            ccx, ccy = cx + cw / 2.0, cy + ch / 2.0
                            ocx, ocy = ox + ow / 2.0, oy + oh / 2.0
                            return (ox <= ccx <= ox + ow and oy <= ccy <= oy + oh) or (
                                cx <= ocx <= cx + cw and cy <= ocy <= cy + ch
                            )

                        detections = [
                            cd for cd in detections if not any(_same_object(cd, od) for od in off_dets)
                        ] + off_dets

            tracks, assigned, next_track_id = update_tracks(
                tracks, detections, args.track_gate_px, flash_max_missed, next_track_id,
                flash_window=args.flash_window,
            )
            track_by_id = {t.track_id: t for t in tracks}

            frame_out = frame_full.copy() if need_overlay else None
            image_path = ""
            if assigned:
                if file_mode:
                    # Reference the original source frame instead of saving a copy.
                    image_path = label
                else:
                    ts = time.time()
                    image_path = os.path.join(args.log_dir, f"frame_{int(ts * 1000)}.jpg")
                    cv2.imwrite(image_path, frame_full)

            for det, track_id, (sx, sy) in assigned:
                detection_count += 1
                flash_state = ""
                if args.classify_flash:
                    trk = track_by_id.get(track_id)
                    if trk is not None:
                        flash_state = trk.flash.classify(
                            args.flash_min_frames, args.flash_min_toggles, args.flash_solid_ratio
                        )
                off_north_m, off_east_m = project_pixel_to_ground_ned(
                    sx, sy, intr, args.altitude_m, args.heading_deg
                )
                drone_north_m = drone_east_m = 0.0
                if telemetry is not None:
                    snap = telemetry.snapshot()
                    if snap["have_pose"]:
                        drone_north_m, drone_east_m = snap["north"], snap["east"]
                # With no --connect, drone_north_m/east_m are 0 and this is
                # identical to the old static-origin behaviour (origin_lat/lon
                # treated as the drone's one and only position). With
                # --connect, origin_lat/lon is the datum and this is the
                # drone's LIVE position plus the pixel offset.
                north_m, east_m = drone_north_m + off_north_m, drone_east_m + off_east_m
                lat, lon = ned_to_gps(north_m, east_m, args.origin_lat, args.origin_lon)
                x, y, w, h = det.bbox_full

                if frame_out is not None:
                    color_bgr = COLOR_DRAW[det.color]
                    cv2.rectangle(frame_out, (x, y), (x + w, y + h), color_bgr, 2)
                    cv2.circle(frame_out, (int(sx), int(sy)), 4, color_bgr, -1)
                    label_txt = f"{det.color} t{track_id} conf={det.confidence:.2f}"
                    cv2.putText(frame_out, label_txt, (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_bgr, 2)

                writer.writerow(
                    [
                        f"{time.time():.3f}",
                        image_path,
                        track_id,
                        det.color,
                        f"{det.confidence:.4f}",
                        f"{sx:.2f}",
                        f"{sy:.2f}",
                        x,
                        y,
                        w,
                        h,
                        f"{north_m:.3f}",
                        f"{east_m:.3f}",
                        f"{lat:.8f}",
                        f"{lon:.8f}",
                        f"{args.altitude_m:.2f}",
                        intr.source,
                        flash_state,
                    ]
                )
                src_tag = os.path.basename(label) if label else "cam"
                print(
                    f"[GPS] {src_tag} t{track_id} {det.color} conf={det.confidence:.2f} "
                    f"px=({sx:.0f},{sy:.0f}) NED=N{north_m:+.2f}m E{east_m:+.2f}m "
                    f"-> lat={lat:.7f} lon={lon:.7f}"
                )

                if transmitter is not None and det.color in ("red", "green", "blue"):
                    try:
                        transmitter.transmit(target_id=track_id, color=det.color, lat=lat, lon=lon, frame=frame_count)
                    except Exception as exc:  # noqa: BLE001 - a dropped report shouldn't kill the detection loop
                        print(f"[WARN] MAVLink TX failed: {exc}")
            f.flush()

            if args.save_video and frame_out is not None:
                if video_writer is None:
                    video_path = os.path.join(args.log_dir, f"session_{int(time.time())}.mp4")
                    h_out, w_out = frame_out.shape[:2]
                    video_writer = cv2.VideoWriter(
                        video_path, cv2.VideoWriter_fourcc(*"mp4v"), 15.0, (w_out, h_out)
                    )
                    print(f"[INFO] Saving video to {video_path}")
                video_writer.write(frame_out)

            if display:
                cv2.imshow(window_name, frame_out)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("c") and yolo_model is None:
                    calibrate_sv_threshold(frame_det, args.calib_color)

    gen.close()
    if display:
        cv2.destroyAllWindows()
    if video_writer is not None:
        video_writer.release()
    if transmitter is not None:
        transmitter.close()
    if telemetry is not None:
        telemetry.stop = True
    print(f"[INFO] Processed {frame_count} frame(s), {detection_count} detection(s). Log: {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
