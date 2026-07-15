#!/usr/bin/env python3
"""Generate a presentation-ready Stage-A HSV results diagram.

Stats are read dynamically from captures/hsv_results/detections.csv (same
source metrics_summary.py uses) rather than hard-coded from a specific run.
"""

from __future__ import annotations

import argparse
import csv
import os

import cv2
import numpy as np


W, H = 1400, 900

PIPELINE_NAME = "Stage-A HSV Detection Pipeline"
COURSE_LINE = "RobotX 2026  |  UCSD"

CLASS_COLORS = {
    "red": (60, 60, 220),
    "green": (60, 180, 60),
    "blue": (200, 100, 0),
}


def load_stats(csv_path: str, captures_dir: str) -> dict:
    """Compute all diagram stats from detections.csv (mirrors metrics_summary.py)."""
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(
            f"Detections CSV not found: {csv_path}. Run hsv_batch_detect.py first."
        )

    with open(csv_path, "r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    classes = ["red", "green", "blue"]
    by_class: dict[str, list[float]] = {c: [] for c in classes}
    per_image_counts: dict[str, int] = {}
    low_conf = 0

    for row in rows:
        image = row.get("image", "")
        color = row.get("color", "").lower()
        conf = float(row.get("confidence") or 0.0)
        if image:
            per_image_counts[image] = per_image_counts.get(image, 0) + 1
        if color in by_class:
            by_class[color].append(conf)
        if conf < 0.25:
            low_conf += 1

    image_files = [
        name
        for name in sorted(os.listdir(captures_dir))
        if os.path.isfile(os.path.join(captures_dir, name)) and name.lower().endswith(".jpg")
    ] if os.path.isdir(captures_dir) else []
    zero_detection_images = [n for n in image_files if per_image_counts.get(n, 0) == 0]

    class_stats = {}
    for cls in classes:
        vals = by_class[cls]
        class_stats[cls] = {
            "count": len(vals),
            "avg": float(np.mean(vals)) if vals else 0.0,
            "min": float(np.min(vals)) if vals else 0.0,
            "max": float(np.max(vals)) if vals else 0.0,
            "color": CLASS_COLORS[cls],
        }

    overall_avg_conf = (
        float(np.mean([float(r.get("confidence") or 0.0) for r in rows])) if rows else 0.0
    )

    return {
        "images_processed": len(image_files) if image_files else len(per_image_counts),
        "total_detections": len(rows),
        "false_positives": low_conf,
        "zero_detection_images": len(zero_detection_images),
        "overall_avg_conf": overall_avg_conf,
        "class_stats": class_stats,
    }


def _text(
    img: np.ndarray,
    s: str,
    org: tuple[int, int],
    scale: float = 0.7,
    color: tuple[int, int, int] = (25, 25, 25),
    thickness: int = 2,
) -> None:
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def _center_x(text: str, x1: int, x2: int, scale: float, thickness: int) -> int:
    (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    return x1 + ((x2 - x1 - tw) // 2)


def draw_title_bar(img: np.ndarray) -> None:
    cv2.rectangle(img, (0, 0), (W - 1, 79), (30, 30, 60), -1)
    _text(img, PIPELINE_NAME, (24, 50), scale=1.0, color=(255, 255, 255), thickness=2)
    (tw, _), _ = cv2.getTextSize(COURSE_LINE, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    _text(img, COURSE_LINE, (W - tw - 24, 50), scale=0.6, color=(210, 210, 210), thickness=1)


def draw_top_panels(img: np.ndarray, class_stats: dict, total_detections: int) -> None:
    y1, y2 = 80, 400
    x_a1, x_a2 = 0, W // 3
    x_b1, x_b2 = W // 3, (2 * W) // 3
    x_c1, x_c2 = (2 * W) // 3, W

    panel_bg = (245, 245, 245)
    border = (70, 70, 70)
    cv2.rectangle(img, (x_a1, y1), (x_a2 - 1, y2 - 1), panel_bg, -1)
    cv2.rectangle(img, (x_b1, y1), (x_b2 - 1, y2 - 1), panel_bg, -1)
    cv2.rectangle(img, (x_c1, y1), (x_c2 - 1, y2 - 1), panel_bg, -1)
    cv2.rectangle(img, (0, y1), (W - 1, y2 - 1), border, 1)
    cv2.line(img, (x_a2, y1), (x_a2, y2), border, 1)
    cv2.line(img, (x_b2, y1), (x_b2, y2), border, 1)

    # Panel A: Detection Counts
    title = "Detections per Class"
    _text(img, title, (_center_x(title, x_a1, x_a2, 0.8, 2), y1 + 36), scale=0.8, thickness=2)
    rows = [("red", 160), ("green", 245), ("blue", 330)]
    for cls, y in rows:
        d = class_stats[cls]
        cv2.circle(img, (x_a1 + 55, y - 8), 18, d["color"], -1)
        _text(img, cls.upper(), (x_a1 + 88, y), scale=0.8, thickness=2)
        _text(img, str(d["count"]), (x_a1 + 250, y + 8), scale=1.4, thickness=3)
        _text(img, f"avg conf: {d['avg']:.3f}", (x_a1 + 90, y + 28), scale=0.52, thickness=1, color=(80, 80, 80))
    _text(img, f"Total: {total_detections}", (x_a1 + 120, y2 - 24), scale=0.95, thickness=2)

    # Panel B: Bar Chart per class
    title = "Bar Chart per Class"
    _text(img, title, (_center_x(title, x_b1, x_b2, 0.8, 2), y1 + 36), scale=0.8, thickness=2)
    chart_left = x_b1 + 70
    chart_right = x_b2 - 40
    bar_max_w = chart_right - chart_left
    max_count = max(1, max(v["count"] for v in class_stats.values()))
    for i, cls in enumerate(("red", "green", "blue")):
        d = class_stats[cls]
        y = y1 + 95 + i * 74
        cv2.putText(img, cls, (x_b1 + 20, y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 2, cv2.LINE_AA)
        w = int((d["count"] / max_count) * bar_max_w)
        cv2.rectangle(img, (chart_left, y), (chart_left + w, y + 36), d["color"], -1)
        cv2.rectangle(img, (chart_left, y), (chart_left + bar_max_w, y + 36), (120, 120, 120), 1)
        _text(img, str(d["count"]), (chart_left + w + 10, y + 26), scale=0.65, thickness=2)

    # Panel C: Confidence gauges
    title = "Confidence Ranges"
    _text(img, title, (_center_x(title, x_c1, x_c2, 0.8, 2), y1 + 36), scale=0.8, thickness=2)
    gx1 = x_c1 + 100
    gx2 = x_c2 - 35
    gw = gx2 - gx1
    for i, cls in enumerate(("red", "green", "blue")):
        d = class_stats[cls]
        y = y1 + 108 + i * 84
        _text(img, cls, (x_c1 + 20, y + 8), scale=0.68, thickness=2)
        cv2.rectangle(img, (gx1, y), (gx2, y + 22), (230, 230, 230), -1)
        cv2.rectangle(img, (gx1, y), (gx2, y + 22), (110, 110, 110), 1)
        x_min = gx1 + int(d["min"] * gw)
        x_max = gx1 + int(d["max"] * gw)
        x_avg = gx1 + int(d["avg"] * gw)
        cv2.rectangle(img, (x_min, y + 4), (x_max, y + 18), d["color"], -1)
        cv2.line(img, (x_avg, y - 4), (x_avg, y + 26), (20, 20, 20), 2)
        _text(img, f"min {d['min']:.3f}", (gx1, y + 44), scale=0.5, thickness=1, color=(80, 80, 80))
        _text(img, f"avg {d['avg']:.3f}", (gx1 + 130, y + 44), scale=0.5, thickness=1, color=(30, 30, 30))
        _text(img, f"max {d['max']:.3f}", (gx2 - 100, y + 44), scale=0.5, thickness=1, color=(80, 80, 80))


def draw_before_after(img: np.ndarray, after_count: int, before_count: int | None) -> None:
    """Before/after ROI comparison. `before_count` requires a separate run of
    the detector with ROI filtering disabled - detections.csv only ever holds
    final, post-filter rows, so it cannot be derived from that CSV alone.
    Pass --before-roi to show the comparison; otherwise this panel just shows
    the current (after-filter) count.
    """
    y1, y2 = 400, 760
    cv2.rectangle(img, (0, y1), (W - 1, y2 - 1), (255, 255, 255), -1)
    cv2.rectangle(img, (0, y1), (W - 1, y2 - 1), (70, 70, 70), 1)

    if before_count is None:
        title = "AFTER ROI Crop + Shape Gates"
        _text(img, title, (_center_x(title, 0, W, 0.9, 2), y1 + 36), scale=0.9, thickness=2)
        rx1, rx2 = 460, 940
        ry1, ry2 = y1 + 70, y2 - 45
        cv2.rectangle(img, (rx1, ry1), (rx2, ry2), (235, 255, 235), -1)
        cv2.rectangle(img, (rx1, ry1), (rx2, ry2), (100, 100, 100), 2)
        _text(img, "After ROI fix", (rx1 + 20, ry1 + 38), scale=0.9, thickness=2, color=(30, 110, 30))
        _text(img, f"{after_count}", (rx1 + 175, ry1 + 145), scale=2.0, thickness=4, color=(20, 130, 20))
        _text(img, "detections", (rx1 + 155, ry1 + 188), scale=0.8, thickness=2, color=(30, 110, 30))
        note = "Pass --before-roi <count> (from an unfiltered run) to show the before/after comparison."
        _text(img, note, (_center_x(note, 0, W, 0.5, 1), y2 - 18), scale=0.5, thickness=1, color=(120, 120, 120))
        return

    title = "BEFORE vs AFTER ROI Crop + Shape Gates"
    _text(img, title, (_center_x(title, 0, W, 0.9, 2), y1 + 36), scale=0.9, thickness=2)

    # Left: before
    lx1, lx2 = 80, 640
    ly1, ly2 = y1 + 70, y2 - 45
    cv2.rectangle(img, (lx1, ly1), (lx2, ly2), (240, 240, 255), -1)
    cv2.rectangle(img, (lx1, ly1), (lx2, ly2), (100, 100, 100), 2)
    _text(img, "Before ROI fix", (lx1 + 20, ly1 + 38), scale=0.9, thickness=2, color=(40, 40, 120))
    _text(img, f"{before_count}", (lx1 + 180, ly1 + 145), scale=2.0, thickness=4, color=(20, 20, 160))
    _text(img, "detections", (lx1 + 185, ly1 + 188), scale=0.8, thickness=2, color=(40, 40, 120))

    # Right: after
    rx1, rx2 = 760, 1320
    ry1, ry2 = ly1, ly2
    cv2.rectangle(img, (rx1, ry1), (rx2, ry2), (235, 255, 235), -1)
    cv2.rectangle(img, (rx1, ry1), (rx2, ry2), (100, 100, 100), 2)
    _text(img, "After ROI fix", (rx1 + 20, ry1 + 38), scale=0.9, thickness=2, color=(30, 110, 30))
    _text(img, f"{after_count}", (rx1 + 205, ry1 + 145), scale=2.0, thickness=4, color=(20, 130, 20))
    _text(img, "detections", (rx1 + 185, ry1 + 188), scale=0.8, thickness=2, color=(30, 110, 30))

    # Delta / improvement
    delta = before_count - after_count
    reduction = (delta / before_count) * 100.0 if before_count > 0 else 0.0
    arrow_y = (ly1 + ly2) // 2
    cv2.arrowedLine(img, (655, arrow_y), (745, arrow_y), (60, 60, 60), 5, tipLength=0.2)
    _text(img, f"-{delta} detections", (625, arrow_y - 30), scale=0.62, thickness=2)
    _text(img, f"{reduction:.1f}% reduction", (620, arrow_y + 45), scale=0.62, thickness=2, color=(20, 120, 20))


def draw_summary_banner(img: np.ndarray, stats: dict) -> None:
    y1, y2 = 760, 900
    cv2.rectangle(img, (0, y1), (W - 1, y2 - 1), (80, 165, 90), -1)
    cv2.rectangle(img, (0, y1), (W - 1, y2 - 1), (50, 90, 55), 1)
    _text(img, "SUMMARY", (24, y1 + 40), scale=0.9, color=(255, 255, 255), thickness=2)
    cs = stats["class_stats"]
    line1 = (
        f"Images: {stats['images_processed']}   |   Total detections: {stats['total_detections']}   |   "
        f"Overall avg confidence: {stats['overall_avg_conf']:.3f}"
    )
    line2 = (
        f"False positives (conf < 0.25): {stats['false_positives']}   |   "
        f"Images with zero detections: {stats['zero_detection_images']}"
    )
    line3 = f"Class counts: red={cs['red']['count']}, green={cs['green']['count']}, blue={cs['blue']['count']}"
    _text(img, line1, (220, y1 + 35), scale=0.65, color=(245, 255, 245), thickness=2)
    _text(img, line2, (220, y1 + 72), scale=0.65, color=(245, 255, 245), thickness=2)
    _text(img, line3, (220, y1 + 109), scale=0.65, color=(245, 255, 245), thickness=2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv-path", default=os.path.join("captures", "hsv_results", "detections.csv"))
    parser.add_argument("--captures-dir", default="captures")
    parser.add_argument(
        "--before-roi", type=int, default=None,
        help="Raw candidate detection count from a run with ROI/shape filtering disabled, "
             "for the before/after comparison panel. Not derivable from detections.csv alone.",
    )
    args = parser.parse_args()

    stats = load_stats(args.csv_path, args.captures_dir)

    out_dir = os.path.join("captures", "hsv_results")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "results_diagram.png")

    canvas = np.full((H, W, 3), 255, dtype=np.uint8)
    draw_title_bar(canvas)
    draw_top_panels(canvas, stats["class_stats"], stats["total_detections"])
    draw_before_after(canvas, stats["total_detections"], args.before_roi)
    draw_summary_banner(canvas, stats)
    cv2.imwrite(out_path, canvas)
    print(f"Saved: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
