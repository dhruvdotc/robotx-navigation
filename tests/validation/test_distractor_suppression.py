#!/usr/bin/env python3
"""Real-frame validation: run the HSV detector (the path the sim distractors
were designed to bait) on the 65 real held-out course frames, match detections
to the buoy GT (IoU>0.5), and report how many detections land OFF a buoy - i.e.
distractor / background false positives, and what colour they were called.
"""
import os
import sys
from types import SimpleNamespace

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS = os.path.join(REPO, "yolo_comparison_test/path2_switch_proposal/scripts")
sys.path.insert(0, REPO)
import camera_live_feed as clf  # noqa: E402
from color_utils import load_color_ranges  # noqa: E402

clf.COLOR_RANGES = load_color_ranges(
    classes_dir=os.path.join(SCRIPTS, os.pardir, "captures", "classes")
)

ARGS = SimpleNamespace(
    target_diameter_m=0.32, altitude_m=6.0, kernel_size=5, min_circularity=0.35,
    min_color_ratio=0.12, off_sat_max=60, off_val_max=80, dark_block=51, dark_c=10,
)
INTR = clf.Intrinsics(fx=1319.07, fy=1407.5, cx=960.0, cy=540.0, dist=np.zeros(5), K=np.eye(3), source="test")


def read_gt(lbl, w, h):
    out = []
    if not os.path.isfile(lbl):
        return out
    with open(lbl, encoding="utf-8") as f:
        for line in f:
            p = line.split()
            if len(p) != 5:
                continue
            cx, cy, bw, bh = (float(v) for v in p[1:])
            out.append(((cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h))
    return out


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def main():
    val_img = os.path.join(SCRIPTS, "dataset/images/val")
    val_lbl = os.path.join(SCRIPTS, "dataset/labels/val")
    names = sorted(n for n in os.listdir(val_img) if n.lower().endswith(".jpg"))
    TP = FP = 0
    fp_colors = {}
    for name in names:
        frame = cv2.imread(os.path.join(val_img, name))
        if frame is None:
            continue
        frame = clf.apply_clahe_to_v(frame)
        H, Wd = frame.shape[:2]
        dw, dh = 960, 540
        det = cv2.resize(frame, (dw, dh), interpolation=cv2.INTER_AREA)
        hsv_det = cv2.cvtColor(det, cv2.COLOR_BGR2HSV)
        hsv_full = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mx, my = int(0.1 * dw), int(0.1 * dh)
        roi = (mx, my, dw - mx, dh - my)
        dets = clf.find_detections(frame, det, hsv_det, hsv_full, roi, ARGS, INTR)
        gt = read_gt(os.path.join(val_lbl, os.path.splitext(name)[0] + ".txt"), Wd, H)
        used = set()
        for d in dets:
            x, y, w, h = d.bbox_full
            box = (x, y, x + w, y + h)
            best, bg = 0.0, -1
            for gi, g in enumerate(gt):
                if gi in used:
                    continue
                ov = iou(box, g)
                if ov > best:
                    best, bg = ov, gi
            if best > 0.5:
                TP += 1
                used.add(bg)
            else:
                FP += 1
                fp_colors[d.color] = fp_colors.get(d.color, 0) + 1
    prec = TP / (TP + FP) if (TP + FP) else 0.0
    print(f"HSV detector on {len(names)} real course frames (with distractors present):")
    print(f"  on-buoy (TP)={TP}  off-buoy/distractor (FP)={FP}  precision={prec:.3f}")
    print(f"  FP colours: {fp_colors or '(none)'}")


if __name__ == "__main__":
    main()
