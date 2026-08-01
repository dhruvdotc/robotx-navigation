#!/usr/bin/env python3
"""Diagnostic: is there a size/shape gate that rejects the YOLO false
positives on the val set WITHOUT touching any true positive?

Runs the *current* (raw) find_detections_yolo() from camera_live_feed.py on the
held-out val set, greedy-matches to GT (IoU>0.5, same as validation_step3), and
prints the per-detection diameter + crop shape metrics split by TP vs FP so a
gate threshold can be chosen from data, not guessed. Backs the decision to
ship the optional --yolo-size-gate / --yolo-min-circularity flags OFF by
default - see docs/07_roadmap.md.
"""
from __future__ import annotations

import os
import sys

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS = os.path.join(REPO, "yolo_comparison_test/path2_switch_proposal/scripts")
sys.path.insert(0, REPO)

from camera_live_feed import find_detections_yolo, load_yolo_model  # noqa: E402

CLS_OF = {"red": 0, "green": 1, "blue": 2}


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def read_gt(lbl_path, w, h):
    boxes = []
    if not os.path.isfile(lbl_path):
        return boxes
    with open(lbl_path, encoding="utf-8") as f:
        for line in f:
            p = line.split()
            if len(p) != 5:
                continue
            cls = int(p[0])
            cx, cy, bw, bh = (float(v) for v in p[1:])
            boxes.append((cls, (cx - bw / 2) * w, (cy - bh / 2) * h,
                          (cx + bw / 2) * w, (cy + bh / 2) * h))
    return boxes


def crop_shape_metrics(frame, bbox):
    """circularity, solidity of the largest saturation blob inside the box."""
    x, y, w, h = bbox
    crop = frame[y:y + h, x:x + w]
    if crop.size == 0 or w < 3 or h < 3:
        return 0.0, 0.0
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    s = hsv[:, :, 1]
    _, th = cv2.threshold(s, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return 0.0, 0.0
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    per = cv2.arcLength(c, True)
    if area <= 0 or per <= 1e-6:
        return 0.0, 0.0
    circ = 4.0 * np.pi * area / (per * per)
    hull = cv2.convexHull(c)
    ha = cv2.contourArea(hull)
    sol = area / ha if ha > 0 else 0.0
    return circ, sol


def main():
    model = load_yolo_model(os.path.join(SCRIPTS, "training/balloon_proper/weights/best.pt"))
    val_img = os.path.join(SCRIPTS, "dataset/images/val")
    val_lbl = os.path.join(SCRIPTS, "dataset/labels/val")
    names = sorted(n for n in os.listdir(val_img) if n.lower().endswith(".jpg"))

    rows = []  # (class, is_tp, diameter, circ, sol)
    for name in names:
        img = cv2.imread(os.path.join(val_img, name))
        if img is None:
            continue
        h, w = img.shape[:2]
        gt = read_gt(os.path.join(val_lbl, os.path.splitext(name)[0] + ".txt"), w, h)
        dets = find_detections_yolo(img, model, 0.25)
        # greedy match per class
        used_gt = set()
        pred_boxes = []
        for d in dets:
            bx = d.bbox_full
            pred_boxes.append((CLS_OF.get(d.color, -1), bx[0], bx[1], bx[0] + bx[2], bx[1] + bx[3], d))
        # sort by descending best IoU handled simply: match greedily
        matches = []
        for pi, pb in enumerate(pred_boxes):
            for gi, gb in enumerate(gt):
                if gb[0] != pb[0]:
                    continue
                ov = iou((pb[1], pb[2], pb[3], pb[4]), (gb[1], gb[2], gb[3], gb[4]))
                if ov > 0.5:
                    matches.append((ov, pi, gi))
        matches.sort(reverse=True)
        used_p, used_g = set(), set()
        for ov, pi, gi in matches:
            if pi in used_p or gi in used_g:
                continue
            used_p.add(pi)
            used_g.add(gi)
        for pi, pb in enumerate(pred_boxes):
            d = pb[5]
            x, y, bw, bh = d.bbox_full
            diameter = max(bw, bh)
            circ, sol = crop_shape_metrics(img, d.bbox_full)
            rows.append((d.color, pi in used_p, diameter, circ, sol))

    # summarize
    def summ(vals):
        if not vals:
            return "n=0"
        a = np.array(vals)
        return f"n={len(a)} min={a.min():.1f} p10={np.percentile(a,10):.1f} med={np.median(a):.1f} p90={np.percentile(a,90):.1f} max={a.max():.1f}"

    print("\n===== YOLO GATE SEPARABILITY DIAGNOSTIC =====")
    for metric_i, metric_name in [(2, "DIAMETER(px)"), (3, "CIRCULARITY"), (4, "SOLIDITY")]:
        print(f"\n--- {metric_name} ---")
        tp_vals = [r[metric_i] for r in rows if r[1]]
        fp_vals = [r[metric_i] for r in rows if not r[1]]
        print(f"  TP: {summ(tp_vals)}")
        print(f"  FP: {summ(fp_vals)}")
        for col in ("red", "green", "blue"):
            tv = [r[metric_i] for r in rows if r[1] and r[0] == col]
            fv = [r[metric_i] for r in rows if not r[1] and r[0] == col]
            print(f"    {col:5s} TP {summ(tv)} | FP {summ(fv)}")

    # expected_d for a few altitudes (fx from calibration ~1319, target 0.32m)
    fx = 1319.071398
    print("\n--- expected_d = fx*0.32/alt (fx=1319) ---")
    for alt in (6, 8, 10, 12, 15):
        print(f"  alt={alt:2d}m -> expected_d={fx*0.32/alt:.1f}px  [0.5x={0.5*fx*0.32/alt:.1f}, 2.0x={2.0*fx*0.32/alt:.1f}]")

    print(f"\nTotal dets: {len(rows)}  TP={sum(1 for r in rows if r[1])}  FP={sum(1 for r in rows if not r[1])}")
    # Full FP dump for inspection
    print("\n--- individual FPs (color, diameter, circ, sol) ---")
    for r in rows:
        if not r[1]:
            print(f"  {r[0]:5s} d={r[2]:.0f} circ={r[3]:.2f} sol={r[4]:.2f}")


if __name__ == "__main__":
    main()
