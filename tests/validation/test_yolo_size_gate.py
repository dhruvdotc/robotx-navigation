#!/usr/bin/env python3
"""Validate the optional YOLO size/circularity gates: run find_detections_yolo()
on the held-out val set in each gate mode and report P/R/F1 using the SAME
greedy IoU match (>0.5, class-agnostic overall) as validation_step3_val_inference.py,
so the numbers are directly comparable to the fresh baseline.

Modes:
  off    - default (both gates disabled): must reproduce the baseline exactly.
  size10 - size gate on, expected_d for 10 m AGL, tol 0.5x-2.0x (HSV defaults).
  circ50 - circularity gate at 0.5.
"""
from __future__ import annotations

import os
import sys

import cv2

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS = os.path.join(REPO, "yolo_comparison_test/path2_switch_proposal/scripts")
sys.path.insert(0, REPO)

from camera_live_feed import find_detections_yolo, load_yolo_model  # noqa: E402

CLS_OF = {"red": 0, "green": 1, "blue": 2}
FX = 1319.071398  # calibration fx, matches camera_live_feed LEGACY/calib


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
    out = []
    if not os.path.isfile(lbl_path):
        return out
    with open(lbl_path, encoding="utf-8") as f:
        for line in f:
            p = line.split()
            if len(p) != 5:
                continue
            cx, cy, bw, bh = (float(v) for v in p[1:])
            out.append(((cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h))
    return out


def greedy(preds, gts):
    """class-agnostic overall match, mirrors validation_step3 greedy_match(None)."""
    matches = []
    for pi, pb in enumerate(preds):
        for gi, gb in enumerate(gts):
            ov = iou(pb, gb)
            if ov > 0.5:
                matches.append((ov, pi, gi))
    matches.sort(reverse=True)
    up, ug = set(), set()
    for _, pi, gi in matches:
        if pi in up or gi in ug:
            continue
        up.add(pi)
        ug.add(gi)
    tp = len(up)
    return tp, len(preds) - tp, len(gts) - tp


def prf(tp, fp, fn):
    p = tp / (tp + fp) if (tp + fp) else 0.0
    r = tp / (tp + fn) if (tp + fn) else 0.0
    f = 2 * p * r / (p + r) if (p + r) else 0.0
    return p, r, f


def run(model, names, val_img, val_lbl, mode):
    TP = FP = FN = 0
    for name in names:
        img = cv2.imread(os.path.join(val_img, name))
        if img is None:
            continue
        h, w = img.shape[:2]
        gt = read_gt(os.path.join(val_lbl, os.path.splitext(name)[0] + ".txt"), w, h)
        if mode == "off":
            dets = find_detections_yolo(img, model, 0.25)
        elif mode == "size10":
            dets = find_detections_yolo(img, model, 0.25, expected_d=FX * 0.32 / 10.0, size_tol=(0.5, 2.0))
        elif mode == "circ50":
            dets = find_detections_yolo(img, model, 0.25, min_circularity=0.5)
        else:
            raise ValueError(mode)
        preds = [(d.bbox_full[0], d.bbox_full[1], d.bbox_full[0] + d.bbox_full[2], d.bbox_full[1] + d.bbox_full[3]) for d in dets]
        tp, fp, fn = greedy(preds, gt)
        TP += tp
        FP += fp
        FN += fn
    p, r, f = prf(TP, FP, FN)
    return TP, FP, FN, p, r, f


def main():
    model = load_yolo_model(os.path.join(SCRIPTS, "training/balloon_proper/weights/best.pt"))
    val_img = os.path.join(SCRIPTS, "dataset/images/val")
    val_lbl = os.path.join(SCRIPTS, "dataset/labels/val")
    names = sorted(n for n in os.listdir(val_img) if n.lower().endswith(".jpg"))
    print("\nmode     TP  FP  FN   Prec   Recall   F1")
    for mode in ("off", "size10", "circ50"):
        TP, FP, FN, p, r, f = run(model, names, val_img, val_lbl, mode)
        print(f"{mode:7s} {TP:3d} {FP:3d} {FN:3d}  {p:.3f}  {r:.3f}   {f:.3f}")


if __name__ == "__main__":
    main()
