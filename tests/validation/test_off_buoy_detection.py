#!/usr/bin/env python3
"""Validate OFF/black Safe-Passage buoy detection: drive find_off_buoys() on
controlled frames, isolating the OFF/black detector from the (unrelated)
colour-classification path so the result is unambiguous.
"""
import os
import sys
from types import SimpleNamespace

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
from camera_live_feed import Intrinsics, find_off_buoys  # noqa: E402

ARGS = SimpleNamespace(
    kernel_size=5, min_circularity=0.35, target_diameter_m=0.32, altitude_m=10.0,
    off_sat_max=60, off_val_max=80, dark_block=51, dark_c=10,
)
INTR = Intrinsics(fx=1319.07, fy=1407.5, cx=480.0, cy=270.0,
                  dist=np.zeros(5), K=np.eye(3), source="test")


def run(frame):
    det = cv2.resize(frame, (960, 540), interpolation=cv2.INTER_AREA)
    hsv_full = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = det.shape[:2]
    roi = (int(0.1 * w), int(0.1 * h), int(0.9 * w), int(0.9 * h))
    return find_off_buoys(frame, det, hsv_full, roi, ARGS, INTR)


def water(w=960, h=540):
    f = np.full((h, w, 3), (110, 105, 70), np.uint8)
    n = np.random.default_rng(1).integers(-8, 8, (h, w, 3), dtype=np.int16)
    return np.clip(f.astype(np.int16) + n, 0, 255).astype(np.uint8)


def main():
    ok = True

    # 1. dark OFF blob on water -> exactly one black det near the blob centre
    f = water()
    cv2.circle(f, (430, 250), 26, (40, 42, 38), -1)  # dark grey, r=26 -> d=52 in [21,84]
    dets = run(f)
    hit = [d for d in dets if abs(d.cx_full - 430) < 25 and abs(d.cy_full - 250) < 25]
    print(f"[1] dark OFF blob      : {len(dets)} black det(s); near-blob={len(hit)} "
          f"{'PASS' if len(hit) == 1 else 'FAIL'}")
    ok &= len(hit) == 1

    # 2. dark BLUE lit blob (saturated) -> must NOT be called black
    f = water()
    bgr = cv2.cvtColor(np.uint8([[(114, 220, 60)]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()
    cv2.circle(f, (430, 250), 26, bgr, -1)
    dets = run(f)
    hit = [d for d in dets if abs(d.cx_full - 430) < 25 and abs(d.cy_full - 250) < 25]
    print(f"[2] dark BLUE lit blob : {len(hit)} black det(s) at blob "
          f"{'PASS (none)' if len(hit) == 0 else 'FAIL (misread as black)'}")
    ok &= len(hit) == 0

    # 3. plain water -> no black false positives
    dets = run(water())
    print(f"[3] plain water        : {len(dets)} black det(s) {'PASS' if len(dets) == 0 else 'WARN(FP)'}")
    ok &= len(dets) == 0

    print("RESULT:", "ALL PASS" if ok else "FAILURES ABOVE")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
