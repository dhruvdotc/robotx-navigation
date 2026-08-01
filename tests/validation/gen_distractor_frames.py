#!/usr/bin/env python3
"""Synthetic frame of the sim's DISTRACTOR obstacles at their real material
colours (from simulation/gazebo/worlds/robotx_uav_course.sdf) plus the borderline
worst-cases the README flags, on water, at buoy size. The detector must NOT call
any of them a navigation-buoy colour (red/green/blue) or an OFF buoy (black).
"""
import os

import cv2
import numpy as np

W, H, R = 960, 540, 26


def water(seed=3):
    f = np.full((H, W, 3), (110, 105, 70), np.uint8)
    n = np.random.default_rng(seed).integers(-8, 8, (H, W, 3), dtype=np.int16)
    return np.clip(f.astype(np.int16) + n, 0, 255).astype(np.uint8)


def main():
    out = "/tmp/distractor_frames"
    os.makedirs(out, exist_ok=True)
    f = water()
    # SDF diffuse colours (linear RGB -> BGR*255), the surface colour the sensor sees:
    olive = (46, 102, 36)       # RGB 0.18,0.40,0.14  -> hue ~55 (below green 75-105)
    brown = (31, 82, 140)       # RGB 0.55,0.32,0.12  -> hue ~14 (above red 0-10)
    gray_panel = (128, 128, 128)
    gray_barrel = (97, 97, 97)
    cv2.circle(f, (120, 150), R, olive, -1)
    cv2.circle(f, (330, 150), R, brown, -1)
    cv2.circle(f, (540, 150), R, gray_panel, -1)
    cv2.circle(f, (750, 150), R, gray_barrel, -1)
    # borderline worst-cases (README: olive can bleed toward green, orange toward red)
    cv2.circle(f, (250, 380), R, cv2.cvtColor(np.uint8([[(72, 150, 130)]]), cv2.COLOR_HSV2BGR)[0, 0].tolist(), -1)  # olive hue 72, just under green 75
    cv2.circle(f, (520, 380), R, cv2.cvtColor(np.uint8([[(12, 200, 150)]]), cv2.COLOR_HSV2BGR)[0, 0].tolist(), -1)  # orange hue 12, just over red 10
    for i in range(3):
        cv2.imwrite(os.path.join(out, f"frame_{i:03d}.jpg"), f)
    print(f"wrote 3 distractor frames to {out}: olive, brown, gray-panel, gray-barrel + 2 borderline")


if __name__ == "__main__":
    main()
