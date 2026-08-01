#!/usr/bin/env python3
"""Generate a synthetic nadir Safe-Passage test frame for OFF/black buoy
detection. Blobs use the sensor-real HSV values documented in
simulation/light_buoy_cycler.py so the test exercises realistic inputs:
  red   hue ~0    (lit)
  green hue ~86   (lit)
  blue  hue ~114  (lit, deliberately DARK val -> must classify blue, NOT black)
  OFF   dark grey (unlit -> must classify black)
on blue-green water (grayscale ~103).
"""
import os

import cv2
import numpy as np

W, H = 960, 540
R = 26


def hsv_blob(img, cx, cy, hsv):
    bgr = cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()
    cv2.circle(img, (cx, cy), R, bgr, -1)


def main():
    out_dir = "/tmp/off_buoy_frames"
    os.makedirs(out_dir, exist_ok=True)
    # blue-green water, grayscale ~103
    frame = np.full((H, W, 3), (110, 105, 70), np.uint8)
    # slight texture so CLAHE/threshold have something realistic to chew on
    noise = np.random.default_rng(0).integers(-8, 8, (H, W, 3), dtype=np.int16)
    frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    hsv_blob(frame, 160, 150, (0, 230, 210))     # red  lit
    hsv_blob(frame, 430, 150, (86, 220, 200))    # green lit
    hsv_blob(frame, 700, 150, (114, 220, 60))    # blue lit (dark)  -> blue not black
    hsv_blob(frame, 430, 380, (0, 0, 40))        # OFF: dark grey    -> black
    # a coloured-but-dim distractor grey-ish patch that is NOT round-enough/dark
    # -> should NOT become a black buoy (kept out of the ROI-size/shape gates)

    for i in range(3):  # a few identical frames so the run has >1 frame
        cv2.imwrite(os.path.join(out_dir, f"frame_{i:03d}.jpg"), frame)
    print(f"wrote 3 frames to {out_dir} ({W}x{H}, blob r={R})")
    print("expected: red@160,150  green@430,150  blue@700,150  black@430,380")


if __name__ == "__main__":
    main()
