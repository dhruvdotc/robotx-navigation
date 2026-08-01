#!/usr/bin/env python3
"""Generate integration frames for online colour re-adaptation: a single GREEN
blob whose hue drifts DOWN 92 -> 58 across 60 frames (lighting drift toward
yellow-green, out of the green range's lower edge where no other colour
catches it). Without online re-adaptation the fixed green range loses it
entirely once hue < ~77; with --online-recolor the green range follows down
and keeps detecting it as green.
"""
import os

import cv2
import numpy as np

W, H, R = 960, 540, 26
N = 60


def water(seed):
    f = np.full((H, W, 3), (110, 105, 70), np.uint8)
    n = np.random.default_rng(seed).integers(-8, 8, (H, W, 3), dtype=np.int16)
    return np.clip(f.astype(np.int16) + n, 0, 255).astype(np.uint8)


def main():
    out = "/tmp/recolor_drift_frames"
    os.makedirs(out, exist_ok=True)
    for i in range(N):
        hue = int(92 - (92 - 58) * i / (N - 1))   # 92 -> 58 (drift toward yellow)
        bgr = cv2.cvtColor(np.uint8([[(hue, 220, 200)]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()
        f = water(i)
        cv2.circle(f, (450, 270), R, bgr, -1)
        cv2.imwrite(os.path.join(out, f"frame_{i:03d}.jpg"), f)
    print(f"wrote {N} frames to {out}: one blob, hue drift 92 -> 58")


if __name__ == "__main__":
    main()
