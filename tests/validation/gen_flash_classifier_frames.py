#!/usr/bin/env python3
"""Generate an ordered frame sequence for validating the flashing-vs-solid
track classifier. One green blob is lit every frame (SOLID); another toggles
15-on / 15-off (FLASHING, ~1 s/1 s at 15 fps). Drives the real detector via
--image-dir.
"""
import os

import cv2
import numpy as np

W, H, R = 960, 540, 26
N = 90          # 3 flash periods of 15+15
PHASE = 15      # frames per on/off half-period
GREEN_BGR = cv2.cvtColor(np.uint8([[(86, 220, 200)]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()


def water(seed):
    f = np.full((H, W, 3), (110, 105, 70), np.uint8)
    n = np.random.default_rng(seed).integers(-8, 8, (H, W, 3), dtype=np.int16)
    return np.clip(f.astype(np.int16) + n, 0, 255).astype(np.uint8)


def main():
    out = "/tmp/flash_classifier_frames"
    os.makedirs(out, exist_ok=True)
    for i in range(N):
        f = water(i)
        cv2.circle(f, (200, 270), R, GREEN_BGR, -1)          # SOLID: always on
        if (i // PHASE) % 2 == 0:                             # FLASHING: on/off
            cv2.circle(f, (700, 270), R, GREEN_BGR, -1)
        cv2.imwrite(os.path.join(out, f"frame_{i:03d}.jpg"), f)
    print(f"wrote {N} frames to {out}: SOLID@200,270 (always), FLASHING@700,270 ({PHASE}on/{PHASE}off)")


if __name__ == "__main__":
    main()
