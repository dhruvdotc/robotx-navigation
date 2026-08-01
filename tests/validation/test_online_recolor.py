#!/usr/bin/env python3
"""Unit-validate online colour re-adaptation: the hue EMA follows an observed
drift and re-centres the ranges."""
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
from camera_live_feed import OnlineColorAdapter  # noqa: E402

RANGES = {
    "red": [((0, 50, 45), (12, 255, 255)), ((168, 50, 45), (179, 255, 255))],
    "green": [((77, 50, 45), (101, 255, 255))],
    "blue": [((108, 50, 45), (132, 255, 255))],
}


def roi(hue):
    return np.full((30, 30, 3), (hue, 200, 200), np.uint8)


def main():
    ad = OnlineColorAdapter(RANGES, alpha=0.3, hue_margin=12, sat_floor=50, val_floor=45)
    seed = {c: round(h, 1) for c, h in ad.ema_hue.items()}
    print("seed EMA hues:", seed)
    ok = abs(seed["green"] - 89) <= 2 and abs(seed["blue"] - 120) <= 2 and seed["red"] <= 6

    # drift green observations 90 -> 114
    for hue in range(90, 116, 2):
        ad.observe("green", roi(hue))
    g = ad.ema_hue["green"]
    print(f"green EMA after drift to 114: {g:.1f} (seed was {seed['green']})")
    ok &= g > seed["green"] + 10  # clearly followed the drift upward

    rng = ad.build_ranges()
    gc = (rng["green"][0][0][0] + rng["green"][0][1][0]) / 2.0
    print(f"green range re-centred to: {rng['green']}  (centre ~{gc:.0f})")
    ok &= abs(gc - g) <= 2  # range centred on the EMA hue

    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
