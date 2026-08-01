#!/usr/bin/env python3
"""Comprehensive scenarios for the flashing-vs-solid classifier (TrackFlashState
in camera_live_feed.py). Drives classify() directly on ON/OFF sequences (the
classifier is a pure function of the per-frame ON/OFF history), covering the
handbook's 1 s-on/1 s-off cadence at several framerates, detection dropouts on a
solid light, an all-dark track, and the entry(flashing-blue) vs exit(steady-blue)
distinction that actually decides port/starboard.

Run before/after any change to classify() to confirm no scenario regresses.
"""
from __future__ import annotations

import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
from camera_live_feed import TrackFlashState  # noqa: E402

# CLI defaults for the flash classifier.
MIN_FRAMES = 30
MIN_TOGGLES = 2
SOLID_RATIO = 0.85


def seq(pattern, window):
    """Feed a bool list into a TrackFlashState(window) and classify."""
    st = TrackFlashState(window=window)
    for on in pattern:
        st.observe(on)
    return st.classify(MIN_FRAMES, MIN_TOGGLES, SOLID_RATIO)


def flash(on_n, off_n, total):
    out = []
    while len(out) < total:
        out += [True] * on_n + [False] * off_n
    return out[:total]


def dropouts(base, drop_idxs):
    out = list(base)
    for i in drop_idxs:
        out[i] = not out[i]
    return out


SCENARIOS = [
    # name, pattern, window, expected
    ("solid_clean",            [True] * 60,               60,  "solid"),
    ("solid_with_dropouts",    dropouts([True] * 60, [10, 25, 40, 55]), 60, "solid"),   # 4 missed frames on a solid light
    ("flash_10fps",            flash(10, 10, 60),          60,  "flashing"),
    ("flash_15fps",            flash(15, 15, 60),          60,  "flashing"),
    # At 30fps a 1s/1s flash fills a 60-frame window as [30 on, 30 off] - only
    # ONE transition, indistinguishable from a light that turned off once. It is
    # information-theoretically undetectable at this window; correct answer is
    # 'unknown'. The fix is a wider window (>= 2 periods), shown next.
    ("flash_30fps_win60_undersized", flash(30, 30, 60),    60,  "unknown"),
    ("flash_30fps_win120",     flash(30, 30, 120),         120, "flashing"),   # correctly-sized window
    ("flash_with_dropouts",    dropouts(flash(15, 15, 60), [3, 20, 47]), 60, "flashing"),
    ("all_dark",               [False] * 60,               60,  "off"),        # track alive but light dark whole window
    ("short_track",            [True] * 20,                60,  "unknown"),    # < min_frames
    ("entry_flashing_blue",    flash(15, 15, 60),          60,  "flashing"),   # ENTRY marker
    ("exit_steady_blue",       [True] * 60,                60,  "solid"),      # EXIT marker
]


def main():
    passed = failed = 0
    print(f"{'scenario':24s} {'expected':10s} {'got':10s} result")
    for name, pattern, window, expected in SCENARIOS:
        got = seq(pattern, window)
        ok = got == expected
        passed += ok
        failed += not ok
        print(f"{name:24s} {expected:10s} {got:10s} {'PASS' if ok else 'FAIL'}")
    print(f"\n{passed} passed, {failed} failed")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
