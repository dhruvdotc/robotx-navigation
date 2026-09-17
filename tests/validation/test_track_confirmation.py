#!/usr/bin/env python3
"""Synthetic frame-sequence validation for --min-track-hits.

The static single-image val set used everywhere else in tests/validation/ has
no frame-to-frame continuity, so it cannot exercise this feature at all (a
track only ever gets one match, hits==1 always, regardless of the threshold).
This test instead simulates two short frame sequences directly against the
real update_tracks() function, mirroring the reporting filter camera_live_feed.py
applies around its update_tracks() call:

  1. A persistent real buoy: matched at ~the same screen position every frame.
     Should eventually be reported (a couple of frames of confirmation latency
     is fine -- a real buoy stays in view for seconds at flight framerate).
  2. A one-off spurious detection (the failure mode measured in
     tests/validation/audit_yolo_fps.py: a confident, single-frame, non-recurring
     background false positive): appears once, never again. Should never cross
     the confirmation threshold, so it is never reported.
"""
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

from camera_live_feed import Detection, update_tracks  # noqa: E402

GATE_PX = 70.0
MAX_MISSED = 8
MIN_HITS = 3


def det(cx, cy, color="red"):
    return Detection(color=color, confidence=0.9, cx_full=cx, cy_full=cy, radius_det=20.0, bbox_full=(int(cx - 20), int(cy - 20), 40, 40))


def confirmed_reports(assigned, track_by_id, min_hits):
    """Mirrors the reporting filter in camera_live_feed.py's main loop."""
    if min_hits <= 1:
        return assigned
    return [a for a in assigned if track_by_id.get(a[1]) is not None and track_by_id[a[1]].hits >= min_hits]


def run_sequence(frames, min_hits):
    """frames: list of detection-lists, one per simulated frame."""
    tracks = []
    next_id = 0
    reported_per_frame = []
    for dets in frames:
        tracks, assigned, next_id = update_tracks(tracks, dets, GATE_PX, MAX_MISSED, next_id)
        track_by_id = {t.track_id: t for t in tracks}
        reported_per_frame.append(confirmed_reports(assigned, track_by_id, min_hits))
    return reported_per_frame


def main():
    failures = []

    # --- Scenario A: persistent real buoy, 8 frames, same screen position ---
    persistent = [[det(500, 400)] for _ in range(8)]
    reports_off = run_sequence(persistent, min_hits=1)
    reports_on = run_sequence(persistent, min_hits=MIN_HITS)

    total_off = sum(len(r) for r in reports_off)
    total_on = sum(len(r) for r in reports_on)
    print(f"Scenario A (persistent buoy, 8 frames): reports with min_hits=1 -> {total_off}/8 frames, "
          f"min_hits={MIN_HITS} -> {total_on}/8 frames")
    if total_off != 8:
        failures.append("min_hits=1 (current default) should report the persistent buoy every frame")
    if total_on != 8 - (MIN_HITS - 1):
        failures.append(f"min_hits={MIN_HITS} should suppress exactly the first {MIN_HITS - 1} frames, then report every frame after")
    # confirm the buoy IS eventually reported, not lost forever
    if not any(reports_on):
        failures.append("persistent buoy was never confirmed -- min-hits gate is too strict or broken")

    # --- Scenario B: one-off spurious FP (real measured failure mode), never recurs ---
    one_off = [[det(900, 700, color="green")]] + [[] for _ in range(MAX_MISSED + 2)]
    reports_fp_off = run_sequence(one_off, min_hits=1)
    reports_fp_on = run_sequence(one_off, min_hits=MIN_HITS)
    total_fp_off = sum(len(r) for r in reports_fp_off)
    total_fp_on = sum(len(r) for r in reports_fp_on)
    print(f"Scenario B (one-off spurious FP, never recurs): reports with min_hits=1 -> {total_fp_off}, "
          f"min_hits={MIN_HITS} -> {total_fp_on}")
    if total_fp_off != 1:
        failures.append("min_hits=1 (current default) should report the one-off FP once, as measured live")
    if total_fp_on != 0:
        failures.append(f"min_hits={MIN_HITS} should suppress the one-off FP entirely -- it never got a second match")

    print()
    if failures:
        print("FAILED:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("PASSED: min-hits confirmation gate reports the persistent buoy (delayed a few frames) "
          "and fully suppresses the one-off spurious detection, without changing default (min_hits=1) behaviour.")


if __name__ == "__main__":
    main()
