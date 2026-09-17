#!/usr/bin/env python3
"""Auto-scan the held-out val set for every post-circ50 YOLO false positive and
check whether it's a real, unlabeled buoy or a genuine distractor confusion.

Method: project both the labeled GT box(es) and the "FP" box in a frame to
real-world ground offsets (project_pixel_to_ground_ned, the same math the
live pipeline uses), then compute the FP's offset relative to each labeled
buoy in that frame. Search every pairwise offset between real buoys in the
frame's course layout for the closest match. A small residual (roughly
<2 m, consistent with the known unmodeled height-parallax bias for these
0.99 m tall cylinders viewed off-nadir - see docs/07_roadmap.md, "IMU
pitch/roll compensation... not done") means the "FP" is very likely a real,
mislabeled buoy. A large residual means it's a genuine distractor confusion.

This is how the 2026-09-17 GT fix (course1_frame_1783758568617.jpg was
missing 2 real buoy labels) was found. Re-run this after any dataset change
or retrain to catch new labeling gaps before they get mis-scored as detector
false positives.

Frame filenames must start with a known course tag (course1/course2/course3)
or "loiter" (which flies Course 1's world) to be checked; anything else is
skipped with a note, since the real-world layout is course-specific.
"""
import os
import sys

import cv2

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS = os.path.join(REPO, "yolo_comparison_test/path2_switch_proposal/scripts")
sys.path.insert(0, REPO)
import camera_live_feed as clf  # noqa: E402
import json

with open(os.path.join(REPO, "calibration/camera_intrinsics_latest.json")) as f:
    _cal = json.load(f)
import numpy as np  # noqa: E402

_K = np.array([[_cal["fx"], 0, _cal["cx"]], [0, _cal["fy"], _cal["cy"]], [0, 0, 1]], dtype=np.float64)
_DIST = np.zeros(5)  # sim renders a clean pinhole; matches --no-undistort used for all sim frames
INTR = clf.Intrinsics(fx=_cal["fx"], fy=_cal["fy"], cx=_cal["cx"], cy=_cal["cy"], dist=_DIST, K=_K, source="calib")
ALT = 10.0
HEADING = 0.0
CLASS_NAME = {0: "red", 1: "green", 2: "blue"}

# Known real buoy layouts (N, E) in metres, from each course's world SDF /
# simulation/README.md. "loiter" flies Course 1's world.
COURSE_LAYOUTS = {
    "course1": {
        "gate1_green": (1.25, 10), "gate1_red": (-1.25, 10),
        "gate2_green": (1.25, 25), "gate2_red": (-1.25, 25),
        "gate3_green": (1.25, 40), "gate3_red": (-1.25, 40),
        "light_buoy": (0, 50),
    },
    "course2": {
        "green1": (10, 8), "red1": (-11, 14),
        "green2": (-8, 24), "red2": (6, 31),
        "green3": (7, 42), "red3": (-5, 48),
        "light_buoy": (2, 55),
    },
    "course3": {
        "gate1_green": (1.25, 10), "gate1_red": (-1.25, 10),
        "gate2_green": (1.25, 25), "gate2_red": (-1.25, 25),
        "gate3_green": (15, 36.25), "gate3_red": (15, 33.75),
        "gate4_green": (30, 36.25), "gate4_red": (30, 33.75),
        "light_buoy": (42, 35),
    },
    "loiter": {
        "gate1_green": (1.25, 10), "gate1_red": (-1.25, 10),
        "gate2_green": (1.25, 25), "gate2_red": (-1.25, 25),
        "gate3_green": (1.25, 40), "gate3_red": (-1.25, 40),
        "light_buoy": (0, 50),
    },
}

REAL_MATCH_M = 2.0  # residual below this: very likely a real, unlabeled buoy
PLAUSIBLE_M = 4.0  # residual below this: worth a human look


def course_of(name):
    for tag in ("course1", "course2", "course3", "loiter"):
        if name.startswith(tag):
            return tag
    return None


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
            cls = int(p[0])
            cx, cy, bw, bh = (float(v) for v in p[1:])
            out.append((cls, (cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h))
    return out


def ground(px, py):
    return clf.project_pixel_to_ground_ned(px, py, INTR, ALT, HEADING)


def best_match(dN, dE, layout, fp_color):
    """Closest pairwise offset (candidate - anchor) among same-colored real
    buoys in the layout, to the measured (dN, dE)."""
    names = [n for n in layout if fp_color in n.lower()]
    best = None
    for a in layout:
        an, ae = layout[a]
        for b in names:
            if b == a:
                continue
            bn, be = layout[b]
            err = ((bn - an - dN) ** 2 + (be - ae - dE) ** 2) ** 0.5
            if best is None or err < best[0]:
                best = (err, a, b)
    return best


def main():
    model = clf.load_yolo_model(os.path.join(SCRIPTS, "training/balloon_proper/weights/best.pt"))
    val_img = os.path.join(SCRIPTS, "dataset/images/val")
    val_lbl = os.path.join(SCRIPTS, "dataset/labels/val")
    names = sorted(n for n in os.listdir(val_img) if n.lower().endswith(".jpg"))

    likely_real, plausible, distractor, skipped = 0, 0, 0, 0
    for name in names:
        course = course_of(name)
        img = cv2.imread(os.path.join(val_img, name))
        if img is None:
            continue
        h, w = img.shape[:2]
        gt = read_gt(os.path.join(val_lbl, os.path.splitext(name)[0] + ".txt"), w, h)
        dets = clf.find_detections_yolo(img, model, 0.25, min_circularity=0.5)

        fps = []
        for d in dets:
            x, y, bw, bh = d.bbox_full
            box = (x, y, x + bw, y + bh)
            best_iou = max((iou(box, (g[1], g[2], g[3], g[4])) for g in gt), default=0.0)
            if best_iou <= 0.5:
                fps.append(d)
        if not fps:
            continue

        if course is None or not gt:
            for d in fps:
                print(f"[SKIP: no course layout or no GT anchor] {name}: {d.color} conf={d.confidence:.2f}")
                skipped += 1
            continue

        layout = COURSE_LAYOUTS[course]
        gt_ground = [(cls, *ground((x1 + x2) / 2, (y1 + y2) / 2)) for cls, x1, y1, x2, y2 in gt]
        for d in fps:
            x, y, bw, bh = d.bbox_full
            fn, fe = ground(x + bw / 2, y + bh / 2)
            row_best = None
            for cls, gn, ge in gt_ground:
                err, a, b = best_match(fn - gn, fe - ge, layout, d.color)
                if row_best is None or err < row_best[0]:
                    row_best = (err, a, b)
            err, a, b = row_best
            if err < REAL_MATCH_M:
                verdict, likely_real = "LIKELY REAL BUOY (check GT!)", likely_real + 1
            elif err < PLAUSIBLE_M:
                verdict, plausible = "plausible, worth a look", plausible + 1
            else:
                verdict, distractor = "genuine distractor", distractor + 1
            print(f"[{course}] {name}: {d.color} conf={d.confidence:.2f} -> best layout match "
                  f"{a}->{b} residual={err:.2f}m  [{verdict}]")

    print(f"\n{likely_real} likely real (GT gap), {plausible} plausible/worth a look, "
          f"{distractor} genuine distractor, {skipped} skipped (no course tag or no GT in frame)")


if __name__ == "__main__":
    main()
