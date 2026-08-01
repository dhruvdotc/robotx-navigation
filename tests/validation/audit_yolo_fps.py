#!/usr/bin/env python3
"""Characterize the YOLO false positives on the held-out val set, so we know
what they actually are before trying to "fix" them.

For each prediction that matches NO ground-truth box of any class at IoU>0.5
(the class-agnostic FP definition validation_step3 uses), report:
  - conf, predicted class, box + diameter
  - best IoU to ANY GT box and that GT's class (>0.5 same-class = would be a TP;
    0.3-0.5 = localization/near-miss; different-class overlap = class confusion
    or a GT labelling error; ~0 = detection on background/distractor)
  - whether it overlaps another PREDICTION (duplicate-box indicator)

The val GT here is itself weak (HSV auto-labels), so a "FP" may be the model
being right and the label being wrong - this script is how we tell the cases
apart. Read-only; writes annotated crops to /tmp/fp_audit for inspection.
"""
from __future__ import annotations

import os

import cv2
from ultralytics import YOLO

SCRIPTS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "yolo_comparison_test/path2_switch_proposal/scripts",
)
NAMES = {0: "red", 1: "green", 2: "blue"}


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def read_gt(lbl, w, h):
    out = []
    if not os.path.isfile(lbl):
        return out
    with open(lbl, encoding="utf-8") as f:
        for line in f:
            p = line.split()
            if len(p) != 5:
                continue
            cls = int(p[0])
            cx, cy, bw, bh = (float(v) for v in p[1:])
            out.append((cls, (cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h))
    return out


def main():
    out_dir = "/tmp/fp_audit"
    os.makedirs(out_dir, exist_ok=True)
    model = YOLO(os.path.join(SCRIPTS, "training/balloon_proper/weights/best.pt"))
    val_img = os.path.join(SCRIPTS, "dataset/images/val")
    val_lbl = os.path.join(SCRIPTS, "dataset/labels/val")
    names = sorted(n for n in os.listdir(val_img) if n.lower().endswith(".jpg"))

    total_fp = 0
    kinds = {"gt_miss_same_class": 0, "class_confusion": 0, "localization": 0,
             "duplicate": 0, "background": 0}
    print(f"{'image':32s} {'cls':5s} {'conf':5s} {'diam':5s} {'bestIoU':7s} {'gtcls':6s} {'dupPred':7s} kind")
    for name in names:
        img = cv2.imread(os.path.join(val_img, name))
        if img is None:
            continue
        h, w = img.shape[:2]
        gt = read_gt(os.path.join(val_lbl, os.path.splitext(name)[0] + ".txt"), w, h)
        res = model(img, verbose=False)[0]
        preds = []
        if res.boxes is not None:
            for i in range(len(res.boxes)):
                conf = float(res.boxes.conf[i].item())
                if conf < 0.25:
                    continue
                cls = int(res.boxes.cls[i].item())
                x1, y1, x2, y2 = res.boxes.xyxy[i].tolist()
                preds.append((cls, conf, x1, y1, x2, y2))

        # class-agnostic greedy TP match (mirror validation_step3)
        matches = []
        for pi, p in enumerate(preds):
            for gi, g in enumerate(gt):
                ov = iou((p[2], p[3], p[4], p[5]), (g[1], g[2], g[3], g[4]))
                if ov > 0.5:
                    matches.append((ov, pi, gi))
        matches.sort(reverse=True)
        used_p, used_g = set(), set()
        for _, pi, gi in matches:
            if pi in used_p or gi in used_g:
                continue
            used_p.add(pi)
            used_g.add(gi)

        for pi, p in enumerate(preds):
            if pi in used_p:
                continue  # this pred is a TP
            total_fp += 1
            pbox = (p[2], p[3], p[4], p[5])
            # best IoU to any GT
            best_iou, best_gt_cls = 0.0, None
            for g in gt:
                ov = iou(pbox, (g[1], g[2], g[3], g[4]))
                if ov > best_iou:
                    best_iou, best_gt_cls = ov, g[0]
            # overlap with another pred (dup)
            dup = any(pj != pi and iou(pbox, (q[2], q[3], q[4], q[5])) > 0.4 for pj, q in enumerate(preds))
            diam = max(p[4] - p[2], p[5] - p[3])
            if dup:
                kind = "duplicate"
            elif best_iou > 0.5 and best_gt_cls == p[0]:
                kind = "gt_miss_same_class"  # shouldn't happen (would be TP) but guard
            elif best_iou > 0.5 and best_gt_cls != p[0]:
                kind = "class_confusion"
            elif best_iou >= 0.3:
                kind = "localization"
            elif best_iou > 0.05:
                kind = "class_confusion"
            else:
                kind = "background"
            kinds[kind] = kinds.get(kind, 0) + 1
            gtc = NAMES.get(best_gt_cls, "-") if best_gt_cls is not None else "-"
            print(f"{name:32s} {NAMES[p[0]]:5s} {p[1]:.2f}  {diam:5.0f} {best_iou:7.2f} {gtc:6s} {str(dup):7s} {kind}")
            # save crop
            x, y = int(max(0, p[2])), int(max(0, p[3]))
            crop = img[y:int(p[5]), x:int(p[4])]
            if crop.size:
                cv2.imwrite(os.path.join(out_dir, f"fp_{total_fp:02d}_{NAMES[p[0]]}_{name}"), crop)

    print(f"\nTotal FPs: {total_fp}")
    for k, v in kinds.items():
        if v:
            print(f"  {k}: {v}")


if __name__ == "__main__":
    main()
