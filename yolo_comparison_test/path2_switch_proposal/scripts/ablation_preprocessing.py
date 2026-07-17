#!/usr/bin/env python3
"""
ablation_preprocessing.py — Preprocessing pipeline ablation study.

Compares 4 preprocessing variants on the same 65 raw val images with
the same frozen YOLO weights and same GT labels.  Answers the question:
"which pipeline should we use for BOTH training and inference?"

Pipelines tested
----------------
  A  CLAHE YUV  + Gray-World WB + Gaussian unsharp   [current]
  B  CLAHE LAB  + Gray-World WB + Gaussian unsharp
  C  CLAHE YUV  + Gray-World WB + Bilateral unsharp
  D  CLAHE LAB  + Gray-World WB + Bilateral unsharp   [fully proposed]

Fair test
---------
  • Raw captures for the val images are read from ../captures/
    (identified via split_manifest.txt — same seed=42 split as training).
  • GT labels are the existing dataset/labels/val/ YOLO .txt files
    (positions do not change with colour preprocessing).
  • Model weights are unchanged: training/balloon_proper/weights/best.pt
  • IoU threshold: 0.5  Confidence threshold: 0.25
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field

import cv2
import numpy as np
from ultralytics import YOLO

# ---------------------------------------------------------------------------
# Preprocessing primitives
# ---------------------------------------------------------------------------

def clahe_yuv(img: np.ndarray) -> np.ndarray:
    yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    yuv[:, :, 0] = clahe.apply(yuv[:, :, 0])
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)


def clahe_lab(img: np.ndarray) -> np.ndarray:
    """CLAHE on L channel in perceptually-uniform LAB space.
    Normalises luma without touching the a/b colour axes, so hue is
    unaffected — unlike YUV where Y bleed can shift chroma slightly."""
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    lab[:, :, 0] = clahe.apply(lab[:, :, 0])
    return cv2.cvtColor(lab, cv2.COLOR_Lab2BGR)


def gray_world_wb(img: np.ndarray) -> np.ndarray:
    f = img.astype(np.float64)
    gray = f.mean()
    for c in range(3):
        m = f[:, :, c].mean()
        if m > 1e-6:
            f[:, :, c] *= gray / m
    return np.clip(f, 0, 255).astype(np.uint8)


def unsharp_gaussian(img: np.ndarray, sigma: float = 1.0, strength: float = 0.8) -> np.ndarray:
    blurred = cv2.GaussianBlur(img, (0, 0), sigma)
    return np.clip(cv2.addWeighted(img, 1.0 + strength, blurred, -strength, 0), 0, 255).astype(np.uint8)


def unsharp_bilateral(img: np.ndarray, strength: float = 0.8) -> np.ndarray:
    """Edge-preserving unsharp mask using bilateral filter.
    Sharpens buoy boundaries without amplifying the noisy water texture
    that surrounds them — the primary source of red false positives."""
    blurred = cv2.bilateralFilter(img, d=7, sigmaColor=50, sigmaSpace=50)
    return np.clip(cv2.addWeighted(img, 1.0 + strength, blurred, -strength, 0), 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# Pipeline definitions
# ---------------------------------------------------------------------------

PIPELINES: dict[str, list] = {
    "A  CLAHE-YUV + GW-WB + Gauss unsharp  [current]": [
        clahe_yuv, gray_world_wb, unsharp_gaussian,
    ],
    "B  CLAHE-LAB + GW-WB + Gauss unsharp             ": [
        clahe_lab, gray_world_wb, unsharp_gaussian,
    ],
    "C  CLAHE-YUV + GW-WB + Bilateral unsharp         ": [
        clahe_yuv, gray_world_wb, unsharp_bilateral,
    ],
    "D  CLAHE-LAB + GW-WB + Bilateral unsharp [full]  ": [
        clahe_lab, gray_world_wb, unsharp_bilateral,
    ],
}


def apply_pipeline(img: np.ndarray, steps: list) -> np.ndarray:
    for fn in steps:
        img = fn(img)
    return img


# ---------------------------------------------------------------------------
# Matching helpers
# ---------------------------------------------------------------------------

@dataclass
class Box:
    cls: int
    conf: float
    x1: float; y1: float; x2: float; y2: float


def iou(a: Box, b: Box) -> float:
    ix1, iy1 = max(a.x1, b.x1), max(a.y1, b.y1)
    ix2, iy2 = min(a.x2, b.x2), min(a.y2, b.y2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(0.0, a.x2 - a.x1) * max(0.0, a.y2 - a.y1)
    area_b = max(0.0, b.x2 - b.x1) * max(0.0, b.y2 - b.y1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def greedy_match(preds: list[Box], gts: list[Box],
                 cls_filter: int | None = None) -> tuple[int, int, int]:
    pids = [i for i, p in enumerate(preds) if cls_filter is None or p.cls == cls_filter]
    gids = [i for i, g in enumerate(gts)  if cls_filter is None or g.cls == cls_filter]
    candidates = sorted(
        [(iou(preds[pi], gts[gi]), pi, gi) for pi in pids for gi in gids if iou(preds[pi], gts[gi]) > 0.5],
        reverse=True,
    )
    used_p: set[int] = set()
    used_g: set[int] = set()
    tp = 0
    for _, pi, gi in candidates:
        if pi in used_p or gi in used_g:
            continue
        used_p.add(pi); used_g.add(gi); tp += 1
    return tp, len(pids) - len(used_p), len(gids) - len(used_g)


def prf(tp: int, fp: int, fn: int) -> tuple[float, float, float]:
    p = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    r = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f = (2 * p * r / (p + r)) if (p + r) > 0 else 0.0
    return p, r, f


def read_gt(lbl_path: str, w: int, h: int) -> list[Box]:
    boxes: list[Box] = []
    if not os.path.isfile(lbl_path):
        return boxes
    for line in open(lbl_path, encoding="utf-8"):
        parts = line.strip().split()
        if len(parts) != 5:
            continue
        cls = int(parts[0])
        cx_n, cy_n, bw_n, bh_n = map(float, parts[1:])
        bw, bh = bw_n * w, bh_n * h
        cx, cy = cx_n * w, cy_n * h
        boxes.append(Box(cls, 1.0, cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2))
    return boxes


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    root = os.path.dirname(__file__)
    repo_root = os.path.join(root, "..", "..")   # path2_switch_proposal/../..

    captures_dir   = os.path.join(root, "..", "captures")
    manifest_path  = os.path.join(root, "..", "preprocessed_captures", "split_manifest.txt")
    val_lbl_dir    = os.path.join(root, "dataset", "labels", "val")
    weights        = os.path.join(root, "training", "balloon_proper", "weights", "best.pt")

    # --- Load val filenames from manifest ---
    if not os.path.isfile(manifest_path):
        print(f"[ERROR] split_manifest.txt not found at {manifest_path}")
        return 1
    val_names: list[str] = []
    for line in open(manifest_path, encoding="utf-8"):
        if line.startswith("val\t"):
            val_names.append(line.strip().split("\t", 1)[1])
    print(f"Val set: {len(val_names)} raw images (from split_manifest.txt)\n")

    # --- Load model ---
    model = YOLO(weights)

    CLASS_NAMES = {0: "red", 1: "green", 2: "blue"}

    results_table: list[dict] = []

    for pipeline_name, steps in PIPELINES.items():
        tp_all = fp_all = fn_all = 0
        cls_stats = {c: [0, 0, 0] for c in (0, 1, 2)}
        missing = 0

        for fname in val_names:
            raw_path = os.path.join(captures_dir, fname)
            if not os.path.isfile(raw_path):
                missing += 1
                continue

            raw = cv2.imread(raw_path)
            if raw is None:
                missing += 1
                continue
            h, w = raw.shape[:2]

            proc = apply_pipeline(raw.copy(), steps)

            result = model(proc, verbose=False)[0]
            preds: list[Box] = []
            if result.boxes is not None:
                for i in range(len(result.boxes)):
                    conf = float(result.boxes.conf[i].item())
                    if conf < 0.25:
                        continue
                    cls = int(result.boxes.cls[i].item())
                    x1, y1, x2, y2 = result.boxes.xyxy[i].tolist()
                    preds.append(Box(cls, conf, x1, y1, x2, y2))

            stem = os.path.splitext(fname)[0]
            lbl_path = os.path.join(val_lbl_dir, stem + ".txt")
            gt = read_gt(lbl_path, w, h)

            tp, fp, fn = greedy_match(preds, gt)
            tp_all += tp; fp_all += fp; fn_all += fn
            for c in (0, 1, 2):
                ct, cf, cn = greedy_match(preds, gt, c)
                cls_stats[c][0] += ct
                cls_stats[c][1] += cf
                cls_stats[c][2] += cn

        p, r, f1 = prf(tp_all, fp_all, fn_all)
        per_cls = {c: prf(*cls_stats[c]) for c in (0, 1, 2)}
        results_table.append({
            "name": pipeline_name,
            "tp": tp_all, "fp": fp_all, "fn": fn_all,
            "P": p, "R": r, "F1": f1,
            "per_cls": per_cls,
            "missing": missing,
        })

    # --- Print results ---
    HDR = "\n{:<52}  {:>5}  {:>5}  {:>5}  {:>6}  {:>6}  {:>6}"
    ROW = "{:<52}  {:>5}  {:>5}  {:>5}  {:>6.3f}  {:>6.3f}  {:>6.3f}"

    print("=" * 88)
    print("ABLATION: preprocessing pipeline vs detection accuracy (same model, same GT labels)")
    print("=" * 88)
    print(HDR.format("Pipeline", "TP", "FP", "FN", "Prec", "Rec", "F1"))
    print("-" * 88)
    for r in results_table:
        print(ROW.format(r["name"], r["tp"], r["fp"], r["fn"], r["P"], r["R"], r["F1"]))

    print()
    print("Per-class breakdown:")
    CLS_HDR = "  {:<52}  {:>14}  {:>14}  {:>14}"
    CLS_ROW = "  {:<52}  {:>14}  {:>14}  {:>14}"
    print(CLS_HDR.format("Pipeline", "red P/R/F1", "green P/R/F1", "blue P/R/F1"))
    print("  " + "-" * 98)
    for r in results_table:
        cls_str = []
        for c in (0, 1, 2):
            cp, cr, cf = r["per_cls"][c]
            cls_str.append(f"{cp:.2f}/{cr:.2f}/{cf:.2f}")
        print(CLS_ROW.format(r["name"], *cls_str))

    # --- Save results ---
    out_path = os.path.join(root, "ablation_results.txt")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("PREPROCESSING ABLATION RESULTS\n")
        f.write("=" * 88 + "\n")
        for r in results_table:
            f.write(f"\n{r['name']}\n")
            f.write(f"  Overall:  TP={r['tp']} FP={r['fp']} FN={r['fn']}  "
                    f"P={r['P']:.3f} R={r['R']:.3f} F1={r['F1']:.3f}\n")
            for c in (0, 1, 2):
                cp, cr, cf = r["per_cls"][c]
                f.write(f"  {CLASS_NAMES[c]:<6}: P={cp:.3f} R={cr:.3f} F1={cf:.3f}\n")

    print(f"\nSaved → {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
