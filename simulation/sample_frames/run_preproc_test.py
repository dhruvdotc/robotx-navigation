#!/usr/bin/env python3
"""
Visual + numerical test of 00_preprocess_training_data.py
on the 4 clean simulation sample frames.

Outputs a per-frame comparison grid to:
  simulation/sample_frames/preproc_test/
"""
from __future__ import annotations

import sys
import warnings
from pathlib import Path

import cv2
import numpy as np

warnings.filterwarnings("error")

ROOT = Path(__file__).resolve().parents[2]
FRAMES_DIR = ROOT / "simulation" / "sample_frames"
OUT_DIR = FRAMES_DIR / "preproc_test"
OUT_DIR.mkdir(exist_ok=True)

sys.path.insert(0, str(ROOT / "yolo_comparison_test/path2_switch_proposal/scripts"))
from importlib.util import spec_from_file_location, module_from_spec

spec = spec_from_file_location(
    "pp",
    ROOT / "yolo_comparison_test/path2_switch_proposal/scripts/00_preprocess_training_data.py",
)
pp = module_from_spec(spec)
spec.loader.exec_module(pp)

sources = sorted(p for p in FRAMES_DIR.glob("*.png") if "annotated" not in p.name)
print(f"Test frames: {[p.name for p in sources]}\n")

pipeline = pp.build_aug_pipeline()
print("Augmentation pipeline built OK\n")

SCALE = 0.35
FONT  = cv2.FONT_HERSHEY_SIMPLEX


def thumb(img: np.ndarray) -> np.ndarray:
    h, w = img.shape[:2]
    return cv2.resize(img, (int(w * SCALE), int(h * SCALE)))


def put_label(img: np.ndarray, text: str) -> np.ndarray:
    out = img.copy()
    cv2.rectangle(out, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
    cv2.putText(out, text, (6, 22), FONT, 0.60, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def channel_std(img: np.ndarray) -> float:
    """Std-dev of per-channel means - high value = colour cast."""
    return float(np.std([img[:, :, c].mean() for c in range(3)]))


all_passed = True

for src in sources:
    img = cv2.imread(str(src))
    assert img is not None, f"Could not read {src}"
    h, w = img.shape[:2]

    # Phase 2 only: normalize the original
    normed = pp.color_normalize(img)

    # Phase 1 + 2: augment then normalize (3 variants)
    aug_normed: list[np.ndarray] = []
    for _ in range(3):
        aug    = pp.augment_image(img, pipeline)
        aug_n  = pp.color_normalize(aug)
        assert aug_n.shape == img.shape, "Shape changed after augment+normalize"
        aug_normed.append(aug_n)

    # ── Numerical checks ────────────────────────────────────────────────────
    std_before = channel_std(img)
    std_after  = channel_std(normed)
    # Gray-World should reduce channel imbalance (unless image was already balanced)
    if std_before > 3.0:
        assert std_after < std_before, (
            f"{src.name}: GW WB increased channel imbalance "
            f"({std_before:.2f} -> {std_after:.2f})"
        )

    assert normed.min() >= 0 and normed.max() <= 255, "Pixel values out of [0,255]"

    # Augmentations should produce meaningfully different images (not identity)
    diffs = [float(np.abs(img.astype(np.int16) - an.astype(np.int16)).mean())
             for an in aug_normed]
    for i, d in enumerate(diffs):
        assert d > 0.5, f"Aug {i+1} looks identical to original (mean diff={d:.3f})"

    print(f"  {src.name}  [{w}×{h}]")
    print(f"    channel-std   before={std_before:.2f}  after={std_after:.2f}  ✓")
    print(f"    pixel range   [{normed.min()}, {normed.max()}]  ✓")
    for i, (an, d) in enumerate(zip(aug_normed, diffs)):
        print(f"    aug_{i+1}  shape={an.shape}  mean-diff-vs-orig={d:.1f} px  ✓")

    # ── Visual grid (2 rows × 3 cols) ───────────────────────────────────────
    # Row 1: original │ color-normalized │ aug1+normalized
    # Row 2: aug2+normalized │ aug3+normalized │ blank
    panels = [
        put_label(thumb(img),          "original"),
        put_label(thumb(normed),       "color-normalized"),
        put_label(thumb(aug_normed[0]),"aug1 + normalized"),
        put_label(thumb(aug_normed[1]),"aug2 + normalized"),
        put_label(thumb(aug_normed[2]),"aug3 + normalized"),
    ]
    # Pad to 6 so rows are even
    panels.append(np.zeros_like(panels[0]))
    row1 = np.hstack(panels[:3])
    row2 = np.hstack(panels[3:])
    grid = np.vstack([row1, row2])

    out_path = OUT_DIR / f"{src.stem}_grid.jpg"
    ok = cv2.imwrite(str(out_path), grid, [cv2.IMWRITE_JPEG_QUALITY, 90])
    assert ok, f"Failed to write {out_path}"
    print(f"    grid saved → {out_path.name}\n")


print("=" * 60)
if all_passed:
    print(f"ALL {len(sources)} FRAMES PASSED.  Grids in:")
    print(f"  {OUT_DIR}")
else:
    print("SOME FRAMES FAILED - see above.")
    sys.exit(1)
