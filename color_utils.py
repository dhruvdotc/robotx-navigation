#!/usr/bin/env python3
"""Shared color range helpers for Stage-A HSV detection pipelines."""

from __future__ import annotations

import os
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class HSVRange:
    low: tuple[int, int, int]
    high: tuple[int, int, int]


FALLBACK_COLOR_RANGES: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {
    "red": [((0, 100, 70), (10, 255, 255)), ((170, 100, 70), (179, 255, 255))],
    "green": [((75, 60, 50), (105, 255, 255)),
    ],
    "blue": [((100, 80, 60), (130, 255, 255))],
}


def list_images(folder: str) -> list[str]:
    exts = (".jpg", ".jpeg", ".png", ".bmp")
    paths: list[str] = []
    for name in sorted(os.listdir(folder)):
        path = os.path.join(folder, name)
        if os.path.isfile(path) and name.lower().endswith(exts):
            paths.append(path)
    return paths


def circular_hue_mean(hues: np.ndarray) -> int:
    angles = (hues.astype(np.float32) / 180.0) * 2.0 * np.pi
    s = np.sin(angles).mean()
    c = np.cos(angles).mean()
    mean_angle = np.arctan2(s, c)
    if mean_angle < 0:
        mean_angle += 2.0 * np.pi
    return int(round((mean_angle / (2.0 * np.pi)) * 180.0)) % 180


def make_ranges_for_hue(hue_center: int, hue_margin: int, s_min: int, v_min: int) -> list[HSVRange]:
    h_low = hue_center - hue_margin
    h_high = hue_center + hue_margin
    if h_low < 0:
        return [
            HSVRange((0, s_min, v_min), (h_high, 255, 255)),
            HSVRange((180 + h_low, s_min, v_min), (179, 255, 255)),
        ]
    if h_high > 179:
        return [
            HSVRange((h_low, s_min, v_min), (179, 255, 255)),
            HSVRange((0, s_min, v_min), (h_high - 180, 255, 255)),
        ]
    return [HSVRange((h_low, s_min, v_min), (h_high, 255, 255))]


def derive_class_hsv_ranges(classes_dir: str, hue_margin: int, sat_floor: int, val_floor: int) -> dict[str, list[HSVRange]]:
    out: dict[str, list[HSVRange]] = {}
    if not os.path.isdir(classes_dir):
        return out

    for path in list_images(classes_dir):
        color = os.path.splitext(os.path.basename(path))[0].lower()
        if color not in {"red", "green", "blue"}:
            continue

        img = cv2.imread(path)
        if img is None:
            continue
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        valid = (s > 25) & (v > 25)
        if int(valid.sum()) < 10:
            valid = np.ones_like(h, dtype=bool)

        hue_center = circular_hue_mean(h[valid])
        # Use the caller's floor directly, not max(floor, crop's own 15th
        # percentile) -- see 01_autolabel.py's derive_class_hsv_ranges for
        # why: a near-solid reference crop's own percentile is fragile to
        # any real-world gap (compression, lighting, colour normalisation)
        # between the crop and actual in-scene renderings.
        s_min = sat_floor
        v_min = val_floor
        out[color] = make_ranges_for_hue(hue_center, hue_margin, s_min, v_min)

    return out


def build_mask(
    hsv: np.ndarray,
    ranges: list[HSVRange] | list[tuple[tuple[int, int, int], tuple[int, int, int]]],
) -> np.ndarray:
    acc = None
    for r in ranges:
        if isinstance(r, HSVRange):
            low, high = r.low, r.high
        else:
            low, high = r
        m = cv2.inRange(
            hsv,
            np.array(low, dtype=np.uint8),
            np.array(high, dtype=np.uint8),
        )
        acc = m if acc is None else cv2.bitwise_or(acc, m)
    if acc is None:
        return np.zeros(hsv.shape[:2], dtype=np.uint8)
    return acc


def is_off_buoy(hsv_roi: np.ndarray, sat_max: int = 60, val_max: int = 80) -> bool:
    """True if an ROI looks like an unlit (OFF / BLACK) Safe-Passage beacon.

    The OFF state is a dark LED panel with no colour. Requiring BOTH a low median
    saturation and a low median value is deliberate: a dark but *coloured* buoy
    (e.g. the sim's intentionally dark blue light, grayscale ~20 yet highly
    saturated - see simulation/light_buoy_cycler.py) must not be misread as OFF.
    Water sits above the value floor, a lit colour above the saturation floor;
    only a genuinely dark, grey blob passes both.
    """
    if hsv_roi.size == 0:
        return False
    s_med = float(np.median(hsv_roi[:, :, 1]))
    v_med = float(np.median(hsv_roi[:, :, 2]))
    return s_med <= sat_max and v_med <= val_max


def _ranges_to_tuples(ranges_map: dict[str, list[HSVRange]]) -> dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]]:
    out: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {}
    for color, ranges in ranges_map.items():
        out[color] = [(r.low, r.high) for r in ranges]
    return out


def _print_tuple_ranges(title: str, ranges_map: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]]) -> None:
    print(title)
    for color in ("red", "green", "blue"):
        for i, (low, high) in enumerate(ranges_map.get(color, [])):
            print(f"  {color}[{i}] low={low} high={high}")


# ---------------------------------------------------------------------------
# Phase-2 colour normalisation (mirrors yolo_comparison_test/path2_switch_proposal/
# scripts/00_preprocess_training_data.py exactly -- the YOLO fine-tune's training
# images were run through this same pipeline, so inference must match it too).
# ---------------------------------------------------------------------------
NORM_CLAHE_CLIP = 2.0
NORM_CLAHE_TILE = (8, 8)
NORM_UNSHARP_SIGMA = 1.0
NORM_UNSHARP_STRENGTH = 0.8


def clahe_yuv(img_bgr: np.ndarray) -> np.ndarray:
    """CLAHE on the Y (luma) channel in YUV colour space."""
    yuv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YUV)
    clahe = cv2.createCLAHE(clipLimit=NORM_CLAHE_CLIP, tileGridSize=NORM_CLAHE_TILE)
    yuv[:, :, 0] = clahe.apply(yuv[:, :, 0])
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)


def gray_world_wb(img_bgr: np.ndarray) -> np.ndarray:
    """Gray-World white balance: scale channels so their means match."""
    img_f = img_bgr.astype(np.float64)
    b_mean = img_f[:, :, 0].mean()
    g_mean = img_f[:, :, 1].mean()
    r_mean = img_f[:, :, 2].mean()
    gray_mean = (b_mean + g_mean + r_mean) / 3.0

    if b_mean > 1e-6:
        img_f[:, :, 0] *= gray_mean / b_mean
    if g_mean > 1e-6:
        img_f[:, :, 1] *= gray_mean / g_mean
    if r_mean > 1e-6:
        img_f[:, :, 2] *= gray_mean / r_mean

    return np.clip(img_f, 0, 255).astype(np.uint8)


def unsharp_mask(
    img_bgr: np.ndarray,
    sigma: float = NORM_UNSHARP_SIGMA,
    strength: float = NORM_UNSHARP_STRENGTH,
) -> np.ndarray:
    """Unsharp masking: blend a high-frequency detail residual back in."""
    blurred = cv2.GaussianBlur(img_bgr, (0, 0), sigma)
    sharpened = cv2.addWeighted(img_bgr, 1.0 + strength, blurred, -strength, 0)
    return np.clip(sharpened, 0, 255).astype(np.uint8)


def color_normalize(img_bgr: np.ndarray) -> np.ndarray:
    """Full Phase-2 pipeline: CLAHE (YUV) -> Gray-World WB -> Unsharp Masking.

    Apply this to a frame before handing it to a YOLO model fine-tuned via
    00_preprocess_training_data.py's default (non---skip-normalize) path --
    otherwise inference sees a different colour/contrast distribution than
    training did.
    """
    img = clahe_yuv(img_bgr)
    img = gray_world_wb(img)
    img = unsharp_mask(img)
    return img


def load_color_ranges(
    classes_dir: str = "captures/classes",
    hue_margin: int = 12,
    sat_floor: int = 50,
    val_floor: int = 45,
) -> dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]]:
    derived = derive_class_hsv_ranges(classes_dir, hue_margin, sat_floor, val_floor)
    if derived:
        tuple_ranges = _ranges_to_tuples(derived)
        _print_tuple_ranges("Derived HSV ranges:", tuple_ranges)
        return tuple_ranges

    print(f"Classes directory missing or empty: {classes_dir}")
    print("Using fallback HSV ranges:")
    _print_tuple_ranges("Fallback HSV ranges:", FALLBACK_COLOR_RANGES)
    return {color: list(ranges) for color, ranges in FALLBACK_COLOR_RANGES.items()}
