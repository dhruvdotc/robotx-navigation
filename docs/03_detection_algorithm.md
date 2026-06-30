# Detection Algorithm

The detector is a two-stage CV pipeline implemented in `camera_live_feed.py` (live feed) and `hsv_batch_detect.py` (batch). Both share the same core logic.

---

## Overview

```
Frame
  │
  ▼
┌─────────────────────────────┐
│  Stage 1: Shape proposals   │  color-agnostic
│  Canny edges + contours     │
│  → circularity + size gate  │
└─────────────┬───────────────┘
              │ candidate ROIs
              ▼
┌─────────────────────────────┐
│  Stage 2: Color classify    │  HSV thresholding inside ROI only
│  → pick best color by ratio │
│  → confidence score         │
└─────────────┬───────────────┘
              │ Detection(color, conf, centroid)
              ▼
         Kalman tracking  (camera_live_feed.py only)
              │
              ▼
         GPS projection → MAVLink transmit
```

---

## Stage 1 — Shape proposals (color-agnostic)

**File:** `camera_live_feed.py` lines 381–418, `hsv_batch_detect.py` `_proposal_contours()`

```
frame_det (detection resolution, e.g. 1920×1080 or 960×540)
  │
  ├── GaussianBlur(5×5, σ=0)
  ├── Canny(40, 120)              # edge map
  └── [hsv_batch only] Otsu threshold on saturation channel
       → bitwise-OR edges with saturation mask (hybrid proposals)
  │
  morphologyEx(CLOSE, kernel)     # close gaps in edges
  │
  findContours(RETR_EXTERNAL)
  │
  for each contour:
    ├── area < 8 px²  → skip
    ├── circularity = 4π·area/perimeter²  < min_circularity → skip
    ├── size gate: diameter must be within [0.5×, 2.0×] expected_diameter
    │              expected_diameter = fx × target_diameter_m / altitude_m
    └── → candidate
```

Size gating is the primary false-positive suppressor. At 10 m AGL with a 0.32 m buoy and fx≈1319, the expected diameter is ~42 px. Objects dramatically smaller or larger are rejected.

---

## Stage 2 — HSV color classification

**File:** `camera_live_feed.py` lines 419–460

For each candidate ROI from Stage 1:

```
Scale ROI bbox back to full-resolution frame
  │
  Extract hsv_roi from full-res HSV image
  │
  for each color in {red, green, blue}:
    ├── build_mask(hsv_roi, COLOR_RANGES[color])
    ├── morphologyEx(OPEN) → morphologyEx(CLOSE)   # clean up mask
    └── ratio = count_nonzero(mask) / roi_area
  │
  best_color = argmax(ratio)
  │
  if best_ratio < min_color_ratio → discard
  │
  centroid = color mask moments (falls back to contour centroid)
  │
  confidence = 0.45 × size_term + 0.55 × best_ratio
    size_term = 1 - |diameter - expected_diameter| / expected_diameter
```

### HSV color ranges

Defined in `color_utils.py` (fallback) and derived dynamically from `captures/classes/` reference crops:

| Color | Hue range | S min | V min | Notes |
|-------|-----------|-------|-------|-------|
| red | 0–10 and 170–179 | 100 | 70 | Wraps around 0° |
| green | 75–105 | 60 | 50 | Spring-green emissive; sim buoys at ~hue 81 |
| blue | 100–130 | 80 | 60 | |

> **Important:** the simulation README states green range as 75–99, but the actual code upper bound is **105**. Use 75–105 when discussing the code.

When `captures/classes/` exists, `derive_class_hsv_ranges()` computes a median hue per color from the reference crops and applies `--hue-margin` (default ±12) around it, overriding the fallback.

---

## Kalman tracking (`camera_live_feed.py` only)

After each frame's detections, `update_tracks()` matches detections to existing tracks by nearest centroid within `--track-gate-px` (default: auto-scaled to expected diameter).

Each track has a `cv2.KalmanFilter` (constant-velocity, 4-state: x, y, dx, dy). Tracks that miss more than `--max-track-missed` consecutive frames are dropped.

Tracking reduces duplicate GPS reports for the same buoy across frames.

---

## Key tunable parameters

| Flag | Default | Effect |
|------|---------|--------|
| `--altitude-m` | 10 | AGL altitude — scales expected buoy diameter |
| `--target-diameter-m` | 0.32 | Expected buoy diameter in metres |
| `--fx-px` | 1500 (default); calibration JSON overrides | Focal length |
| `--min-circularity` | 0.2 (live feed) | Lower = accept more elongated shapes |
| `--min-color-ratio` | 0.12 (live feed) | Lower = accept less colorful ROIs |
| `--calibration` | `calibration/camera_intrinsics_latest.json` | Intrinsics JSON path |
| `--no-undistort` | off | Skip lens undistortion (always use in Gazebo — ogre2 renders clean pinhole) |

---

## Detection confidence score

```
size_term = max(0, 1 - |detected_diameter - expected_diameter| / expected_diameter)
conf = clip(0.45 × size_term + 0.55 × color_ratio, 0, 1)
```

A perfect-size, fully-saturated buoy scores ~1.0. The size term penalizes detections whose apparent size doesn't match the expected buoy size at the given altitude.

---

## Where to look when the detector misbehaves

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Zero detections on real buoys | HSV ranges too tight for current lighting | Re-derive ranges with `captures/classes/`; lower `--sat-min-floor` or widen `--hue-margin` |
| Dozens of false positives | Size gating too loose or altitude wrong | Check `--altitude-m` is correct; increase `--min-circularity` |
| Detections flicker | Low color ratio acceptance | Raise `--min-color-ratio`; check `--track-gate-px` |
| Green buoys detected as blue | Green range upper bound too low | Verify code uses 75–105; check `captures/classes/green/` crops |
