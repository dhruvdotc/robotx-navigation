# Data Pipeline

Covers the full cycle: capture raw images → augment → batch-detect → review metrics.
This is the loop to run every time you collect new images at a field session or want to re-tune HSV thresholds.

---

## Directory layout expected by the scripts

```
captures/
├── *.jpg                      # raw captured frames (flat, no subdirs)
├── classes/
│   ├── red.jpg                # one reference crop of a red buoy (exact filename)
│   ├── green.jpg              # one reference crop of a green buoy
│   └── blue.jpg               # one reference crop of a blue buoy
└── hsv_results/
    ├── detections.csv         # output: per-detection rows
    ├── annotated/             # output: annotated JPGs per input frame
    ├── augmentation_test.jpg  # output: 2×2 augmentation grid
    └── metrics.png            # output: bar chart
```

---

## Step 1 — Capture images

Point the camera at the target scene and press Spacebar to save frames.

```bash
python camera_capture_spacebar.py \
    --camera-index 0 \
    --output-dir captures \
    --prefix capture
```

Hotkeys: `Space` saves a frame, `q` quits.

**At competition:** point at each buoy color (red, green, blue) and save 10–20 clean frames per color. Also save frames with distractors visible.

### Organize class references

After capturing, hand-pick **one tight, well-lit crop per color** and save them as:
- `captures/classes/red.jpg`
- `captures/classes/green.jpg`
- `captures/classes/blue.jpg`

The filename stem must be exactly `red`, `green`, or `blue` — `color_utils.derive_class_hsv_ranges()` reads the color name from the filename. Other filenames in `classes/` are silently ignored.

These are used by `hsv_batch_detect.py` and `camera_live_feed.py` to automatically derive HSV ranges for that day's lighting. If `captures/classes/` is empty or missing, the scripts fall back to the hard-coded fallback ranges in `color_utils.py`.

---

## Step 2 — Augmentation smoke test

Run before batch-detecting to see how the detector holds up under UAV-realistic noise.

```bash
python augment_test.py                        # uses first .jpg in captures/
python augment_test.py captures/my_image.jpg  # specific file
```

Saves `captures/hsv_results/augmentation_test.jpg` — a 2×2 grid:
- top-left: clean image + detections
- top-right / bottom-left / bottom-right: three independently noise-augmented variants

The noise model (`apply_uav_noise` in `augment_test.py`) applies:
1. Gaussian blur (σ=2.5) — simulates motion smear
2. Random-angle motion blur (13×13 kernel) — simulates propwash direction
3. 3 random glare hotspots (Gaussian falloff, +65 V channel) — simulates sun glints
4. Gaussian pixel noise (σ=8)

Also prints avg retention rate (how many detections survive the noise on average).

**What to look for:** retention rate above ~70% is healthy. If it tanks, the HSV ranges may need widening or the V-channel floor needs lowering.

---

## Step 3 — Batch detection

Runs the full two-stage pipeline over every image in `captures/` and writes annotated output + CSV.

```bash
python hsv_batch_detect.py \
    --captures-dir captures \
    --classes-dir captures/classes \
    --out-dir captures/hsv_results
```

Key tuning flags:

| Flag | Default | Effect |
|------|---------|--------|
| `--hue-margin` | 12 | ±hue margin around the reference median when deriving ranges |
| `--sat-min-floor` | 50 | Minimum saturation for any detected pixel |
| `--val-min-floor` | 45 | Minimum value (brightness) for any detected pixel |
| `--min-proposal-area` | 900 | Minimum contour area to consider (px²) |
| `--min-color-ratio` | 0.07 | Fraction of ROI pixels that must match the winning color |
| `--min-circularity` | 0.08 | Contour shape filter (0=any, 1=perfect circle) |
| `--min-solidity` | 0.50 | Contour convex-hull fill ratio |
| `--nms-iou` | 0.45 | Non-max suppression IoU threshold |

**Outputs:**
- `captures/hsv_results/detections.csv` — one row per detection with `image, color, confidence, cx, cy, x, y, w, h, area`
- `captures/hsv_results/annotated/*.jpg` — original images with bounding boxes drawn

---

## Step 4 — Review metrics

```bash
python metrics_summary.py
```

Prints to stdout:
- Images processed / total detections
- Per-class: count, avg/min/max confidence
- False positive count (conf < 0.25)
- Images with zero detections (potential misses)
- A one-liner "presentation summary" you can copy into slides

Also saves `captures/hsv_results/metrics.png` — a horizontal bar chart.

```bash
python visualize_results.py
```

Generates `captures/hsv_results/results_diagram.png` — a full presentation-ready panel with detection counts, bar chart, confidence ranges, and before/after ROI comparison.

> **Note:** `visualize_results.py` currently has hard-coded numbers (from the 6/26 run). Update the constants at the top of the file after each new batch run.

---

## Full pipeline one-liner (for a quick field re-run)

```bash
python camera_capture_spacebar.py && \
python augment_test.py && \
python hsv_batch_detect.py && \
python metrics_summary.py
```

---

## Outstanding TODOs (see [07_roadmap.md](07_roadmap.md))

- **Augmentation:** add more realistic glare patterns, water reflections, varying altitude blur scale
- **Flexibility:** make it easy to swap in new `best.pt` weights and new HSV thresholds without editing code
- **Preprocessing filters:** standardize input frames (white-balance, exposure normalization) before detection
- **Auto-validation:** pick best model checkpoint automatically during training based on per-class mAP
