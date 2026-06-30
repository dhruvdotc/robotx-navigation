# Roadmap & Progress Tracking

Last updated: 2026-06-28

---

## Milestone: Competition-Ready Pipeline

Goal: full end-to-end cycle (capture → annotate → train → deploy → test) runnable from scratch on competition day in new lighting conditions.

---

## Completed ✅

- [x] Two-stage HSV detector (`camera_live_feed.py`, `hsv_batch_detect.py`)
- [x] Kalman tracking across frames
- [x] GPS projection: pixel → NED → lat/lon (`project_pixel_to_ground_ned`, `ned_to_gps`)
- [x] Camera calibration (fx=1319, fy=1407, RMS=1.057 px, 40-frame checkerboard)
- [x] MAVLink buoy report protocol + UDP ground station (`mavlink_comms/`)
- [x] Full Gazebo Harmonic SITL stack (3 courses: straight, lawnmower, L-shaped dogleg)
- [x] Distractor obstacles in all sim courses (olive panels, orange crates, gray barrels/panels)
- [x] Augmentation smoke test (`augment_test.py` — blur + motion blur + glare hotspots)
- [x] Batch detection + metrics pipeline (`hsv_batch_detect.py`, `metrics_summary.py`, `visualize_results.py`)
- [x] One-command field demo scripts (`fulldemo/`)
- [x] Simulation accuracy report: 6/6 buoys detected, mean error 0.16 m, max 1.04 m (Course 1, 10 m AGL)
- [x] Documentation (`docs/`)

---

## In Progress 🔄

- [ ] YOLO model training pipeline — need to document end-to-end train loop with `ultralytics`
- [ ] `visualize_results.py` — currently has hard-coded stats from 6/26 run; needs to read from CSV dynamically

---

## TODO — Priority order

### 1. Improve augmentation pipeline
**File:** `augment_test.py` / `apply_uav_noise()`

Current augmentations: Gaussian blur, random-angle motion blur (13×13 kernel), 3 random glare hotspots, Gaussian pixel noise.

**Gaps:**
- Glare hotspots are circular Gaussian; real sun glints on water are streak-shaped
- No water-surface reflection simulation
- No altitude-dependent blur scaling (blur should increase with altitude)
- No hue/saturation shift to simulate different times of day

**Suggested additions:**
- Streak glare: apply motion-blur-like kernel along sun direction for hotspots
- Saturation jitter: ±15 on S channel to simulate overcast vs sunny
- Altitude-aware blur: scale Gaussian σ proportionally to `altitude_m / 10`
- White-balance shift: random color temperature offset (warm/cool)

---

### 2. Flexible model weights + HSV thresholds

**Problem:** swapping to a new `best.pt` or changing HSV thresholds requires editing code or passing long flags.

**Suggestions:**
- Add a config JSON (`config/detection_config.json`) that specifies `model_path`, `yolo_conf`, `hsv_ranges`, `min_color_ratio`, `altitude_m`, `target_diameter_m`
- `camera_live_feed.py` and `run_detection_jetson.sh` read from config by default; command-line flags override
- Makes it easy to version-control different tuning presets (e.g. `config/sunny_day.json`, `config/overcast.json`)

---

### 3. Preprocessing / standardizing filters

**Problem:** no input normalization before detection. Different cameras, lighting angles, and exposure settings produce very different raw frames, making fixed HSV thresholds brittle.

**Suggestions:**
- CLAHE on the V channel before HSV thresholding (already implemented as `apply_clahe_to_v()` in `camera_live_feed.py` — wire it in by default)
- Auto white-balance normalization on each frame before HSV conversion
- Optional exposure normalization: histogram stretching on V channel
- Consider a standardization pass in `hsv_batch_detect.py` before the proposal step

**Note:** `apply_clahe_to_v()` already exists in `camera_live_feed.py` (line 504) but is not called in the main detection loop. Easiest short-term win: add a `--clahe` flag to enable it.

---

### 4. Auto-validation in training loop

**Problem:** training YOLO currently requires manual inspection to pick the best checkpoint. Need automatic selection based on per-class mAP on a held-out validation set.

**Suggestions:**
- Hold out 20% of captured images as a validation set (stratified by color class)
- Run `model.val()` at the end of each epoch on the validation set
- Track per-class mAP (mAP@0.5) for red, green, blue separately
- Save checkpoint only when the harmonic mean of per-class mAPs improves
- Write a `training/train.py` wrapper that does this automatically and saves `best_validated.pt`

---

### 5. Documentation (this folder)

- [x] `00_index.md` — index + quick-start cheat sheet
- [x] `01_environment_setup.md` — Mac, Ubuntu/WSL, Jetson setup
- [x] `02_data_pipeline.md` — capture → augment → detect → metrics
- [x] `03_detection_algorithm.md` — two-stage CV deep-dive
- [x] `04_gps_projection.md` — pixel → NED → GPS math + calibration
- [x] `05_simulation.md` — Gazebo SITL, 3 courses
- [x] `06_real_flight.md` — Jetson + GCS full demo
- [x] `07_roadmap.md` — this file
- [ ] `08_training.md` — YOLO training pipeline (once #4 is done)

---

## Known bugs / technical debt

| Issue | File | Notes |
|-------|------|-------|
| Green HSV range: README says 75–99, code is 75–105 | `color_utils.py` line 21, `simulation/README.md` lines 230/244 | Fix README to say 75–105, or tighten code to 99 |
| `visualize_results.py` has hard-coded stats | `visualize_results.py` top constants | Should read from `captures/hsv_results/detections.csv` dynamically |
| No IMU attitude correction in GPS projection | `camera_live_feed.py` `project_pixel_to_ground_ned()` | Assumes perfect nadir; pitch/roll during flight adds lateral error |
| `--fx-px` default (1500) diverges from calibration (1319) | `camera_live_feed.py` arg default | Always pass `--calibration` or fix default to match calibration JSON |

---

## Competition day checklist

- [ ] Collect 20+ raw images per color (red, green, blue) at actual venue lighting
- [ ] Place reference crops in `captures/classes/{red,green,blue}/`
- [ ] Run `python augment_test.py` — check retention rate ≥ 70%
- [ ] Run `python hsv_batch_detect.py` — verify per-class detections look correct in annotated images
- [ ] Run `python metrics_summary.py` — zero false positives, zero missed images
- [ ] Update `buoy_best.pt` on Jetson if a new model was trained
- [ ] Run `bash simulation/run_course.sh --course 1` — verify sim still passes
- [ ] Set correct `--altitude-m` (measure AGL at your planned flight height)
- [ ] Confirm network link (ping Jetson from Mac)
- [ ] Test full pipeline end-to-end with `fulldemo/` scripts before competition
