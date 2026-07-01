# Roadmap & Progress Tracking

Last updated: 2026-06-30

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
- [x] CLAHE on V channel before HSV thresholding — `apply_clahe_to_v()` called unconditionally in `camera_live_feed.py` main loop (line 570)
- [x] Documentation (`docs/`)

---

## In Progress 🔄

- [ ] YOLO model training pipeline — documentation written (`08_annotation_and_training.md`); code improvements pending (auto-validation, integrated train loop — see TODO #4)
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

**Status:** CLAHE is already done (see Completed above). Remaining gaps:

**Problem:** Different cameras, lighting angles, and exposure settings produce very different raw frames, making fixed HSV thresholds brittle.

**Remaining suggestions:**
- Auto white-balance normalization on each frame before HSV conversion
- Optional exposure normalization: histogram stretching on V channel
- Consider a standardization pass in `hsv_batch_detect.py` before the proposal step

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
- [x] `08_annotation_and_training.md` — YOLO annotation + training pipeline
- [x] `09_competition_day.md` — competition day cheat sheet

---

## Known bugs / technical debt

| Issue | File | Notes |
|-------|------|-------|
| Green HSV range: README says 75–99, code is 75–105 | `color_utils.py` line 21, `simulation/README.md` lines 230/244 | Fix README to say 75–105, or tighten code to 99 |
| `visualize_results.py` has hard-coded stats | `visualize_results.py` top constants | Should read from `captures/hsv_results/detections.csv` dynamically |
| No IMU attitude correction in GPS projection | `camera_live_feed.py` `project_pixel_to_ground_ned()` | Assumes perfect nadir; pitch/roll during flight adds lateral error |
| `--fx-px` default (1500) diverges from calibration (1319) | `camera_live_feed.py` arg default | Always pass `--calibration-file` flag; default is a fallback only |
| **`run_detection_jetson.sh` passes flags that don't exist in `camera_live_feed.py`** | `fulldemo/run_detection_jetson.sh` + `camera_live_feed.py` | Flags `--yolo-model`, `--gcs-ip`, `--save-video`, `--drone-lat`, `--drone-lon`, `--heading-deg`, `--headless` are planned for YOLO integration (TODO #2). Script will crash until then. Run `camera_live_feed.py` directly. |
| **`run_detection_jetson.sh` looks for `buoy_best.onnx`, not `.pt`** | `fulldemo/run_detection_jetson.sh` lines 11–19 | Export trained model with `model.export(format='onnx')` and copy as `buoy_best.onnx`; see `08_annotation_and_training.md` |
| **`jetson_setup.sh` does not install `ultralytics`** | `jetson_setup.sh` | Install manually: `pip install ultralytics` in `.venv-mavlink` after setup |

---

## Competition day checklist

- [ ] Collect 30–50 raw images per color (red, green, blue) at actual venue lighting
- [ ] Place one reference crop per color (named exactly `red.jpg`, `green.jpg`, `blue.jpg`) in both `captures/classes/` (for live detector) and `yolo_comparison_test/path2_switch_proposal/captures/classes/` (for training)
- [ ] Run `python augment_test.py` — check retention rate ≥ 70%
- [ ] Run `python hsv_batch_detect.py` — verify per-class detections in annotated images
- [ ] Run `python metrics_summary.py` — minimal false positives, minimal missed images
- [ ] Train new model: `01_autolabel.py` → `02_finetune.py` → validation steps
- [ ] Export ONNX and copy to Jetson as `buoy_best.onnx` (needed for `run_detection_jetson.sh` when TODO #2 is done)
- [ ] Set correct `--altitude-m` for your planned flight height
- [ ] Confirm network link (ping Jetson from Mac)
- [ ] Start GCS: `bash fulldemo/run_gcs_mac.sh` — wait for `Listening UDP 14555`
- [ ] Start detector on Jetson directly via `camera_live_feed.py` (see `09_competition_day.md`)
- [ ] Verify `[GPS]` lines appear when camera points at buoy
