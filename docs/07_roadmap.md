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
- [x] Augmentation smoke test (`augment_test.py` - blur + motion blur + glare hotspots)
- [x] Batch detection + metrics pipeline (`hsv_batch_detect.py`, `metrics_summary.py`, `visualize_results.py`)
- [x] One-command field demo scripts (`fulldemo/`)
- [x] Simulation accuracy report: 6/6 buoys detected, mean error 0.16 m, max 1.04 m (Course 1, 10 m AGL)
- [x] CLAHE on V channel before HSV thresholding - `apply_clahe_to_v()` in `camera_live_feed.py`, applied unconditionally on the HSV path; skipped on the YOLO path (`--yolo-model`) to avoid a distribution shift vs. the un-normalized images the model was fine-tuned on
- [x] Documentation (`docs/`)
- [x] YOLO fine-tuning + honest validation pipeline (`yolo_comparison_test/path2_switch_proposal/scripts/01_autolabel.py` → `02_finetune.py` → `validation_step1-5_*.py`); real checked-in weights, held-out mAP50 = 0.968
- [x] YOLO inference wired into `camera_live_feed.py` (`--yolo-model`, `--yolo-conf`) plus the rest of TODO #2's flags (`--gcs-ip`, `--save-video`, `--drone-lat`/`--drone-lon`, `--heading-deg`, `--headless`) - unblocks `fulldemo/run_detection_jetson.sh`. Not yet run live (no network in the dev sandbox that wrote it to install `ultralytics`) - verify with a real camera/model before field use.
- [x] `visualize_results.py` reads stats from `detections.csv` dynamically instead of a hard-coded run
- [x] Scan-the-Code light buoy color cycling in all 3 sim courses (`simulation/light_buoy_cycler.py`, launched by `run_course.sh`) - entity spawn/swap, because gz-sim Harmonic's `visual_config` material changes never reach the sensor render scene
- [x] Train/val split leakage fixed at the root: `00_preprocess_training_data.py` splits RAW captures before augmentation (recorded in `split_manifest.txt`); `01_autolabel.py`/`02_finetune.py`/`validation_step1` preserve the split
- [x] Sim-trained 3-class model (red/green/BLUE) with honest held-out validation: mAP50 0.991, overfit check HEALTHY - see `yolo_comparison_test/path2_switch_proposal/results_sim_courses_v2/README.md` including its caveats (weak-label GT errors, sim-only imagery)
- [x] `cv_bridge` NumPy-2 ABI segfault fixed: `camera_live_feed.py` and `accuracy_verify.py` decode rgb8 ROS images manually with numpy
- [x] Safe Passage UAV detectors (`docs/10_safe_passage.md`): P1 OFF/black buoy (`--detect-off-buoys`), P2 flashing-vs-solid track classifier (`--classify-flash`, the flashing-BLUE-entry vs steady-BLUE-exit discriminator), P3 online HSV EMA re-adaptation (`--online-recolor`). All default-OFF (shipped pipeline unchanged); validated on synthetic + real frames. P4 (YOLO 4th `black` class) is a documented stretch.
- [x] YOLO reject-only post-filters (`--yolo-size-gate`, `--yolo-min-circularity`) in `find_detections_yolo()` - default OFF, validated no-regression vs a fresh baseline (P0.922/R1.000/mAP50 0.994); the size gate at 10 m AGL was measured to crater recall to 0.404, hence off by default
- [x] `model/` retraining entry point (`model/README.md` + one-command `model/run_pipeline.sh`) - wraps the existing scripts with a fail-fast mAP50 gate; validated end-to-end (mAP50 0.994, overfit HEALTHY, stress retention 97%)

---

## In Progress 🔄

- [ ] Auto-validation in the training loop itself (see TODO #4 below) - the 5-step validation pipeline exists but is run manually, not integrated into `02_finetune.py`
- [ ] Real-photo fine-tune: the sim-trained 3-class model exists but all its data is Gazebo renders; real venue photos remain the missing ingredient before field use
- [ ] IMU pitch/roll compensation in GPS projection (yaw/heading is now handled via `--heading-deg`; pitch/roll is not)
- [ ] Autolabel quality audit: found systematic weak-label errors (double red+green boxes on one object, a missed fully-saturated red buoy in `results_sim_courses_v2` val, see its README) - model metrics measure agreement with these labels, so spot-fix labels or build a small human-verified test set
- [ ] Normalization mismatch: training images get Phase-2 color normalization but `camera_live_feed.py --yolo-model` feeds raw frames; apply the same normalization at inference or retrain without it
- [ ] Detector Stage-1 is grayscale-Canny only: a bright blue object on blue-green water produces no edge (measured in sim; same physics on real water where blue is already the weakest class) - a saturation-aware proposal channel would fix a real blind spot. **Partially addressed:** P1's dark-blob proposal (`--detect-off-buoys`) covers the low-*value* (dark / OFF-buoy) blind spot; the low-*saturation* blue-on-water case is still open.

---

## RobotX 2026 Handbook compliance notes

Cross-checked against the *2026 Maritime RobotX Challenge Team Handbook*
(rev. 2026-03-30), sections 3.3.1 / 3.3.2 / 3.4, fetched 2026-07-31 from the
GitBook mirror (`robonation.gitbook.io/robotx-2026-team-handbook`). Handbook is
treated as source of truth; no discrepancies found vs. our internal design.

### Known gap: RoboCommand reporting (not built)

Handbook 3.4 requires every SoS to report status to RoboNation's **RoboCommand**
over a hard-wired RJ-45 link, using **Protocol Buffers** (`.proto` schema →
`protoc` → compact binary). Our `mavlink_comms/` → MAVLink UDP → ground-station
path is the team's own *internal* telemetry; it is **not** the RoboCommand
channel.

> Don't build a speculative `.proto` integration yet - the message schema is
> unpublished ("Additional details to be provided in a future iteration of the
> Handbook"). Tracked here; wire it once RoboNation releases the schema.

### Safe Passage (3.3.2) - UAV perception scope

- The UAV's detection target is the **top-visible** beacon (nadir camera), a
  physically separate light from the side-visible one a surface boat sees.
- Five beacon states: OFF, flashing RED, flashing GREEN, flashing BLUE, steady
  BLUE. Flashing = 1 s on / 1 s off, repeating.
- Core Tier: UAV is **optional** (its perception earns the multi-vehicle
  collaboration bonus; side-visible RED/GREEN are lit so a USV could transit
  alone). UAV job at Core = detect / classify / report RED, GREEN, BLACK (OFF)
  buoys plus the BLUE entry/exit markers - **not** path planning.
- Flashing-BLUE (ENTRY) vs steady-BLUE (EXIT) differ only in temporal pattern,
  so the flash-state classifier (P2) is the required discriminator between them.
- Full plan: `docs/10_safe_passage.md`.

### Obstacle avoidance (3.3.1) - cross-cutting, not a scored task

"Every course element is an object to be avoided or approached safely";
distractor "obstacle buoys" may be scattered in the operating areas. Physical
avoidance is the USV/UUV's job. The UAV's buildable piece is **perception**: not
misclassifying distractors (olive panels, orange crates, gray barrels) as a
navigation-buoy colour.

**Measured (A3, 2026-07-31)** on the 65 real held-out course frames that contain
the sim distractors:

| Detector | On-buoy (TP) | Distractor/bg FP | Precision | Buoy recall |
|----------|-------------|------------------|-----------|-------------|
| YOLO (`best.pt`) | 94 | 8 (buoy-shaped: dup boxes / weak-label misses) | 0.922 | 1.00 |
| HSV two-stage | 75 | 5 (4 green ~olive, 1 red ~orange) | 0.938 | ~0.80 |

Distractors are overwhelmingly suppressed by both paths. Residual: the **olive
panels occasionally bleed into green** (the documented hue-boundary risk - olive
hue ~60-70 vs green 75-105), and one orange bled into red. No non-buoy
"obstacle" reporting channel exists in the pipeline, so none was added (out of
scope; physical avoidance is the USV's job). Harnesses: `tests/validation/gen_distractor_frames.py`, `tests/validation/test_distractor_suppression.py`.

---

## TODO - Priority order

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

**Status:** `--yolo-model`/`--yolo-conf` now exist and switch `camera_live_feed.py` onto a YOLO detection path at runtime (no code edits needed) - see Completed above. Swapping models is now a CLI flag, not a code change.

**Problem still open:** every run still needs its own long flag list; there's no saved preset.

**Suggestions (not yet implemented):**
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

- [x] `00_index.md` - index + quick-start cheat sheet
- [x] `01_environment_setup.md` - Mac, Ubuntu/WSL, Jetson setup
- [x] `02_data_pipeline.md` - capture → augment → detect → metrics
- [x] `03_detection_algorithm.md` - two-stage CV deep-dive
- [x] `04_gps_projection.md` - pixel → NED → GPS math + calibration
- [x] `05_simulation.md` - Gazebo SITL, 3 courses
- [x] `06_real_flight.md` - Jetson + GCS full demo
- [x] `07_roadmap.md` - this file
- [x] `08_annotation_and_training.md` - YOLO annotation + training pipeline
- [x] `09_competition_day.md` - competition day cheat sheet
- [x] `11_simulink_sensor_sim.md` - Simulink GPS/IMU noise injection layer

---

## Known bugs / technical debt

| Issue | File | Notes |
|-------|------|-------|
| ~~Green HSV range: README said 75–99, code is 75–105~~ | `simulation/README.md` lines 230/244 | **Fixed** - README now says 75–105 to match code. Newly noted: green (75–105) and blue (100–130) overlap between 100–105; see `03_detection_algorithm.md`. |
| ~~`visualize_results.py` has hard-coded stats~~ | `visualize_results.py` | **Fixed** - reads `--csv-path`/`--captures-dir` dynamically (mirrors `metrics_summary.py`'s CSV logic). The before/after-ROI comparison panel now needs an explicit `--before-roi <count>` (from a separate unfiltered run) since final `detections.csv` alone can't reconstruct pre-filter candidate counts; without it, the panel just shows the current after-filter count. |
| No IMU attitude correction in GPS projection | `camera_live_feed.py` `project_pixel_to_ground_ned()` | Still assumes perfect nadir; pitch/roll during flight adds lateral error. **Partially addressed**: the new `--heading-deg` rotates the projection for yaw (compass heading), which the sim never needed (flies yaw-locked at 0) but a real drone does. Pitch/roll compensation is still unimplemented. |
| ~~`--fx-px` legacy fallback (1500) diverges from calibration (1319)~~ | `camera_live_feed.py` `LEGACY_FX_PX`/`LEGACY_FY_PX` | **Fixed** - legacy fallback now matches the measured calibration (fx=1319.07, fy=1407.50) instead of a guessed 1500. Still always pass `--calibration-file` in normal use; this only matters if that file is missing. |
| ~~`run_detection_jetson.sh` passes flags that don't exist in `camera_live_feed.py`~~ | `fulldemo/run_detection_jetson.sh` + `camera_live_feed.py` | **Fixed** - `--yolo-model`, `--yolo-conf`, `--gcs-ip`, `--save-video`, `--drone-lat`/`--drone-lon` (aliases of `--origin-lat`/`--origin-lon`), `--heading-deg`, `--headless` (alias of `--no-display`) all now exist. `--yolo-model` swaps in a YOLO detection path (see `find_detections_yolo()`) in place of the two-stage HSV pipeline; `--gcs-ip` wires MAVLink STATUSTEXT TX via `mavlink_comms.transmitter.BuoyMavlinkTransmitter`. **Not yet verified with a live run** - this sandbox has no outbound network access, so `ultralytics`/`torch` couldn't be installed to smoke-test the YOLO path end-to-end; the code mirrors the exact `model(img, verbose=False)[0]` / `.boxes.conf/.cls/.xyxy` pattern already proven working in `02_finetune.py` and `validation_step3_val_inference.py`, but run it once for real before trusting it in the field. |
| **`run_detection_jetson.sh` looks for `buoy_best.onnx`, not `.pt`** | `fulldemo/run_detection_jetson.sh` lines 11–19 | Still applies - export trained model with `model.export(format='onnx')` and copy as `buoy_best.onnx`; see `08_annotation_and_training.md`. (The script already falls back to `.pt` demo weights if no `.onnx` is found, so it won't hard-fail, just run unoptimized.) |
| ~~`jetson_setup.sh` does not install `ultralytics`~~ | `jetson_setup.sh` | **Fixed** - step 3b now runs `pip install ultralytics`. Still uses a generic (non-JetPack) torch wheel; GPU-accelerated inference needs a manual JetPack-matched torch install first. |

---

## Competition day checklist

- [ ] Collect 30–50 raw images per color (red, green, blue) at actual venue lighting
- [ ] Place one reference crop per color (named exactly `red.jpg`, `green.jpg`, `blue.jpg`) in both `captures/classes/` (for live detector) and `yolo_comparison_test/path2_switch_proposal/captures/classes/` (for training)
- [ ] Run `python augment_test.py` - check retention rate ≥ 70%
- [ ] Run `python hsv_batch_detect.py` - verify per-class detections in annotated images
- [ ] Run `python metrics_summary.py` - minimal false positives, minimal missed images
- [ ] Train new model: `01_autolabel.py` → `02_finetune.py` → validation steps
- [ ] Export ONNX and copy to Jetson as `buoy_best.onnx` (`run_detection_jetson.sh` now picks it up automatically - verify this actually runs before relying on it on competition day)
- [ ] Set correct `--altitude-m` for your planned flight height
- [ ] Confirm network link (ping Jetson from Mac)
- [ ] Start GCS: `bash fulldemo/run_gcs_mac.sh` - wait for `Listening UDP 14555`
- [ ] Start detector on Jetson directly via `camera_live_feed.py` (see `09_competition_day.md`)
- [ ] Verify `[GPS]` lines appear when camera points at buoy
