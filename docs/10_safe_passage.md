# Safe Passage Task — Detection Implementation Plan

**RobotX 2026, Task 1 ("Safe Passage"), page 56.** Cross-checked against the
live *2026 Maritime RobotX Challenge Team Handbook* (rev. 2026-03-30), sections
3.3.1/3.3.2/3.4 - see `docs/07_roadmap.md` -> "RobotX 2026 Handbook compliance
notes" for the full compliance review and the RoboCommand known-gap. No
discrepancies found between this plan and the handbook text.

> **Status (2026-07-31): P1, P2, and P3 below are implemented and validated.**
> P4 remains a stretch goal. See the "Implementation Priority" and "Validation"
> sections at the end for what shipped, the CLI flags, and the real numbers.

---

## Task Requirements

The surface vehicle must navigate through a channel marked by buoys. The UAV's job is to detect and classify all buoys ahead of time and report their GPS positions so the surface vehicle can determine which side to pass them on.

| Light state | Color | Meaning |
|---|---|---|
| **Flashing** | Red | Starboard side of channel — surface vehicle passes on its **right** |
| **Flashing** | Green | Port side of channel — surface vehicle passes on its **left** |
| **Flashing** | Blue | Safe **entry** point to the channel |
| **Solid** | Blue | Safe **exit** point from the channel |
| **Off** | Black / unlit | Present, must be detected and classified as "off" |

The drone's output per-buoy is: `(lat, lon, color, light_mode)` where `light_mode ∈ {flashing, solid, off}`.

> **Dual-beacon note (handbook 3.3.2):** each buoy actually carries two
> physically separate beacons - one visible from the side (for surface craft)
> and one visible only from directly above (for the UAV). Our detection target
> is the top-visible beacon on the nadir camera.

---

## Current Capability vs Gaps

| Capability | Current state | Gap |
|---|---|---|
| Red / green detection | ✅ Working (YOLO mAP50=0.994, recall=0.989 - held-out, 65 val images) | None |
| Blue detection | ✅ Working (F1=1.000 on val set) | None |
| **Flashing vs solid** | ✅ Implemented (`--classify-flash`) | See Gap 1 + Validation |
| **Off / black buoys** | ✅ Implemented (`--detect-off-buoys`) | See Gap 2 + Validation |
| **Live color re-adaptation** | ✅ Implemented (`--online-recolor`) | See Gap 3 + Validation |

The three gaps below each need a distinct solution.

---

## Gap 1 — Flashing vs Solid Detection

### The physics

Flashing buoys pulse on/off at a fixed period (~1 Hz for RobotX lights; the
handbook specifies 1 s on / 1 s off). A camera recording at ≥15 fps will see
multiple ON frames and multiple OFF frames within one full cycle. A solid buoy
is always ON.

The drone camera (`camera_live_feed.py`) already produces per-detection
confidence values at every frame. All that is needed is a temporal accumulator
per track to count ON vs OFF frames.

### Proposed implementation

**Add a `TrackFlashState` to the Kalman tracking layer** (`camera_live_feed.py`). Each track already lives for many frames. Augment it with a rolling ON/OFF observation window and classify by on-ratio + toggle count.

> **Implemented as `TrackFlashState`** (`camera_live_feed.py`), attached to each
> `Track` via `flash: TrackFlashState`. `update_tracks()` calls `observe(True)`
> when a track matches a detection and `observe(False)` when it's alive but
> unmatched. `classify(min_frames, min_toggles, solid_ratio)` returns
> `solid` / `flashing` / `unknown`. Enabled via `--classify-flash`; also raises
> the track's missed-frame budget (`--flash-max-missed`, default 25) so a
> flashing track survives its ~1 s dark phase instead of being dropped.
>
> Shipped constants differ slightly from the numbers sketched below, tuned
> against the actual window size: `--flash-window 60` (same), `--flash-min-toggles 2`
> (proposal: 3), `--flash-solid-ratio 0.85` (proposal: 0.90). `flash_state` is
> written as a new `detections.csv` column (not a separate `light_mode` field -
> keeps the existing red/green/blue/black `color` column separate from the
> flash state axis).

**Integration points:**

- In `update_tracks()`: for each active track, increment `on_frames` if it received a detection this frame, `off_frames` if it did not.
- Count `toggle_count` each time the per-frame detection presence flips.
- Add `flash_mode` to the MAVLink STATUSTEXT payload and to `detections.csv`.

> MAVLink STATUSTEXT payload extension (`mavlink_comms/`) is not yet done -
> `flash_state` currently only reaches `detections.csv`. Tracked as a follow-up.

> **Classifier robustness (hardened):** `classify()` is duty-cycle-first, so a
> solid light that drops a few frames to motion blur/occlusion still reads
> `solid` (not thrown to `unknown` by the stray toggles), and a mostly-dark
> track reads `off`. `flash_state` values are now `{solid, flashing, off,
> unknown}`. Detecting a 1 s/1 s flash needs `--flash-window` to span >= ~2
> flash periods (>= ~4*fps frames): the default 60 covers ~15 fps; use 90-120
> for 30 fps. A window too small for the framerate reads a real flash as
> `unknown` on purpose - one period is indistinguishable from a single on->off
> transition. Covered by `tests/validation/test_flash_scenarios.py` (11
> scenarios: framerates 10/15/30 fps, dropouts, off, entry-vs-exit).

**Sim validation:** Add a `--flash-hz N` flag to `simulation/light_buoy_cycler.py` and run on all three courses. Verify `accuracy_verify.py` reports `flashing` for the light buoy and `solid` for gate buoys.

> Not done - `light_buoy_cycler.py` still only models the older red→green→blue
> "Scan the Code" cycle, not a `--flash-hz` flag or the solid/off states. The
> Gazebo/ArduPilot SITL/ROS 2 stack was not runnable in the environment that
> built P1-P3, so validation used synthetic frames built to the sensor-real HSV
> values documented in `light_buoy_cycler.py` instead - see Validation below.
> Wiring `--flash-hz` into the cycler + an `accuracy_verify.py` `light_mode`
> column remains open.

---

## Gap 2 — Black / Off Buoy Detection

### Why the current pipeline misses them

Both the HSV Stage-1 and the YOLO model treat low-saturation objects as background. The HSV pipeline applies:

```
S_min = sat_floor (default 50)
V_min = val_floor (default 45)
```

A black, unlit buoy sits at S≈0, V<80 — well below both floors. Stage-1 proposals are generated from Canny edges; a dark buoy on dark water is also low-contrast, producing few edges.

### Proposed implementation

**Two-stage approach: shape first, color second.**

#### Step A — Saturation-blind shape proposals for dark objects

Add a parallel proposal channel that does **not** require high saturation, using an adaptive threshold instead of Canny so it finds dark blobs on dark water.

> **Implemented as `find_off_buoys()`** (`camera_live_feed.py`): adaptive
> threshold (`cv2.adaptiveThreshold`, `ADAPTIVE_THRESH_MEAN_C`) on the value
> channel proposes dark regions, filtered by the same size/circularity gates
> `find_detections()` already uses. Block size / C constant are exposed as
> `--dark-block` (default 51) / `--dark-c` (default 10).

#### Step B — Off-buoy color classification

For each dark-blob candidate, classify it as "off" if it is dark **and**
achromatic (low saturation).

> **Implemented as `is_off_buoy()`** (`color_utils.py`): requires **both**
> median saturation ≤ `--off-sat-max` (default 60) **and** median value ≤
> `--off-val-max` (default 80) - both floors, not just one, so a deliberately
> dark-but-saturated light (e.g. the sim's dark blue beacon, grayscale ~20 yet
> highly saturated) is never misread as OFF. Uses **median** over the ROI
> rather than the mean sketched above, since a mean is more sensitive to a few
> bright edge pixels dragging a genuinely dark blob's average up.
>
> Reject candidates where any color (red/green/blue) mask exceeds
> `--min-color-ratio` — those are handled by the existing pipeline, not the
> dark-blob path. **Implemented**: a saturation-verified `black` detection
> overrides a weak overlapping colour detection at the same blob (see
> `camera_live_feed.py` `main()` dedup logic) rather than being silently
> discarded, since `is_off_buoy()` already ruled out a genuinely lit buoy.

#### Step C — YOLO path for off buoys

Retrain the YOLO model with a fourth class: `black`.

> **Not implemented (P4, stretch).** P1's HSV-path dark-blob detector covers
> OFF detection today; adding a 4th YOLO class is deferred so it doesn't block
> P1-P3. Would need: reference crop `captures/classes/black.jpg`, an
> `is_off_buoy()` branch in `01_autolabel.py`'s `label_image()`, and `nc: 4` /
> `names: [red, green, blue, black]` in `dataset.yaml`.

---

## Gap 3 — Adaptation to Changes in Buoy Color

### When this matters

Two scenarios:

1. **Scan-the-Code style:** A buoy cycles through red → green → blue (the light buoy already does this). The detector needs to report the current color per frame, not a fixed one from initialization.

2. **HSV drift:** A buoy that was detected as "red" in low morning light may drift toward orange or even green at noon as ambient color temperature shifts. The reference crops captured at setup time may no longer match in-flight.

### Scenario 1 — Cycling buoys (already partially handled)

The light buoy in all three sim courses already cycles. `camera_live_feed.py` re-classifies every frame independently, so successive frames will correctly report different colors as the buoy cycles. No code change needed for single-frame classification.

> Resetting `TrackFlashState` on a colour change (so a flash-state count from
> the previous colour phase doesn't bleed into the next) is not yet wired in;
> today a colour change creates a new track via `update_tracks()`'s existing
> "unmatched detection -> new track" path (different colour = no match against
> the old track's `det.color != track.color` gate), which has the same net
> effect - fresh `TrackFlashState` - but via track replacement rather than an
> explicit reset. Worth confirming this is equivalent under real flight noise.

### Scenario 2 — HSV drift / lighting shift

#### Option A — Periodic online re-calibration (recommended for HSV mode)

Every N frames, blend each colour's hue centre toward what's actually being
observed in high-confidence detections, using a circular EMA so it handles the
0°/180° hue wrap correctly.

> **Implemented as `OnlineColorAdapter`** (`camera_live_feed.py`), enabled via
> `--online-recolor` (HSV path only). Per-colour hue EMA (`--recolor-alpha`,
> default 0.1) is fed from detections at or above `--recolor-min-conf` (default
> 0.5), measured over the ROI's saturated pixels (not just pixels already
> inside the current range, so it can follow drift past the current bounds).
> `COLOR_RANGES` is rebuilt every `--recolor-interval` frames (default 10) via
> `color_utils.make_ranges_for_hue`. Blending is done along the shortest hue
> arc (same wrap-safe idea as the `blend_hue()` sketch below), just
> implemented as a full circular-mean blend rather than a fixed±90° diff.

#### Option B — Retrain YOLO in the field (for major lighting shifts)

The competition day pipeline already supports rapid re-capture + retrain.

> **Implemented and generalized as `model/`** (`model/README.md` +
> `model/run_pipeline.sh`) - a guided, one-command wrapper around exactly this
> re-capture + retrain flow (preprocess → autolabel → retrain → validate →
> ONNX export), with a fail-fast held-out mAP50 gate. `model/fetch_weights.sh`
> also lets a teammate pull the last validated weights from a GitHub Release
> without retraining at all. See `model/README.md`.

Option A (online re-calib) should handle day-of drift without a full retrain. Option B is the fallback when that is not enough.

---

## End-to-End Output Format

After all three gaps are closed, each buoy report to the ground station (MAVLink STATUSTEXT and `detections.csv`) should carry:

```
color       ∈ { red, green, blue, black }
light_mode  ∈ { flashing, solid, off, unknown }
lat, lon    (GPS projected from camera + calibration)
confidence  (0–1, existing formula)
```

> **Current shipped shape:** `color ∈ {red, green, blue, black}` (black from P1)
> and a separate `flash_state ∈ {flashing, solid, off, unknown}` CSV column (from
> P2) - not yet folded into a single combined `light_mode` field, and not yet
> forwarded over MAVLink STATUSTEXT (see Gap 1 follow-up above). The GPS/
> confidence fields are unchanged from the existing pipeline.

The surface vehicle autonomy node consumes this and applies the RobotX port/starboard convention:

| color | light_mode | Action |
|---|---|---|
| red | flashing | Pass on starboard |
| green | flashing | Pass on port |
| blue | flashing | This is the channel entry — approach it |
| blue | solid | This is the channel exit — you are through |
| any | off | Obstacle — avoid |

---

## Implementation Priority

| Priority | Item | Effort | Blocking? | Status |
|---|---|---|---|---|
| **P1** | Off-buoy detection (adaptive threshold + dark-blob class) | ~1 day | Yes — course may have off buoys | ✅ Done - `--detect-off-buoys` |
| **P2** | Flashing vs solid classifier (temporal accumulator) | ~1 day | Yes — required to distinguish entry/exit | ✅ Done - `--classify-flash` |
| **P3** | Color re-adaptation (online HSV EMA) | ~2 hours | No — fallback is re-run with new reference crops | ✅ Done - `--online-recolor` |
| **P4** | YOLO fourth class `black` | ~4 hours + data collect | No — HSV dark-blob path covers it for now | Not started (stretch) |

---

## Files to Modify

| File | Change | Status |
|---|---|---|
| `camera_live_feed.py` | Add `TrackFlashState`, dark-blob proposal channel, `find_off_buoys()`, `OnlineColorAdapter`, online recalib interval flag | ✅ Done |
| `color_utils.py` | Add `is_off_buoy()`; `make_ranges_for_hue()`/`circular_hue_mean()` already existed and are reused by the EMA rebuild | ✅ Done |
| `simulation/light_buoy_cycler.py` | Add `--flash-hz` flag; expose a "solid"/"off" mode so gate buoys can be marked for testing | Not started |
| `yolo_comparison_test/path2_switch_proposal/scripts/01_autolabel.py` | Add `black` class; add `is_off_buoy()` branch to `label_image()` | Not started (P4) |
| `simulation/accuracy_verify.py` | Add `light_mode` column to accuracy report; verify against ground-truth mode per buoy | Not started |
| `docs/00_index.md` | Add this file to the index table | ✅ Done |
| `mavlink_comms/` | Forward `flash_state`/`black` color over MAVLink STATUSTEXT | Not started |

---

## Validation

The full Gazebo Harmonic + ArduPilot SITL + ROS 2 course (`simulation/`,
`light_buoy_cycler.py`) is the eventual end-to-end check for P1-P3 (and the
only way to validate the still-open `--flash-hz` / `accuracy_verify.py` items
above), but that stack was not runnable in the environment that built P1-P3
and the beacon states here (OFF, flashing, steady) are not yet modelled by the
cycler. Each priority was therefore validated against **synthetic frames built
to the sensor-real HSV values** documented in `simulation/light_buoy_cycler.py`
(red hue ~0, green ~86, blue ~114 dark, water grayscale ~103), driving the
*actual* detection/tracking code, plus real held-out course frames where noted:

- **P1 (off-buoy):** direct test of `find_off_buoys()` - a dark OFF blob
  produces exactly one `black` detection; a deliberately dark-but-saturated
  blue light is correctly NOT read as black; plain water produces zero false
  positives. On 30 real course frames (lit buoys, no OFF buoys present),
  `--detect-off-buoys` produced zero black false positives with red/green
  detection intact. See `tests/validation/test_off_buoy_detection.py`.
- **P2 (flash classifier):** a 90-frame synthetic sequence with one
  always-lit blob and one 15-on/15-off blob (~1 s/1 s at 15 fps) - the solid
  blob's track ends `flash_state=solid`, the flashing blob's ends
  `flash_state=flashing`. See `tests/validation/gen_flash_classifier_frames.py`.
- **P3 (online recolor):** unit test confirms the hue EMA follows an observed
  90→114 drift to 109.4 and the rebuilt range re-centres correctly; an
  end-to-end run through `main()` shows the green EMA move from its seed (90)
  to an observed hue of 98, proving the observe→rebuild→detect loop fires from
  real detections. See `tests/validation/test_online_recolor.py`.
- **YOLO gates (validated alongside these, not itself a Safe Passage item):**
  reject-only size/circularity gates for the YOLO path, default off - see
  `tests/validation/test_yolo_size_gate.py` and
  `tests/validation/yolo_gate_diagnostic.py`, referenced from
  `docs/07_roadmap.md`.

All three (P1-P3) default OFF; the shipped detection pipeline is unchanged
unless the corresponding flag is passed.
