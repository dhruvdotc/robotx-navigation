# Safe Passage Task — Detection Implementation Plan

**RobotX 2026, Task 1 ("Safe Passage"), page 56**

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

---

## Current Capability vs Gaps

| Capability | Current state | Gap |
|---|---|---|
| Red / green detection | ✅ Working (YOLO mAP50=0.995, recall=1.0) | None |
| Blue detection | ✅ Working (F1=1.000 on val set) | None |
| **Flashing vs solid** | ❌ Not implemented | Detector sees one frame at a time; no temporal classifier |
| **Off / black buoys** | ❌ Not implemented | HSV S-min gate discards all near-gray detections |
| **Live color re-adaptation** | ❌ Not implemented | HSV thresholds and YOLO weights are fixed at launch |

The three gaps below each need a distinct solution.

---

## Gap 1 — Flashing vs Solid Detection

### The physics

Flashing buoys pulse on/off at a fixed period (~1 Hz for RobotX lights). A camera recording at ≥15 fps will see multiple ON frames and multiple OFF frames within one full cycle. A solid buoy is always ON.

The drone camera (`camera_live_feed.py`) already produces per-detection confidence values at every frame. All that is needed is a temporal accumulator per track to count ON vs OFF frames.

### Proposed implementation

**Add a `FlashClassifier` to the Kalman tracking layer** (`camera_live_feed.py`). Each track already lives for many frames. Augment it with:

```python
@dataclass
class TrackFlashState:
    on_frames: int = 0          # frames this track produced a detection
    off_frames: int = 0         # frames this track was in frame but produced no detection
    last_toggle_frame: int = 0
    flash_mode: str = "unknown" # "flashing" | "solid" | "off"
    toggle_count: int = 0       # number of ON→OFF or OFF→ON transitions observed
```

**Logic (run at the end of each frame's tracking update):**

```python
FLASH_WINDOW_FRAMES = 60       # look back over 60 frames (~4 s at 15 fps)
FLASH_MIN_TOGGLES   = 3        # must see ≥3 ON↔OFF transitions to call "flashing"
SOLID_ON_RATIO_MIN  = 0.90     # ≥90% ON frames → call "solid"
OFF_ON_RATIO_MAX    = 0.05     # ≤5% ON frames → call "off"

def classify_flash(state: TrackFlashState) -> str:
    total = state.on_frames + state.off_frames
    if total < 15:
        return "unknown"   # not enough data yet
    on_ratio = state.on_frames / total
    if on_ratio >= SOLID_ON_RATIO_MIN:
        return "solid"
    if on_ratio <= OFF_ON_RATIO_MAX:
        return "off"
    if state.toggle_count >= FLASH_MIN_TOGGLES:
        return "flashing"
    return "unknown"
```

**Integration points:**

- In `update_tracks()`: for each active track, increment `on_frames` if it received a detection this frame, `off_frames` if it did not.
- Count `toggle_count` each time the per-frame detection presence flips.
- Add `flash_mode` to the MAVLink STATUSTEXT payload and to `detections.csv`.

**Sim validation:** Add a `--flash-hz N` flag to `simulation/light_buoy_cycler.py` and run on all three courses. Verify `accuracy_verify.py` reports `flashing` for the light buoy and `solid` for gate buoys.

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

Add a parallel proposal channel that does **not** require high saturation:

```python
def propose_dark_blobs(gray_frame, expected_diameter_px, min_circularity=0.35):
    """
    Propose candidates for dark, low-contrast objects.
    Uses adaptive threshold instead of Canny so it finds dark blobs on dark water.
    """
    # Adaptive Gaussian threshold on the grayscale frame
    block = max(11, int(expected_diameter_px * 1.5) | 1)  # must be odd
    binary = cv2.adaptiveThreshold(
        gray_frame, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        blockSize=block, C=4
    )
    # Morphological close to fill gaps
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 8:
            continue
        perim = cv2.arcLength(cnt, True)
        if perim == 0:
            continue
        circ = 4 * np.pi * area / (perim ** 2)
        if circ < min_circularity:
            continue
        diameter = 2 * np.sqrt(area / np.pi)
        if not (0.4 * expected_diameter_px <= diameter <= 2.5 * expected_diameter_px):
            continue
        candidates.append(cnt)
    return candidates
```

#### Step B — Off-buoy color classification

For each dark-blob candidate, classify it as "off" if:

```python
def is_off_buoy(hsv_roi) -> bool:
    """
    Return True if the ROI is dark and achromatic (unlit / off buoy).
    """
    s_mean = float(hsv_roi[:, :, 1].mean())
    v_mean = float(hsv_roi[:, :, 2].mean())
    # Low saturation, moderate-to-dark value → unlit buoy
    return s_mean < 40 and v_mean < 120
```

Reject candidates where any color (red/green/blue) mask exceeds `--min-color-ratio` — those are handled by the existing pipeline, not the dark-blob path.

#### Step C — YOLO path for off buoys

Retrain the YOLO model with a fourth class: `black`. Capture reference images of the unlit buoy in `captures/classes/black.jpg` and add `black` to the autolabel logic in `01_autolabel.py`:

```python
# In 01_autolabel.py label_image(), after the red/green/blue checks:
elif is_off_buoy(hsv_roi):
    color = "black"
    class_id = 3
```

Add `black` to `dataset.yaml`:

```yaml
names:
  0: red
  1: green
  2: blue
  3: black
```

Collect 30–50 images of unlit buoys at the venue. The YOLO path will then predict `black` directly alongside bounding box.

---

## Gap 3 — Adaptation to Changes in Buoy Color

### When this matters

Two scenarios:

1. **Scan-the-Code style:** A buoy cycles through red → green → blue (the light buoy already does this). The detector needs to report the current color per frame, not a fixed one from initialization.

2. **HSV drift:** A buoy that was detected as "red" in low morning light may drift toward orange or even green at noon as ambient color temperature shifts. The reference crops captured at setup time may no longer match in-flight.

### Scenario 1 — Cycling buoys (already partially handled)

The light buoy in all three sim courses already cycles. `camera_live_feed.py` re-classifies every frame independently, so successive frames will correctly report different colors as the buoy cycles. No code change needed for single-frame classification.

**What to add:** The temporal accumulator from Gap 1 needs to be reset on color change:

```python
if current_detection.color != track.last_color:
    # New color phase — reset flash state but keep the track alive
    track.flash_state = TrackFlashState()
    track.last_color = current_detection.color
```

Report the **most recent stable color** (where "stable" means seen for ≥5 consecutive frames at that color) rather than the instantaneous per-frame color, to avoid noise from transition frames.

### Scenario 2 — HSV drift / lighting shift

#### Option A — Periodic online re-calibration (recommended for HSV mode)

Add an `--online-recalib-interval N` flag. Every N seconds, for each currently-tracked buoy color:

1. Crop the ROI of all high-confidence detections of that color from the last 30 frames.
2. Re-run `derive_class_hsv_ranges()` on those crops (same logic as the reference-crop startup path).
3. Blend the new range with the existing range (exponential moving average on hue center, keep margin fixed):

```python
ALPHA = 0.15   # how quickly to track drift; 0 = never update, 1 = instant

def blend_hue(current_center: int, new_center: int, alpha: float = ALPHA) -> int:
    # Circular blend to handle the 0°/180° wrap
    diff = ((new_center - current_center + 90) % 180) - 90
    return int((current_center + alpha * diff) % 180)
```

This keeps the thresholds chasing reality without jumping on a single outlier frame.

#### Option B — Retrain YOLO in the field (for major lighting shifts)

The competition day pipeline already supports rapid re-capture + retrain. If lighting at the venue is drastically different from training data:

```bash
# 1. Capture 30 new images per color at current lighting
python3 camera_capture_spacebar.py --camera-index 0 --output-dir captures_venue

# 2. Re-run auto-label + fine-tune (20 min on Mac)
python 01_autolabel.py
python 02_finetune.py

# 3. Deploy new best.pt over the old one on the Jetson
scp best.pt babydragon@<JETSON_IP>:~/robotx-navigation/buoy_best.pt

# 4. Restart detector with the new weights
python3 camera_live_feed.py --yolo-model buoy_best.pt ...
```

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

| Priority | Item | Effort | Blocking? |
|---|---|---|---|
| **P1** | Off-buoy detection (adaptive threshold + dark-blob class) | ~1 day | Yes — course may have off buoys |
| **P2** | Flashing vs solid classifier (temporal accumulator) | ~1 day | Yes — required to distinguish entry/exit |
| **P3** | Color re-adaptation (online HSV EMA) | ~2 hours | No — fallback is re-run with new reference crops |
| **P4** | YOLO fourth class `black` | ~4 hours + data collect | No — HSV dark-blob path covers it for now |

---

## Files to Modify

| File | Change |
|---|---|
| `camera_live_feed.py` | Add `TrackFlashState`, `FlashClassifier`, dark-blob proposal channel, `is_off_buoy()`, online recalib interval flag |
| `color_utils.py` | Expose `blend_hue()`, extend `derive_class_hsv_ranges()` to accept a list of ROI arrays (not just file paths) |
| `simulation/light_buoy_cycler.py` | Add `--flash-hz` flag; expose a "solid" mode so gate buoys can be marked solid for testing |
| `yolo_comparison_test/path2_switch_proposal/scripts/01_autolabel.py` | Add `black` class; add `is_off_buoy()` branch to `label_image()` |
| `simulation/accuracy_verify.py` | Add `light_mode` column to accuracy report; verify against ground-truth mode per buoy |
| `docs/00_index.md` | Add this file to the index table |
