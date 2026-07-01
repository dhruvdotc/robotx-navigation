# Competition Day Cheat Sheet

Sequence from arriving on-site to flying. Read [06_real_flight.md](06_real_flight.md) for detailed explanations of any step.

---

## Phase 1 — Arrive & collect images (1–2 hours before flight)

Goal: capture enough images at actual venue lighting to retrain the model.

**On the Jetson (or any camera you're using):**
```bash
ssh babydragon@<JETSON_IP>
cd ~/robotx-navigation
python3 camera_capture_spacebar.py --camera-index 0 --output-dir captures --prefix comp
```

Point at each buoy color and press `Space`. Aim for **30–50 images per color** (red, green, blue). Also grab 10–20 shots with distractors visible.

**Sort reference crops:**
Pick **one** tight, well-lit crop of each buoy color and save with exact filenames:
```
yolo_comparison_test/path2_switch_proposal/captures/classes/
├── red.jpg    ← must be named exactly "red.jpg"
├── green.jpg
└── blue.jpg
```
The filename stem is how the code identifies the color — other names are ignored. Copy all raw captures (flat) into `yolo_comparison_test/path2_switch_proposal/captures/`.

---

## Phase 2 — Auto-label & train (Mac, ~20–30 min)

```bash
cd ~/Downloads/ROBOTX/robotx-navigation/yolo_comparison_test/path2_switch_proposal/scripts

# 1. Auto-label
python 01_autolabel.py
cat path2_dataset/autolabel_summary.txt    # verify box counts look right

# 2. Train (runs in background — ~10-20 min on Mac)
python 02_finetune.py

# 3. Validate (run all 5 steps)
python validation_step1_proper_split.py
python validation_step2_retrain.py
python validation_step3_val_inference.py
python validation_step4_overfit_check.py
python validation_step5_stress_test.py

# 4. Check results (you are already inside scripts/)
cat honest_results.txt    # mAP50 should be > 0.80
```

**If mAP50 < 0.80:** collect more images for the weakest color class, spot-fix labels in LabelImg, re-run from step 2.

---

## Phase 3 — Export ONNX and deploy to Jetson

`run_detection_jetson.sh` looks for `buoy_best.onnx`, not `.pt`. Export first:

```bash
# From repo root (with ultralytics installed)
python3 -c "
from ultralytics import YOLO
m = YOLO('yolo_comparison_test/path2_switch_proposal/scripts/training/balloon_proper/weights/best.pt')
m.export(format='onnx', imgsz=640)
"

# Copy to Jetson
scp yolo_comparison_test/path2_switch_proposal/scripts/training/balloon_proper/weights/best.onnx \
    babydragon@<JETSON_IP>:~/robotx-navigation/buoy_best.onnx
```

> **YOLO integration is TODO #2** — until done, the ONNX file won't be called by the run script automatically. Skip this phase for now and run `camera_live_feed.py` directly in Phase 5.

---

## Phase 4 — Network setup

Pick one:

**Option A — WiFi router (preferred):**
```bash
# Jetson
bash scripts/jetson_wifi_setup.sh    # connect to field router

# Mac — find your IP
ipconfig getifaddr en0
```

**Option B — USB-C (fallback):**
```bash
sudo ifconfig en10 192.168.55.100 netmask 255.255.255.0
ping 192.168.55.1
```

---

## Phase 5 — Start pipeline

**Mac (Terminal 1) — Ground station:**
```bash
cd ~/Downloads/ROBOTX/robotx-navigation
bash fulldemo/run_gcs_mac.sh
```
Wait for: `Listening UDP 14555 for buoy reports`
Detections auto-save to `fulldemo/detections.jsonl`.

**Jetson (Terminal 2) — Detector (HSV pipeline, current state):**
```bash
ssh babydragon@<JETSON_IP>
cd ~/robotx-navigation
source .venv-mavlink/bin/activate
python3 camera_live_feed.py \
  --no-display \
  --camera-index 0 \
  --altitude-m 10 \
  --origin-lat <FIELD_LAT> \
  --origin-lon <FIELD_LON> \
  --log-dir ~/detection_logs
```
Wait for: `Using camera index: 0` then `[INFO] Active intrinsics [...]`

> Once **TODO #2** (YOLO integration) is complete, replace the above with: `GCS_IP=<MAC_IP> bash fulldemo/run_detection_jetson.sh`

**Verify:** point camera at a buoy and confirm `[GPS]` lines appear in the Jetson terminal.

---

## Phase 6 — Tune if needed

| Problem | Fix |
|---------|-----|
| No detections | Lower `--min-color-ratio 0.08`, aim camera at buoy directly |
| Wrong color | Re-run with `--calib-color <color>`, aim center at buoy, press `c` |
| False positives everywhere | Raise `--min-color-ratio 0.18` or `--min-circularity 0.4` |
| GCS not receiving | YOLO/MAVLink TX is TODO #2 — GCS link not wired to current HSV pipeline |

**Quick HSV recalibration (needs display — omit `--no-display`):**
```bash
python3 camera_live_feed.py --camera-index 0 --altitude-m 10 --calib-color green
# Aim center of frame at green buoy, press 'c'
```

---

## Phase 7 — Save session data after flight

```bash
rsync -av babydragon@<JETSON_IP>:~/detection_logs/ \
  ~/Downloads/ROBOTX/robotx-navigation/fulldemo/session_data/
```

**Visualize results (requires TODO #2 — MAVLink TX):**
```bash
python fulldemo/visualize_detections.py fulldemo/detections.jsonl
```

---

## Altitude reminder

The GPS projection assumes **10 m AGL** by default. If you fly at a different altitude, pass `--altitude-m <value>` to `camera_live_feed.py` directly (see Phase 5). There is no altitude flag in `run_detection_jetson.sh` — that script hardcodes `altitude_m=10`.

---

## Pre-flight checklist

- [ ] `captures/classes/` on Jetson has today's reference crops (red.jpg, green.jpg, blue.jpg)
- [ ] GCS running and printing `Listening UDP 14555 for buoy reports` (Mac terminal)
- [ ] Jetson detector running — `[GPS]` lines appear when camera points at buoy
- [ ] `--altitude-m` matches your planned flight altitude (pass to `camera_live_feed.py`)
- [ ] `~/detection_logs/` cleared or new `--log-dir` set for this session
- [ ] Team knows which terminal to watch for `[GPS]` detections (Jetson terminal)
- [ ] **TODO #2 pending:** MAVLink TX not yet wired — GCS won't receive reports until YOLO integration is done
