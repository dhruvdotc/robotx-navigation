# Real Flight - Full Demo

End-to-end guide for field deployment: Jetson Orin Nano runs detection (HSV, or YOLO via `--yolo-model`), logs GPS + color to CSV, and can transmit to a Mac GCS over MAVLink.

---

## Architecture

```
[Camera] → [Jetson: HSV or YOLO detection → GPS projection → MAVLink TX] → UDP → [Mac GCS: receive + log]
```

1. Detector (HSV two-stage, or a fine-tuned YOLO model via `--yolo-model`) finds buoys and classifies as `red`, `green`, or `blue`
2. Pixel centroid projected to GPS lat/lon (flat-earth nadir model; `--heading-deg` corrects for real drone yaw, unlike the sim which always flies yaw-locked)
3. If `--gcs-ip` is set, each confirmed detection is transmitted as MAVLink `STATUSTEXT` over UDP to the ground station

> **Status - TODO #2 (YOLO integration) flags now exist:** `--yolo-model`, `--yolo-conf`, `--gcs-ip`, `--save-video`, `--drone-lat`/`--drone-lon`, `--heading-deg`, `--headless` are all implemented in `camera_live_feed.py`, so `run_detection_jetson.sh` should run as written. **This hasn't been verified with a live camera/model yet** - it was implemented and code-reviewed in a sandbox with no network access to install `ultralytics` for an end-to-end smoke test. Run it once for real (Step 2 below has both the new one-liner and the direct-invocation fallback) before trusting it in the field.

---

## Prerequisites

- Jetson and Mac on same network (see Step 0 below)
- `bash jetson_setup.sh` has been run on the Jetson at least once (creates `.venv-mavlink`)
- `captures/classes/` exists at `~/robotx-navigation/captures/classes/` with reference crops (`red.jpg`, `green.jpg`, `blue.jpg`) - used to derive HSV ranges at startup

---

## Step 0 - Network setup

### Option A: USB-C (benchtop / fallback)

```bash
# On Mac - assign USB ethernet interface
sudo ifconfig en10 192.168.55.100 netmask 255.255.255.0
ping 192.168.55.1   # should reach Jetson
```

Jetson IP: `192.168.55.1`, Mac IP: `192.168.55.100`

### Option B: WiFi router (recommended for field)

Connect both Jetson and Mac to the field router SSID (e.g. `GL-AXT1800-*`).

```bash
# Mac - find your LAN IP
ipconfig getifaddr en0

# Jetson WiFi setup (run once)
bash scripts/jetson_wifi_setup.sh
```

Note both IPs from `ifconfig` / `ipconfig`. Use the **Mac's router IP** as `GCS_IP`.

---

## Step 1 - Start ground station (Mac)

```bash
cd ~/Downloads/ROBOTX/robotx-navigation
bash fulldemo/run_gcs_mac.sh
```

The script creates `.venv-mavlink` on first run (takes ~30s), then starts listening. You should see:
```
Mac IP on WiFi: 192.168.8.xxx
Jetson should use: --gcs-ip 192.168.8.xxx
Listening UDP 14555 for buoy reports (Ctrl+C to stop)
Listening on udpin:0.0.0.0:14555 for RXB| buoy reports (Ctrl+C to stop)
```

Detections are **automatically saved** to `fulldemo/detections.jsonl`. Each received report also prints to stdout as JSON:
```json
{"target_id": 1, "color": "red", "lat": 32.88012, "lon": -117.23418, "frame": 42, "timestamp_ms": ...}
```

Leave the GCS running and move to the Jetson.

---

## Step 2 - Start detection (Jetson)

```bash
ssh babydragon@<JETSON_IP>
cd ~/robotx-navigation
source .venv-mavlink/bin/activate

python3 camera_live_feed.py \
  --no-display \
  --camera-index 0 \
  --altitude-m 10 \
  --origin-lat 32.88010 \
  --origin-lon -117.23420 \
  --log-dir ~/detection_logs
```

**Jetson console output to expect:**
```
Using camera index: 0
[INFO] Active intrinsics [calibration-file]: fx=1319.07 fy=1407.50 cx=870.93 cy=533.10 dist=[...]
[GPS] cam t1 red conf=0.87 px=(960,540) NED=N+1.23m E-0.45m -> lat=32.8801234 lon=-117.2341234
```

> **YOLO integration (TODO #2) is now implemented:** `GCS_IP=<MAC_IP> bash fulldemo/run_detection_jetson.sh` should work as a one-liner (it finds a `.onnx`/`.pt` model automatically and passes `--yolo-model`, `--gcs-ip`, `--save-video`, etc.). The direct `camera_live_feed.py` invocation above still works too (HSV-only, no `--yolo-model`/`--gcs-ip`) and is the safer fallback until the one-liner has been run once for real - it was implemented without a live camera/model available to test against.

**Tune thresholds by editing the flags above, e.g.:**
- Too many false positives → add `--min-color-ratio 0.18` or `--min-circularity 0.4`
- Missing buoys → lower `--min-color-ratio 0.08` or `--min-circularity 0.2`

---

## Step 3 - Diagnose

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| No `[GPS]` lines on Jetson | Nothing passes HSV/YOLO threshold | Point camera at colored balloon/buoy; lower `--min-color-ratio` (HSV) or `--yolo-conf` (YOLO) |
| `[GPS]` on Jetson but no `[GCS]` on Mac | `--gcs-ip` not set, or `mavlink_comms`/vendored `mavcore` missing | Pass `--gcs-ip <MAC_IP>`; run `jetson_setup.sh` first to clone `vendor/mavcore` |
| Detections wrong color (HSV mode) | HSV ranges off for current lighting | Recalibrate with `captures/classes/` crops; use `--calib-color` hotkey |
| Very high false positive rate | Min color ratio too low | Raise `--min-color-ratio 0.18` or `--min-circularity 0.4` |

---

## Step 4 - Collect data during flight

`camera_live_feed.py` writes to `--log-dir` (set to `~/detection_logs/` in Step 2):
- `~/detection_logs/detections.csv` - per-detection log with columns: `timestamp, image_path, track_id, color, confidence, cx, cy, x, y, w, h, north_m, east_m, lat, lon, altitude_m, intrinsics_source`
- `~/detection_logs/frame_<timestamp_ms>.jpg` - saved JPEG for each detection event

After landing, pull to the Mac:

```bash
rsync -av babydragon@<JETSON_IP>:~/detection_logs/ \
  ~/Downloads/ROBOTX/robotx-navigation/fulldemo/session_data/
```

`--save-video` now writes annotated frames to an mp4 in `--log-dir` (works headless too) - add it to the Step 2 command if you want a recording.

---

## Step 5 - Visualize received coordinates

> **If you didn't pass `--gcs-ip`:** `fulldemo/detections.jsonl` stays empty since nothing was transmitted. Use the CSV from Step 4 instead:
>
> ```bash
> # Quick sanity check on detections from this session
> column -t -s, fulldemo/session_data/detections.csv | head -20
> ```

With `--gcs-ip` set and the GCS running (Step 1), the dot-map visualizer works:

```bash
# After flight - from saved JSONL
python fulldemo/visualize_detections.py fulldemo/detections.jsonl

# Live during flight (polls the file every second)
python fulldemo/visualize_detections.py fulldemo/detections.jsonl --live
```

Opens an interactive dot map - each buoy detection is a colored dot at its estimated GPS position.

---

## Live calibration (field tuning)

While `camera_live_feed.py` is running **with a display window** (i.e. without `--no-display`), press `c` to recalibrate the S/V threshold floor for the currently selected `--calib-color`. This samples the center patch of the current frame and adjusts thresholds to match.

```bash
# Run detector with display + calibration mode for green
python3 camera_live_feed.py --camera-index 0 --altitude-m 10 --calib-color green
# Aim center of frame at a green buoy, press 'c'
# Output: "Calibrated green: S_min=XX, V_min=XX"
```

---

## Hotkeys (during live feed)

| Key | Action |
|-----|--------|
| `q` | Quit |
| `c` | Recalibrate S/V floor for `--calib-color` |
