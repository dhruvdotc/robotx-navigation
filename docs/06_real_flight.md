# Real Flight — Full Demo

End-to-end guide for field deployment: Jetson Orin Nano runs HSV detection, logs GPS + color to CSV. MAVLink TX to Mac GCS is wired once TODO #2 (YOLO integration) is complete.

---

## Architecture

```
[Camera] → [Jetson: HSV detection → GPS projection → MAVLink TX] → UDP → [Mac GCS: receive + log]
```

1. HSV two-stage pipeline detects buoys and classifies as `red`, `green`, or `blue`
2. Pixel centroid projected to GPS lat/lon (flat-earth nadir model)
3. Each confirmed detection transmitted as MAVLink `STATUSTEXT` over UDP to the ground station

> **Note — YOLO integration is pending (TODO #2 in roadmap):** `run_detection_jetson.sh` is written for a future version of `camera_live_feed.py` that adds `--yolo-model`, `--gcs-ip`, `--save-video`, `--drone-lat`, `--heading-deg`. These flags **do not exist yet** in the current file. Until the YOLO integration is complete, run `camera_live_feed.py` directly (see Step 2 below). The shell script will fail with unknown-argument errors on the current codebase.

---

## Prerequisites

- Jetson and Mac on same network (see Step 0 below)
- `bash jetson_setup.sh` has been run on the Jetson at least once (creates `.venv-mavlink`)
- `captures/classes/` exists at `~/robotx-navigation/captures/classes/` with reference crops (`red.jpg`, `green.jpg`, `blue.jpg`) — used to derive HSV ranges at startup

---

## Step 0 — Network setup

### Option A: USB-C (benchtop / fallback)

```bash
# On Mac — assign USB ethernet interface
sudo ifconfig en10 192.168.55.100 netmask 255.255.255.0
ping 192.168.55.1   # should reach Jetson
```

Jetson IP: `192.168.55.1`, Mac IP: `192.168.55.100`

### Option B: WiFi router (recommended for field)

Connect both Jetson and Mac to the field router SSID (e.g. `GL-AXT1800-*`).

```bash
# Mac — find your LAN IP
ipconfig getifaddr en0

# Jetson WiFi setup (run once)
bash scripts/jetson_wifi_setup.sh
```

Note both IPs from `ifconfig` / `ipconfig`. Use the **Mac's router IP** as `GCS_IP`.

---

## Step 1 — Start ground station (Mac)

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

## Step 2 — Start detection (Jetson)

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

> **When YOLO integration lands (TODO #2):** replace the above with `GCS_IP=<MAC_IP> bash fulldemo/run_detection_jetson.sh`. Until then, run `camera_live_feed.py` directly.

**Tune thresholds by editing the flags above, e.g.:**
- Too many false positives → add `--min-color-ratio 0.18` or `--min-circularity 0.4`
- Missing buoys → lower `--min-color-ratio 0.08` or `--min-circularity 0.2`

---

## Step 3 — Diagnose

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| No `[GPS]` lines on Jetson | Nothing passes HSV thresholds | Point camera at colored balloon; lower `--min-color-ratio` |
| `[GPS]` on Jetson but no `[GCS]` on Mac | MAVLink TX not wired yet (YOLO TODO) | Currently HSV-only; GCS link requires TODO #2 |
| Detections wrong color | HSV ranges off for current lighting | Recalibrate with `captures/classes/` crops; use `--calib-color` hotkey |
| Very high false positive rate | Min color ratio too low | Raise `--min-color-ratio 0.18` or `--min-circularity 0.4` |

---

## Step 4 — Collect data during flight

`camera_live_feed.py` writes to `--log-dir` (set to `~/detection_logs/` in Step 2):
- `~/detection_logs/detections.csv` — per-detection log with columns: `timestamp, image_path, track_id, color, confidence, cx, cy, x, y, w, h, north_m, east_m, lat, lon, altitude_m, intrinsics_source`
- `~/detection_logs/frame_<timestamp_ms>.jpg` — saved JPEG for each detection event

After landing, pull to the Mac:

```bash
rsync -av babydragon@<JETSON_IP>:~/detection_logs/ \
  ~/Downloads/ROBOTX/robotx-navigation/fulldemo/session_data/
```

> **Note:** there is no `--save-video` flag in the current version. Full video recording requires TODO #2 (YOLO integration).

---

## Step 5 — Visualize received coordinates

> **Current state (HSV-only, no MAVLink TX):** `fulldemo/detections.jsonl` is only populated if the GCS receives MAVLink reports, which requires TODO #2. With the current pipeline, use the CSV from Step 4:
>
> ```bash
> # Quick sanity check on detections from this session
> column -t -s, fulldemo/session_data/detections.csv | head -20
> ```

Once TODO #2 is complete and the MAVLink link is wired, the dot-map visualizer works:

```bash
# After flight — from saved JSONL
python fulldemo/visualize_detections.py fulldemo/detections.jsonl

# Live during flight (polls the file every second)
python fulldemo/visualize_detections.py fulldemo/detections.jsonl --live
```

Opens an interactive dot map — each buoy detection is a colored dot at its estimated GPS position.

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
