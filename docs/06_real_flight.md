# Real Flight — Full Demo

End-to-end guide for field deployment: Jetson Orin Nano runs YOLO + HSV detection, Mac receives GPS + color over MAVLink UDP.

---

## Architecture

```
[Camera] → [Jetson: YOLO → HSV → GPS projection] → MAVLink UDP → [Mac GCS: receive + log]
```

1. YOLO proposes bounding boxes at 960×540 detection resolution
2. HSV thresholding classifies each ROI as `red`, `green`, or `blue`
3. Pixel centroid projected to GPS lat/lon (flat-earth nadir model, 10 m AGL)
4. Each confirmed detection transmitted as MAVLink `STATUSTEXT` over UDP to the ground station

---

## Prerequisites

- Jetson: `buoy_best.pt` at `~/robotx-navigation/buoy_best.pt`
- Jetson and Mac on same network (see Step 0 below)
- `bash jetson_setup.sh` has been run on the Jetson at least once

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

Listens on UDP port `14555`. You should see:
```
Listening on udpin:0.0.0.0:14555 for RXB| buoy reports (Ctrl+C to stop)
```

Each received detection prints as JSON:
```json
{"target_id": 1, "color": "red", "lat": 32.88012, "lon": -117.23418, "frame": 42, "timestamp_ms": ...}
```

To save detections to a file:
```bash
python mavlink_comms/scripts/run_ground_station.py --output-jsonl fulldemo/detections.jsonl
```

Leave the GCS running and move to the Jetson.

---

## Step 2 — Start detection (Jetson)

```bash
ssh babydragon@<JETSON_IP>
cd ~/robotx-navigation

# USB-C option
GCS_IP=192.168.55.100 bash fulldemo/run_detection_jetson.sh

# Router option
GCS_IP=<MAC_ROUTER_IP> bash fulldemo/run_detection_jetson.sh
```

**Jetson console output to expect:**
```
YOLO loaded: buoy_best.pt
MAVLink transmitter → udpout:<GCS_IP>:14555
[TX] t1 red lat=32.88012 lon=-117.23418
```

**Lower thresholds if nothing is being detected:**
```bash
GCS_IP=<MAC_IP> bash fulldemo/run_detection_jetson.sh \
    --yolo-conf 0.15 --min-color-ratio 0.08
```

---

## Step 3 — Diagnose the link

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| No `[TX]` on Jetson | Nothing passes YOLO + HSV thresholds | Point camera at colored balloon; lower `--yolo-conf` |
| `[TX]` on Jetson but no `[GCS]` on Mac | UDP route broken | Check `GCS_IP`; verify interface with `ifconfig en10` or `ping` |
| Detections wrong color | HSV ranges off for current lighting | Recalibrate with `captures/classes/` crops; use `--calib-color` hotkey |
| Very high false positive rate | Low YOLO confidence threshold | Raise `--yolo-conf` to 0.25–0.35 |

---

## Step 4 — Collect data during flight

The Jetson writes two things automatically:
- `~/robotx-navigation/detection_logs/recording_<unix_ts>.avi` — raw video
- `~/robotx-navigation/detection_logs/detections.csv` — per-frame detection log

After landing, pull both to the Mac:

```bash
rsync -av babydragon@<JETSON_IP>:~/robotx-navigation/detection_logs/ \
  ~/Downloads/ROBOTX/robotx-navigation/fulldemo/session_data/
```

To extract a video frame at a specific detection timestamp:
```bash
# video_offset = detection_timestamp - recording_start_timestamp
ffmpeg -ss <offset_seconds> -i recording_<ts>.avi -frames:v 1 frame.jpg
```

---

## Step 5 — Visualize received coordinates

```bash
# After flight — from saved JSONL
python fulldemo/visualize_detections.py fulldemo/detections.jsonl

# Live during flight (polls the file every second)
python fulldemo/visualize_detections.py fulldemo/detections.jsonl --live
```

Opens an interactive dot map — each buoy detection is a colored dot at its estimated GPS position.

---

## Live calibration (field tuning)

While `camera_live_feed.py` is running, press `c` to recalibrate the S/V threshold floor for the currently selected `--calib-color`. This samples the center patch of the current frame and adjusts thresholds to match.

```bash
# Run detector with calibration mode for green
python camera_live_feed.py --camera-index 0 --altitude-m 10 --calib-color green
# Then aim at a green buoy and press 'c'
```

---

## Hotkeys (during live feed)

| Key | Action |
|-----|--------|
| `q` | Quit |
| `c` | Recalibrate S/V floor for `--calib-color` |
