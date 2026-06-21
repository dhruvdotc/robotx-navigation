# robotx-navigation

Clean team repo for **live buoy detection** (Jetson) → **MAVLink UDP** → **Mac ground station**.

Forked from [saxysteph/145-237D-robotx-navigation](https://github.com/saxysteph/145-237D-robotx-navigation) (UCSD CSE 237D / RobotX 2026).

## Quick start

**Mac (Terminal 1):**
```bash
bash fulldemo/run_gcs_mac.sh
```

**Jetson (Terminal 2):**
```bash
ssh babydragon@<JETSON_IP>
cd ~/robotx-navigation
GCS_IP=<MAC_IP> bash fulldemo/run_detection_jetson.sh
```

See `fulldemo/README.md` and `fulldemo/PARTNER_INSTRUCTIONS.md` for WiFi router setup, tuning, and troubleshooting.

## Layout

| Path | Purpose |
|------|---------|
| `camera_live_feed.py` | YOLO + HSV + GPS + MAVLink pipeline |
| `fulldemo/` | One-command Mac/Jetson demo scripts |
| `mavlink_comms/` | UDP buoy protocol + ground station |
| `scripts/` | Jetson WiFi helpers |
| `yolo_comparison_test/.../buoy_balloon_roboflow_best.pt` | Roboflow-trained detector |
| `jetson_setup.sh` | Jetson dependency bootstrap |

## Weights

Runtime prefers (in order): `buoy_best.onnx`, Roboflow `.onnx`, Roboflow `.pt` — see `fulldemo/run_detection_jetson.sh`.
