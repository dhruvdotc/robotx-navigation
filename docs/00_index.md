# RobotX Navigation — Docs Index

> **Goal:** Full end-to-end pipeline for UAV buoy detection at RobotX — capture → annotate → train → deploy → test — reproducible from scratch on competition day.

---

## Files in this folder

| File | What it covers |
|------|---------------|
| [01_environment_setup.md](01_environment_setup.md) | Mac + Ubuntu/WSL + Jetson setup from scratch |
| [02_data_pipeline.md](02_data_pipeline.md) | Capture → augment → batch-detect → metrics |
| [03_detection_algorithm.md](03_detection_algorithm.md) | Two-stage CV pipeline (HSV): how it works |
| [04_gps_projection.md](04_gps_projection.md) | Pixel → NED → lat/lon math and calibration |
| [05_simulation.md](05_simulation.md) | Gazebo SITL — running all 3 courses |
| [06_real_flight.md](06_real_flight.md) | Full demo: Jetson detection + Mac ground station |
| [07_roadmap.md](07_roadmap.md) | Progress tracking + outstanding TODOs |

---

## Quick-start cheat sheet

### Simulation (Ubuntu/WSL)
```bash
bash simulation/run_course.sh --course 1          # headless
bash simulation/run_course.sh --course 1 --visual # 4-window GUI
```

### Real flight (field)
```bash
# Mac — terminal 1
bash fulldemo/run_gcs_mac.sh

# Jetson — terminal 2
GCS_IP=<MAC_IP> bash fulldemo/run_detection_jetson.sh
```

### Data pipeline (after collecting images)
```bash
python camera_capture_spacebar.py                         # capture
python augment_test.py captures/my_image.jpg              # augmentation smoke test
python hsv_batch_detect.py                                # batch detect + annotate
python metrics_summary.py                                 # print metrics + chart
python visualize_results.py                               # results diagram PNG
```

---

## Repo layout (top-level)

| Path | Purpose |
|------|---------|
| `camera_live_feed.py` | Main detector — HSV pipeline, Kalman tracking, GPS projection, MAVLink, ROS input |
| `camera_capture_spacebar.py` | Capture training images by spacebar |
| `hsv_batch_detect.py` | Batch HSV detector over `captures/` folder |
| `augment_test.py` | UAV noise augmentation test (blur + motion blur + glare) |
| `metrics_summary.py` | Parse detections CSV → per-class stats + bar chart |
| `visualize_results.py` | Generate `results_diagram.png` |
| `color_utils.py` | Shared HSV range helpers and `build_mask()` |
| `calibration/` | Camera intrinsics JSON |
| `fulldemo/` | One-command Mac + Jetson demo scripts |
| `mavlink_comms/` | MAVLink buoy report protocol + UDP ground station |
| `simulation/` | Gazebo Harmonic SITL stack (3 courses) |
| `scripts/` | Jetson WiFi + setup helpers |
