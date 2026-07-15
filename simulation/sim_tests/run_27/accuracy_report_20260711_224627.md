# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-11 22:46:30
- **Flight duration:** 49.0 s (OK; minimum 15 s)
- **Detector:** yolo
- **World (ground truth):** `simulation/gazebo/worlds/robotx_uav_course.sdf`
- **Detection log:** `simulation/sim_tests/run_27/detections_20260711_224627.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **550**
- Colour buoys detected: **6 / 6**
- Mean horizontal error: **0.36 m**
- Max horizontal error: **2.43 m**
- Mean detection confidence: **0.96**
- Unmatched detections (no buoy within 3.0 m): **140**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 55 | 0.30 | 2.43 | 0.97 |
| gate1_red | red | -1.25, 10.00 | 55 | 0.31 | 2.43 | 0.96 |
| gate2_green | green | 1.25, 25.00 | 80 | 0.36 | 1.18 | 0.97 |
| gate2_red | red | -1.25, 25.00 | 80 | 0.37 | 1.17 | 0.95 |
| gate3_green | green | 1.25, 40.00 | 70 | 0.39 | 1.27 | 0.97 |
| gate3_red | red | -1.25, 40.00 | 70 | 0.40 | 1.28 | 0.96 |
| light_buoy | light | 0.00, 50.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
