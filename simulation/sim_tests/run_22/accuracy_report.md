# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-11 20:35:21
- **Flight duration:** 62.4 s (OK; minimum 15 s)
- **Detector:** yolo
- **World (ground truth):** `simulation/gazebo/worlds/robotx_uav_course.sdf`
- **Detection log:** `simulation/sim_tests/run_22/detections_20260711_203515.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **237**
- Colour buoys detected: **6 / 6**
- Mean horizontal error: **1.40 m**
- Max horizontal error: **2.69 m**
- Mean detection confidence: **0.89**
- Unmatched detections (no buoy within 3.0 m): **97**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 22 | 0.17 | 0.27 | 0.95 |
| gate1_red | red | -1.25, 10.00 | 22 | 2.65 | 2.69 | 0.80 |
| gate2_green | green | 1.25, 25.00 | 22 | 0.14 | 0.21 | 0.96 |
| gate2_red | red | -1.25, 25.00 | 22 | 2.65 | 2.67 | 0.82 |
| gate3_green | green | 1.25, 40.00 | 26 | 0.15 | 0.21 | 0.95 |
| gate3_red | red | -1.25, 40.00 | 26 | 2.65 | 2.68 | 0.83 |
| light_buoy | light | 0.00, 50.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
