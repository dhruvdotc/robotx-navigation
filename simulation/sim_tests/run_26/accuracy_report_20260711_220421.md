# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-11 22:04:26
- **Flight duration:** 61.6 s (OK; minimum 15 s)
- **Detector:** yolo
- **World (ground truth):** `simulation/gazebo/worlds/robotx_uav_course.sdf`
- **Detection log:** `simulation/sim_tests/run_26/detections_20260711_220421.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **656**
- Colour buoys detected: **6 / 6**
- Mean horizontal error: **1.58 m**
- Max horizontal error: **2.98 m**
- Mean detection confidence: **0.83**
- Unmatched detections (no buoy within 3.0 m): **109**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 100 | 0.38 | 1.16 | 0.95 |
| gate1_red | red | -1.25, 10.00 | 115 | 2.70 | 2.98 | 0.68 |
| gate2_green | green | 1.25, 25.00 | 90 | 0.37 | 1.04 | 0.94 |
| gate2_red | red | -1.25, 25.00 | 92 | 2.70 | 2.87 | 0.74 |
| gate3_green | green | 1.25, 40.00 | 70 | 0.26 | 0.90 | 0.95 |
| gate3_red | red | -1.25, 40.00 | 80 | 2.68 | 2.90 | 0.75 |
| light_buoy | light | 0.00, 50.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
