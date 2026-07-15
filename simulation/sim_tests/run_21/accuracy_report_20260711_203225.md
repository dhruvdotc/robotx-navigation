# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-11 20:32:27
- **Flight duration:** 50.7 s (OK; minimum 15 s)
- **Detector:** hsv
- **World (ground truth):** `simulation/gazebo/worlds/robotx_uav_course.sdf`
- **Detection log:** `simulation/sim_tests/run_21/detections_20260711_203225.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **124**
- Colour buoys detected: **6 / 6**
- Mean horizontal error: **1.39 m**
- Max horizontal error: **2.67 m**
- Mean detection confidence: **0.87**
- Unmatched detections (no buoy within 3.0 m): **32**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 1 | 0.25 | 0.25 | 0.95 |
| gate1_red | red | -1.25, 10.00 | 1 | 2.64 | 2.64 | 0.79 |
| gate2_green | green | 1.25, 25.00 | 22 | 0.15 | 0.23 | 0.96 |
| gate2_red | red | -1.25, 25.00 | 22 | 2.64 | 2.67 | 0.77 |
| gate3_green | green | 1.25, 40.00 | 23 | 0.14 | 0.21 | 0.95 |
| gate3_red | red | -1.25, 40.00 | 23 | 2.64 | 2.67 | 0.80 |
| light_buoy | light | 0.00, 50.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
