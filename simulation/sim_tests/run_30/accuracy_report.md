# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-14 17:17:48
- **Flight duration:** 61.2 s (OK; minimum 15 s)
- **Detector:** hsv
- **World (ground truth):** `simulation/gazebo/worlds/robotx_uav_course.sdf`
- **Detection log:** `simulation/sim_tests/run_30/detections_20260714_171733.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **539**
- Colour buoys detected: **6 / 6**
- Mean horizontal error: **0.20 m**
- Max horizontal error: **1.07 m**
- Mean detection confidence: **0.87**
- Unmatched detections (no buoy within 3.0 m): **96**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 93 | 0.21 | 0.74 | 0.89 |
| gate1_red | red | -1.25, 10.00 | 51 | 0.16 | 0.74 | 0.75 |
| gate2_green | green | 1.25, 25.00 | 101 | 0.22 | 1.07 | 0.89 |
| gate2_red | red | -1.25, 25.00 | 55 | 0.17 | 1.05 | 0.81 |
| gate3_green | green | 1.25, 40.00 | 103 | 0.20 | 0.73 | 0.92 |
| gate3_red | red | -1.25, 40.00 | 40 | 0.16 | 0.44 | 0.84 |
| light_buoy | light | 0.00, 50.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
