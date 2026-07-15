# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-11 01:28:37
- **Flight duration:** 61.8 s (OK; minimum 15 s)
- **World (ground truth):** `simulation/gazebo/worlds/robotx_uav_course.sdf`
- **Detection log:** `simulation/sim_tests/run_10/detections_20260711_012828.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **271**
- Colour buoys detected: **6 / 6**
- Mean horizontal error: **0.14 m**
- Max horizontal error: **0.25 m**
- Mean detection confidence: **0.93**
- Unmatched detections (no buoy within 3.0 m): **82**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 34 | 0.16 | 0.25 | 0.97 |
| gate1_red | red | -1.25, 10.00 | 34 | 0.15 | 0.25 | 0.88 |
| gate2_green | green | 1.25, 25.00 | 30 | 0.14 | 0.22 | 0.98 |
| gate2_red | red | -1.25, 25.00 | 23 | 0.12 | 0.21 | 0.87 |
| gate3_green | green | 1.25, 40.00 | 34 | 0.15 | 0.21 | 0.98 |
| gate3_red | red | -1.25, 40.00 | 34 | 0.14 | 0.20 | 0.88 |
| light_buoy | light | 0.00, 50.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
