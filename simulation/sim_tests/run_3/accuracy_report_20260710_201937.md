# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-10 20:19:38
- **Flight duration:** 100.9 s (OK; minimum 15 s)
- **World (ground truth):** `simulation/gazebo/worlds/robotx_uav_course.sdf`
- **Detection log:** `simulation/sim_tests/run_3/detections_20260710_201937.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **0**
- Colour buoys detected: **0 / 6**
- Mean horizontal error: **nan m**
- Max horizontal error: **nan m**
- Mean detection confidence: **nan**
- Unmatched detections (no buoy within 3.0 m): **0**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 0 | - | - | - |
| gate1_red | red | -1.25, 10.00 | 0 | - | - | - |
| gate2_green | green | 1.25, 25.00 | 0 | - | - | - |
| gate2_red | red | -1.25, 25.00 | 0 | - | - | - |
| gate3_green | green | 1.25, 40.00 | 0 | - | - | - |
| gate3_red | red | -1.25, 40.00 | 0 | - | - | - |
| light_buoy | light | 0.00, 50.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
