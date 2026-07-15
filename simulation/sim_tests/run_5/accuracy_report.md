# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-10 20:23:24
- **Flight duration:** 88.9 s (OK; minimum 15 s)
- **World (ground truth):** `simulation/gazebo/worlds/course_3_dogleg.sdf`
- **Detection log:** `simulation/sim_tests/run_5/detections_20260710_202317.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **223**
- Colour buoys detected: **6 / 8**
- Mean horizontal error: **0.15 m**
- Max horizontal error: **0.68 m**
- Mean detection confidence: **0.95**
- Unmatched detections (no buoy within 3.0 m): **1**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 42 | 0.16 | 0.68 | 0.97 |
| gate1_red | red | -1.25, 10.00 | 42 | 0.15 | 0.67 | 0.88 |
| gate2_green | green | 1.25, 25.00 | 31 | 0.15 | 0.61 | 0.97 |
| gate2_red | red | -1.25, 25.00 | 31 | 0.14 | 0.60 | 0.88 |
| gate3_green | green | 15.00, 36.25 | 38 | 0.14 | 0.25 | 0.99 |
| gate3_red | red | 15.00, 33.75 | 0 | - | - | - |
| gate4_green | green | 30.00, 36.25 | 38 | 0.13 | 0.32 | 1.00 |
| gate4_red | red | 30.00, 33.75 | 0 | - | - | - |
| light_buoy | light | 42.00, 35.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
