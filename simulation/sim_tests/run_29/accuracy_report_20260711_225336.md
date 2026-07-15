# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-11 22:53:39
- **Flight duration:** 87.1 s (OK; minimum 15 s)
- **Detector:** yolo
- **World (ground truth):** `simulation/gazebo/worlds/course_3_dogleg.sdf`
- **Detection log:** `simulation/sim_tests/run_29/detections_20260711_225336.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **803**
- Colour buoys detected: **8 / 8**
- Mean horizontal error: **0.33 m**
- Max horizontal error: **1.19 m**
- Mean detection confidence: **0.96**
- Unmatched detections (no buoy within 3.0 m): **292**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 104 | 0.38 | 1.18 | 0.97 |
| gate1_red | red | -1.25, 10.00 | 104 | 0.38 | 1.19 | 0.95 |
| gate2_green | green | 1.25, 25.00 | 83 | 0.38 | 1.09 | 0.97 |
| gate2_red | red | -1.25, 25.00 | 83 | 0.38 | 1.09 | 0.95 |
| gate3_green | green | 15.00, 36.25 | 37 | 0.22 | 0.58 | 0.94 |
| gate3_red | red | 15.00, 33.75 | 36 | 0.23 | 0.58 | 0.95 |
| gate4_green | green | 30.00, 36.25 | 32 | 0.16 | 0.54 | 0.96 |
| gate4_red | red | 30.00, 33.75 | 32 | 0.18 | 0.55 | 0.96 |
| light_buoy | light | 42.00, 35.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
