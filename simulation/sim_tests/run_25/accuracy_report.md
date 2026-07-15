# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-11 20:51:35
- **Flight duration:** 88.0 s (OK; minimum 15 s)
- **Detector:** yolo
- **World (ground truth):** `simulation/gazebo/worlds/course_3_dogleg.sdf`
- **Detection log:** `simulation/sim_tests/run_25/detections_20260711_205129.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **503**
- Colour buoys detected: **8 / 8**
- Mean horizontal error: **1.44 m**
- Max horizontal error: **2.99 m**
- Mean detection confidence: **0.83**
- Unmatched detections (no buoy within 3.0 m): **63**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 74 | 0.34 | 1.18 | 0.93 |
| gate1_red | red | -1.25, 10.00 | 72 | 2.70 | 2.99 | 0.79 |
| gate2_green | green | 1.25, 25.00 | 76 | 0.36 | 1.32 | 0.92 |
| gate2_red | red | -1.25, 25.00 | 70 | 2.70 | 2.97 | 0.77 |
| gate3_green | green | 15.00, 36.25 | 36 | 0.18 | 0.52 | 0.92 |
| gate3_red | red | 15.00, 33.75 | 35 | 2.60 | 2.68 | 0.59 |
| gate4_green | green | 30.00, 36.25 | 40 | 0.17 | 0.50 | 0.92 |
| gate4_red | red | 30.00, 33.75 | 37 | 2.58 | 2.66 | 0.68 |
| light_buoy | light | 42.00, 35.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
