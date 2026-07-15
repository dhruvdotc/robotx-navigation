# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-11 22:48:14
- **Flight duration:** 274.7 s (OK; minimum 15 s)
- **Detector:** yolo
- **World (ground truth):** `simulation/gazebo/worlds/course_2_search_field.sdf`
- **Detection log:** `simulation/sim_tests/run_28/detections_20260711_224811.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **961**
- Colour buoys detected: **6 / 6**
- Mean horizontal error: **0.58 m**
- Max horizontal error: **1.29 m**
- Mean detection confidence: **0.95**
- Unmatched detections (no buoy within 3.0 m): **561**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| green1 | green | 10.00, 8.00 | 73 | 0.71 | 1.29 | 0.95 |
| green2 | green | -8.00, 24.00 | 61 | 0.54 | 1.26 | 0.95 |
| green3 | green | 7.00, 42.00 | 66 | 0.57 | 1.25 | 0.96 |
| red1 | red | -11.00, 14.00 | 58 | 0.50 | 1.08 | 0.95 |
| red2 | red | 6.00, 31.00 | 67 | 0.54 | 1.23 | 0.96 |
| red3 | red | -5.00, 48.00 | 75 | 0.59 | 1.27 | 0.94 |
| light_buoy | light | 2.00, 55.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
