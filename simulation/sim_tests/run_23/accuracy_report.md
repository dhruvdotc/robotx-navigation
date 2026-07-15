# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-11 20:37:11
- **Flight duration:** 276.7 s (OK; minimum 15 s)
- **Detector:** yolo
- **World (ground truth):** `simulation/gazebo/worlds/course_2_search_field.sdf`
- **Detection log:** `simulation/sim_tests/run_23/detections_20260711_203706.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **23**
- Colour buoys detected: **0 / 6**
- Mean horizontal error: **nan m**
- Max horizontal error: **nan m**
- Mean detection confidence: **nan**
- Unmatched detections (no buoy within 3.0 m): **23**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| green1 | green | 10.00, 8.00 | 0 | - | - | - |
| green2 | green | -8.00, 24.00 | 0 | - | - | - |
| green3 | green | 7.00, 42.00 | 0 | - | - | - |
| red1 | red | -11.00, 14.00 | 0 | - | - | - |
| red2 | red | 6.00, 31.00 | 0 | - | - | - |
| red3 | red | -5.00, 48.00 | 0 | - | - | - |
| light_buoy | light | 2.00, 55.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
