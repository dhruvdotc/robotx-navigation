# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-11 20:45:31
- **Flight duration:** 272.9 s (OK; minimum 15 s)
- **Detector:** yolo
- **World (ground truth):** `simulation/gazebo/worlds/course_2_search_field.sdf`
- **Detection log:** `simulation/sim_tests/run_24/detections_20260711_204528.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **481**
- Colour buoys detected: **3 / 6**
- Mean horizontal error: **0.58 m**
- Max horizontal error: **1.52 m**
- Mean detection confidence: **0.87**
- Unmatched detections (no buoy within 3.0 m): **258**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| green1 | green | 10.00, 8.00 | 94 | 0.66 | 1.52 | 0.84 |
| green2 | green | -8.00, 24.00 | 78 | 0.54 | 1.29 | 0.88 |
| green3 | green | 7.00, 42.00 | 51 | 0.49 | 0.83 | 0.91 |
| red1 | red | -11.00, 14.00 | 0 | - | - | - |
| red2 | red | 6.00, 31.00 | 0 | - | - | - |
| red3 | red | -5.00, 48.00 | 0 | - | - | - |
| light_buoy | light | 2.00, 55.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
