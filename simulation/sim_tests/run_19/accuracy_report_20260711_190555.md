# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-11 19:06:07
- **Flight duration:** 186.0 s (OK; minimum 15 s)
- **World (ground truth):** `simulation/gazebo/worlds/course_2_search_field.sdf`
- **Detection log:** `simulation/sim_tests/run_19/detections_20260711_190555.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **13**
- Colour buoys detected: **0 / 6**
- Mean horizontal error: **nan m**
- Max horizontal error: **nan m**
- Mean detection confidence: **nan**
- Unmatched detections (no buoy within 3.0 m): **13**

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
