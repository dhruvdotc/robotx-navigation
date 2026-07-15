# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-14 17:19:28
- **Flight duration:** 276.4 s (OK; minimum 15 s)
- **Detector:** hsv
- **World (ground truth):** `simulation/gazebo/worlds/course_2_search_field.sdf`
- **Detection log:** `simulation/sim_tests/run_31/detections_20260714_171913.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **707**
- Colour buoys detected: **6 / 6**
- Mean horizontal error: **0.42 m**
- Max horizontal error: **1.13 m**
- Mean detection confidence: **0.80**
- Unmatched detections (no buoy within 3.0 m): **501**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| green1 | green | 10.00, 8.00 | 76 | 0.30 | 0.75 | 0.81 |
| green2 | green | -8.00, 24.00 | 60 | 0.45 | 1.13 | 0.83 |
| green3 | green | 7.00, 42.00 | 43 | 0.44 | 0.84 | 0.83 |
| red1 | red | -11.00, 14.00 | 2 | 0.43 | 0.57 | 0.62 |
| red2 | red | 6.00, 31.00 | 21 | 0.74 | 1.00 | 0.67 |
| red3 | red | -5.00, 48.00 | 4 | 0.47 | 0.64 | 0.60 |
| light_buoy | light | 2.00, 55.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
