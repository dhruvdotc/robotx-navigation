# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-14 17:25:38
- **Flight duration:** 87.9 s (OK; minimum 15 s)
- **Detector:** hsv
- **World (ground truth):** `simulation/gazebo/worlds/course_3_dogleg.sdf`
- **Detection log:** `simulation/sim_tests/run_32/detections_20260714_172524.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **660**
- Colour buoys detected: **7 / 8**
- Mean horizontal error: **0.20 m**
- Max horizontal error: **0.89 m**
- Mean detection confidence: **0.88**
- Unmatched detections (no buoy within 3.0 m): **284**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 95 | 0.20 | 0.56 | 0.87 |
| gate1_red | red | -1.25, 10.00 | 47 | 0.14 | 0.36 | 0.73 |
| gate2_green | green | 1.25, 25.00 | 82 | 0.25 | 0.61 | 0.89 |
| gate2_red | red | -1.25, 25.00 | 41 | 0.19 | 0.60 | 0.84 |
| gate3_green | green | 15.00, 36.25 | 49 | 0.19 | 0.84 | 0.97 |
| gate3_red | red | 15.00, 33.75 | 0 | - | - | - |
| gate4_green | green | 30.00, 36.25 | 54 | 0.19 | 0.89 | 0.97 |
| gate4_red | red | 30.00, 33.75 | 8 | 0.14 | 0.28 | 0.78 |
| light_buoy | light | 42.00, 35.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
