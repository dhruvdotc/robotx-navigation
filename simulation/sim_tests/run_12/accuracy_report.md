# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-11 01:35:14
- **Flight duration:** 86.9 s (OK; minimum 15 s)
- **World (ground truth):** `simulation/gazebo/worlds/course_3_dogleg.sdf`
- **Detection log:** `simulation/sim_tests/run_12/detections_20260711_013507.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **265**
- Colour buoys detected: **6 / 8**
- Mean horizontal error: **0.14 m**
- Max horizontal error: **0.35 m**
- Mean detection confidence: **0.95**
- Unmatched detections (no buoy within 3.0 m): **76**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 34 | 0.16 | 0.28 | 0.97 |
| gate1_red | red | -1.25, 10.00 | 34 | 0.15 | 0.26 | 0.88 |
| gate2_green | green | 1.25, 25.00 | 34 | 0.13 | 0.17 | 0.98 |
| gate2_red | red | -1.25, 25.00 | 22 | 0.11 | 0.19 | 0.87 |
| gate3_green | green | 15.00, 36.25 | 30 | 0.17 | 0.35 | 0.99 |
| gate3_red | red | 15.00, 33.75 | 0 | - | - | - |
| gate4_green | green | 30.00, 36.25 | 35 | 0.13 | 0.30 | 1.00 |
| gate4_red | red | 30.00, 33.75 | 0 | - | - | - |
| light_buoy | light | 42.00, 35.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
