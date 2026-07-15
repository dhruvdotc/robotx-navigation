# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-07-10 21:53:31
- **Flight duration:** 60.6 s (OK; minimum 15 s)
- **World (ground truth):** `simulation/gazebo/worlds/robotx_uav_course.sdf`
- **Detection log:** `simulation/sim_tests/run_6/detections_20260710_215323.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **222**
- Colour buoys detected: **6 / 6**
- Mean horizontal error: **0.15 m**
- Max horizontal error: **0.67 m**
- Mean detection confidence: **0.93**
- Unmatched detections (no buoy within 3.0 m): **1**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 42 | 0.15 | 0.63 | 0.97 |
| gate1_red | red | -1.25, 10.00 | 42 | 0.15 | 0.63 | 0.88 |
| gate2_green | green | 1.25, 25.00 | 38 | 0.15 | 0.56 | 0.97 |
| gate2_red | red | -1.25, 25.00 | 37 | 0.14 | 0.57 | 0.88 |
| gate3_green | green | 1.25, 40.00 | 42 | 0.15 | 0.67 | 0.98 |
| gate3_red | red | -1.25, 40.00 | 20 | 0.17 | 0.67 | 0.87 |
| light_buoy | light | 0.00, 50.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
