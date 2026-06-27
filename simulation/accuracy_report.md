# RobotX UAV Course - Accuracy Verification Report

- **Flight / recording timestamp:** 2026-06-26 22:58:19
- **Flight duration:** 81.9 s (OK; minimum 15 s)
- **World (ground truth):** `simulation/gazebo/worlds/robotx_uav_course.sdf`
- **Detection log:** `simulation/accuracy_logs/detections_20260626_225643.csv`
- **Datum:** lat -35.363262, lon 149.165237

## Summary

- Total logged detections: **512**
- Colour buoys detected: **6 / 6**
- Mean horizontal error: **0.16 m**
- Max horizontal error: **1.04 m**
- Mean detection confidence: **0.87**
- Unmatched detections (no buoy within 3.0 m): **0**

## Per-buoy accuracy

| Buoy | Colour | True N,E (m) | Detections | Mean err (m) | Max err (m) | Mean conf |
|------|--------|--------------|-----------:|-------------:|------------:|----------:|
| gate1_green | green | 1.25, 10.00 | 91 | 0.16 | 0.95 | 0.86 |
| gate1_red | red | -1.25, 10.00 | 87 | 0.16 | 0.95 | 0.88 |
| gate2_green | green | 1.25, 25.00 | 84 | 0.16 | 1.04 | 0.86 |
| gate2_red | red | -1.25, 25.00 | 82 | 0.17 | 1.03 | 0.87 |
| gate3_green | green | 1.25, 40.00 | 84 | 0.16 | 1.01 | 0.86 |
| gate3_red | red | -1.25, 40.00 | 84 | 0.16 | 1.00 | 0.88 |
| light_buoy | light | 0.00, 50.00 | 0 | - | - | - (nadir: black box, no colour - expected miss) |

_Each detection is matched to the nearest same-colour ground-truth buoy within the match radius; error is the horizontal distance between the projected absolute position and the buoy's true position._
