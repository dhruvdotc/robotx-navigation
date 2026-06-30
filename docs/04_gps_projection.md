# GPS Projection

How a pixel detection in the camera frame becomes an absolute GPS coordinate.

**Code:** `camera_live_feed.py` — `project_pixel_to_ground_ned()` and `ned_to_gps()`

---

## Pipeline: pixel → NED → lat/lon

```
pixel (px, py) in full-res frame
         │
         │  Step 1: normalize by intrinsics
         │
         ▼
normalized image coords (x_n, y_n)
  x_n = (px - cx) / fx
  y_n = (py - cy) / fy
         │
         │  Step 2: scale by altitude (flat ground plane, nadir camera)
         │
         ▼
NED ground offset (north_m, east_m)
  east_m  =  x_n × altitude_m      (image +x = East)
  north_m = -y_n × altitude_m      (image +y = South → negate for North)
         │
         │  Step 3: add to origin GPS (datum)
         │
         ▼
absolute GPS (lat, lon)
  earth_r = 6_371_000 m
  lat = origin_lat + degrees(north_m / earth_r)
  lon = origin_lon + degrees(east_m / (earth_r × cos(origin_lat)))
```

---

## Assumptions

- **Nadir camera:** lens points straight down, no pitch/roll offset.
- **Flat earth:** valid for the short ranges (~100 m) in RobotX courses.
- **No yaw:** drone heading is held North (`WP_YAW_BEHAVIOR=0` in ArduCopter). The image +x axis maps to East.
- **Known AGL altitude:** passed as `--altitude-m`. In simulation this is fixed (10 m). In real flight, use the MAVLink altitude from the flight controller if available.

---

## Camera intrinsics (calibrated 2026-04-29)

Stored in `calibration/camera_intrinsics_latest.json`.

| Parameter | Value |
|-----------|-------|
| fx | 1319.071398 px |
| fy | 1407.4984 px |
| cx | 870.93493 px |
| cy | 533.095324 px |
| Image size | 1920 × 1080 |
| RMS reprojection error | 1.057 px |

**Distortion model:** Brown-Conrady (plumb-bob)

| Coefficient | Value |
|-------------|-------|
| k1 | 0.4353804318 |
| k2 | −0.2461908227 |
| p1 | −0.0688481028 |
| p2 | −0.0508214506 |
| k3 | 0.111509133 |

**Calibration rig:** 11×8 checkerboard (10×7 inner corners), 1-inch (0.0254 m) squares, 40 frames.

Pass `--calibration calibration/camera_intrinsics_latest.json` to enable undistortion.
In Gazebo (ogre2), undistortion is skipped — the render engine ignores the `<distortion>` block and produces a clean pinhole image. All sim launchers already pass `--no-undistort`.

---

## Origin (datum) for simulation

The Gazebo world origin is set at `lat=-35.363262, lon=149.165237` (Canberra SITL default).
`fly_course.py` reads the MAVLink GPS fix and sets this as the origin automatically.

For real flight: pass `--origin-lat` and `--origin-lon` to `camera_live_feed.py`, or let it read from MAVLink.

---

## Expected GPS error budget (simulation, Course 1)

From `simulation/accuracy_report.md` (run 2026-06-26):

| Metric | Value |
|--------|-------|
| Buoys detected | 6 / 6 |
| Mean horizontal error | 0.16 m |
| Max horizontal error | 1.04 m |
| Mean detection confidence | 0.87 |
| Unmatched detections (noise) | 0 |

The dominant error source at 10 m AGL with fx≈1319 is altitude uncertainty (~1% altitude error → ~0.1 m ground error per 10 m range), not calibration.

---

## Error sources in real flight

| Source | Impact | Mitigation |
|--------|--------|-----------|
| Altitude uncertainty (baro, not GPS) | ~1% per % error | Use GPS barometric altitude; fly consistent AGL |
| Drone pitch/roll during flight | Lateral pixel shift at edges | Add IMU attitude correction (future work) |
| Lens distortion (real camera) | Up to ~5 px at corners | Always pass `--calibration` on real hardware |
| Datum/origin error | Constant offset on all detections | Set datum from GPS fix at takeoff point |
