# Simulink GPS Sensor Model

MATLAB/Simulink integration for injecting realistic GPS sensor noise into the
Gazebo simulation, using Navigation Toolbox's `gpsSensor` blocks against live
drone odometry.

## What's here

| File | Purpose |
|---|---|
| `gps_sim_navtoolbox.slx` | The Simulink model. Subscribes to live Gazebo odometry, runs it through 4 real `gpsSensor` blocks (Navigation Toolbox), publishes noisy `sensor_msgs/NavSatFix` on `/fix`. |
| `build_gps_model.m` | Builds `gps_sim_navtoolbox.slx` from scratch via the Simulink API. Run this if you need to regenerate or modify the model programmatically rather than by hand. |
| `run_navtoolbox_model.m` | Standalone test script — opens the model, subscribes to `/fix` independently to verify output, runs the sim. Use this to confirm the model itself works before wiring in SITL. |
| `fix_to_gps_input.py` | Bridges `/fix` into ArduPilot SITL as a MAVLink `GPS_INPUT` message, to close the loop so the autopilot's own EKF (not just an offline comparison) uses the Simulink-noisy GPS. Also has a standalone `--diag-offset-test` mode that checks GPS_INPUT → EKF fusion against SITL alone, no Gazebo/ROS2/Simulink needed. |
| `gps_mav_params.parm` | ArduPilot default-params file (`GPS1_TYPE 14`) — load at SITL boot via `--add-param-file` so the MAV GPS driver is configured from the start, no live reboot needed. |
| `gps_sim_navtoolbox_diagram.png` | Block diagram screenshot (below). |
| `pipeline_architecture.svg` | Full pipeline diagram (below) — where the `.slx` sits relative to Gazebo, the ROS 2 bridge, `fix_to_gps_input.py`, and ArduPilot's EKF, with per-stage verification status as of 2026-09-05. |
| `../simulation/mock_fix_publisher.py` | Python stand-in for the `.slx` with the identical `/fix` interface — used for every live end-to-end verification in this doc since no environment touched had MATLAB installed. Includes the buoy-proximity GPS degradation feature (see "Realism improvements" below). |
| `buoy_proximity_verification.png` | Real data from a live flight, showing GPS accuracy degrading near buoys/gates (below). |
| `../simulation/gazebo/models/iris_with_standoffs/` | Repo-owned override adding realistic Gazebo-native IMU sensor noise (see "IMU sensor noise" below). Takes precedence over the toolchain's copy via `GZ_SIM_RESOURCE_PATH` order; omits the 22MB `meshes/` directory (headless-only for now — see the caveat in that section). |
| `imu_noise_verification.png` | Real data from a live flight, showing raw IMU noise (below). |

## Pipeline architecture

![Full pipeline architecture](pipeline_architecture.svg)

This is the whole chain from Gazebo physics to ArduPilot's own position
estimate, with each stage's verification status as of 2026-09-05. Everything
is confirmed working live end to end **except** the `.slx` model's own
MATLAB-side execution, which no environment reachable during this
investigation could run (no MATLAB installed) — its `mock_fix_publisher.py`
stand-in (identical `/fix` output contract) is what actually produced every
verified result on this page.

## Block diagram (the actual `.slx` model)

![gps_sim_navtoolbox block diagram](gps_sim_navtoolbox_diagram.png)

Flow: `OdomSub` (real `/model/iris_uav/odometry`) → `PosVelSel`/`MakeVectors`
(extract N/E position+velocity) → four parallel `GPS_RTKInt` / `GPS_RTKFloat`
/ `GPS_DGPS` / `GPS_Std` blocks (each a differently-configured `gpsSensor`
instance) → `ModeSel` (probabilistic fix-quality state machine, ~8s mean
dwell time) picks which one is "active" via `LLASwitch` → `AssignFix`/
`AssignStatus`/`HdrAssign` build the full `NavSatFix` message → `FixPub`
publishes on `/fix`.

## Prerequisites

- MATLAB **R2023b, R2024a, or R2024b** specifically. ROS Toolbox's ROS 2
  support is release-gated: R2022b/R2023a only support Foxy, R2025a+ only
  support Jazzy — neither talks to this stack's ROS 2 **Humble**. This was
  built and tested on R2024b.
- Toolboxes: **Simulink**, **Robotics System Toolbox**, **ROS Toolbox**,
  **Navigation Toolbox** (or Sensor Fusion and Tracking Toolbox / UAV
  Toolbox — any one of the three provides `sensorgpslib`).
- The rest of this repo's simulation stack running: Gazebo Harmonic +
  ArduPilot SITL + `ardupilot_gazebo` + the odometry/IMU `ros_gz_bridge`
  (see `docs/01_environment_setup.md` and `docs/11_simulink_sensor_sim.md`).
  Note: the apt-packaged `ros-humble-ros-gz-bridge` is compiled against
  Gazebo **Fortress**, not Harmonic — it silently fails to relay any
  messages against a Harmonic Gazebo (no error, just zero messages ever
  delivered). Build `ros_gz` from source with `GZ_VERSION=harmonic` set
  before `colcon build`, or the odometry bridge will look connected but
  never actually pass data. Confirmed hitting this live 2026-09-05 — see
  the "Full pipeline now also verified live" note in Status below.
  `simulation/run_course.sh` now defaults to a source-built overlay at
  `~/ros_gz_overlay_setup.sh` automatically when present, so this is
  handled for you there; it's only a manual concern if you're launching
  the bridge yourself outside that script (as this doc's own commands do).

## How to run (Simulink model only, no SITL closed loop)

1. Start Gazebo + SITL + the odometry bridge:
   ```bash
   bash simulation/run_robotx_uav_sitl.sh --headless
   # in another terminal:
   source /opt/ros/humble/setup.bash
   ros2 run ros_gz_bridge parameter_bridge \
     "/model/iris_uav/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
     "/world/robotx_uav_course/model/uav/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
   ```
   Verify with `ros2 topic hz /model/iris_uav/odometry` before continuing.
2. Open `gps_sim_navtoolbox.slx` in MATLAB and hit Run (or `Ctrl+T`), or run
   `run_navtoolbox_model.m` for a scripted test with `/fix` logged to a file.
3. Check output: `ros2 topic echo /fix`, or `accuracy_verify.py --use-fix`.

The model must be **actively running**, not just open, to do anything —
`EnablePacing`/`PacingRate=1` (already set) keeps its simulated clock
synced to wall-clock time, which is required for real ROS 2 messages to
have a chance to arrive during each timestep.

## How to run (closing the loop into ArduPilot's own EKF)

This part injects the noisy GPS back into SITL so the autopilot's actual
navigation solution uses it. The mechanism is now confirmed end to end at the
SITL level (see **Status** below: `GPS_INPUT` → EKF3 → `GLOBAL_POSITION_INT`
verified 2026-09-05, 100% tracked offset) — what's still open is running it
against the real Gazebo+Simulink pipeline with the vehicle actually flying.

1. Launch SITL with the GPS1_TYPE param baked in from boot (critical — do
   **not** try to set this live via MAVLink and reboot only SITL; that
   desyncs Gazebo's lockstep loop and silently stalls the whole sim):
   ```bash
   python3 $ARDUPILOT/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
     --model JSON --no-mavproxy --no-rebuild -I0 \
     --add-param-file=simulink/gps_mav_params.parm
   ```
2. Optional sanity check before touching Gazebo/Simulink — rerun the
   standalone diagnostic (needs nothing but a running SITL instance) if
   you've changed anything in the GPS_INPUT path since the last confirmed
   run:
   ```bash
   python3 simulink/fix_to_gps_input.py --connect tcp:127.0.0.1:5760 \
     --skip-reboot --diag-offset-test
   ```
   It injects a slow, deliberate 5m northward position ramp (with matching
   velocity, so the ignore-flags path isn't a confounding variable) straight
   into SITL and checks whether `GLOBAL_POSITION_INT` follows it, printing a
   `PASS`/`AMBIGUOUS`/`FAIL` verdict with the actual tracked displacement. It
   isolates the GPS_INPUT → EKF path from Gazebo, the ROS 2 bridge, and
   Simulink entirely, so a failure here can't be blamed on any of those.
3. Once the diagnostic passes, start the real bridge:
   ```bash
   python3 simulink/fix_to_gps_input.py --connect tcp:127.0.0.1:5760 --skip-reboot
   ```
4. Run the Simulink model (see above). The bridge log will show
   `Injected N GPS_INPUT messages` and periodic `ArduPilot EKF believes: ...`
   lines — compare those against Gazebo's real position (which now needs to
   actually be flying a course, e.g. via `fly_course.py`, for this comparison
   to mean anything) to see whether the EKF is tracking the injected noise.

## Status — read before relying on this

**Verified working:**
- The Simulink model itself: live subscription to real Gazebo odometry,
  four real `gpsSensor` instances with independent noise configs, a
  fix-quality state machine, full `NavSatFix` construction (position,
  status, covariance, header timestamp/frame_id) — 126 real messages
  captured in testing, all fields correct.
- The `GPS_INPUT` injection mechanism: connects to SITL, configures
  `GPS1_TYPE`, sends properly-formatted messages, and ArduCopter accepts
  them as a healthy GPS — confirmed by successfully arming on injected
  data alone (`COMMAND_ACK` result `0`/ACCEPTED).

**Now verified (2026-09-05):** GPS_INPUT does reach ArduPilot's EKF3 and
does update `GLOBAL_POSITION_INT`. Ran `fix_to_gps_input.py --diag-offset-test`
against a freshly-built `arducopter` SITL binary (`--model +`, ArduPilot's own
built-in physics, no Gazebo needed for this specific question) with
`GPS1_TYPE=14` set: injected a 5.00m northward ramp over 15s, and
`GLOBAL_POSITION_INT` tracked **5.01m north, 0.00m east** — a clean 100%
match, first try, no glitch rejection, no damping.

The earlier "not yet verified" conclusion below is now understood to have been
a **test-design artifact, not a real finding**:
- The original test armed a stationary Iris on the ground and watched
  `GLOBAL_POSITION_INT` sit at home. The injected fix itself never moved
  either, so a frozen readout was exactly what you'd see whether fusion
  worked perfectly or was completely broken — that test had no power to tell
  the two apart, regardless of the outcome it produced.
- The theory recorded at the time — that `--model JSON`'s FDM ground-truth
  `position` field silently overrides the GPS-driven EKF estimate — does not
  hold up against the ArduPilot source (`AP_GPS_MAV`, the backend
  `GPS1_TYPE=14` selects, reads only from injected `GPS_INPUT` messages; and
  `EK3_SRC1_POSXY` defaults to GPS) and is now further contradicted by this
  direct measurement. Retired.

Run it yourself: `fix_to_gps_input.py --diag-offset-test` needs nothing but a
running `arducopter` SITL instance (no Gazebo/ROS2/Simulink) — see the
"closing the loop" section above for the exact command.

**Full pipeline now also verified live (2026-09-05):** ran Gazebo (real
physics, `robotx_uav_course.sdf`) + ArduPilot SITL (`GPS1_TYPE=14` from boot)
+ `mock_fix_publisher.py` (a Python stand-in for the unavailable-in-CI
`.slx` — same `/fix` interface, already used for the Docker E2E tests) +
`fix_to_gps_input.py` (real relay mode, not the diagnostic) + `fly_course.py`
flying Course 1 for real. Result: the vehicle **armed, took off, flew all 4
waypoints (gates 1–3 + light buoy), and landed**, while `GLOBAL_POSITION_INT`
tracked the actual ~50m of real flight distance via the injected noisy
`GPS_INPUT` stream (longitude moved by an amount equivalent to 49.8m — a
0.4% error against the real distance flown — while the perpendicular axis,
where the real course does not move, correctly stayed flat), with the
injected fix quality visibly cycling between DGPS (`h_acc=0.400m`) and
RTK-float (`h_acc=0.020m`) exactly per the mock's fix-state machine. This
closes out the item below — the full loop works, live, with the vehicle
actually flying.

Getting to that result surfaced two real, unrelated bugs, both now fixed in
this repo:
- **The sensor bridge was silently dead.** `run_course.sh` (and my own first
  attempt at this test) defaulted to `/opt/ros/humble/setup.bash` for the
  `ros_gz_bridge parameter_bridge` that feeds `/model/iris_uav/odometry` to
  Simulink/`mock_fix_publisher.py`. That's the apt-packaged, Fortress-linked
  build — see the "Prerequisites" gotcha above — and against this repo's
  Harmonic `gz sim`, it started up clean, logged no errors, and then
  delivered *zero* messages for the life of the run. Since this is also
  `run_course.sh`'s own default with no override set anywhere on this
  machine, every past full-course run through the normal team workflow has
  likely had the same silently-empty sensor bridge. Fixed:
  `simulation/run_course.sh` now defaults `ROS_SETUP` to
  `~/ros_gz_overlay_setup.sh` (a source-built ros_gz overlay) when that file
  exists, falling back to stock ROS otherwise — safe on any machine without
  that overlay, since it only changes what the *default* resolves to.
- **`mock_fix_publisher.py` had its north/east axes swapped**
  (`self._north = position.x` when Gazebo's ENU convention, confirmed by this
  world's `heading_deg=0`, makes `position.x` **East** and `position.y`
  **North**). A real, uncommitted fix for this was already sitting in a
  separate native-filesystem clone of this repo from earlier work and is what
  actually ran in the successful test above; it's now applied to the tracked
  source too.

## Realism improvements

**Buoy-proximity GPS degradation (2026-09-05, live-verified).** The original
fix-quality state machine (both in the `.slx` and `mock_fix_publisher.py`)
only varied with a time-based Markov dwell (~8s mean), completely decoupled
from where the vehicle actually is. Real GPS multipath and partial-sky
occlusion off a structure at antenna height is a real, well-documented effect,
and the RobotX courses are dense with exactly this kind of structure (gates
15m apart, buoys ~0.5m tall). `mock_fix_publisher.py` now computes distance to
the nearest real buoy/gate (positions read directly off each course's world
SDF — `--course 1/2/3`) and applies a **continuous** accuracy-degradation
multiplier (up to 7x the base sigma) within `--degrade-radius` (default 8m),
on top of the existing state-quality label.

**This took two iterations to get right, and the failure is worth recording.**
The first version only biased the Markov chain's transition *weights* near a
buoy, still gated by the ~8s dwell timer. Live-tested on Course 1 (~67s
flight, only ~9 total state transitions), the result came back
**backwards** — mean `h_acc` near buoys (0.284m) was *better* than in open
water (0.384m), purely because a couple of transitions happened to land on
lucky/unlucky draws while the timer, not position, was what actually gated
when the label could change. The weight bias was firing correctly (verified
in the raw log — status flipped to worst-case right as distance crossed under
8m), but with so few transitions on a short course, the discrete/probabilistic
mechanism couldn't produce a reliable correlation. Fixed by making the
degradation continuous and distance-driven instead of timer-and-probability-
driven — see the commit for both versions if you want the full before/after.

Live result, Course 1, full flight (armed → 4 waypoints → landed):

![Buoy proximity GPS degradation, live flight data](buoy_proximity_verification.png)

- 225 samples within 8m of a buoy/gate: mean `h_acc` = **5.04m**
- 74 samples in open water: mean `h_acc` = **0.22m**
- ~23x accuracy degradation near structures, tracking distance continuously
  and monotonically (visible as the sawtooth in the top panel, mirroring the
  distance-to-nearest-buoy sawtooth below it) — not a step function, and not
  dependent on transition timing.

**Follow-up fix (same day):** the first live plot had a spurious vertical
spike at the very start (multiple different `h_acc` values all at east≈0).
Root cause: the dwell timer was seeded at node-construction time, before real
odometry/motion exists — Gazebo/bridge/arm/takeoff startup latency is a few
seconds, `_MEAN_DURATION`'s exponential distribution has real mass under
that, so the initial dwell had frequently already "expired" before the
vehicle ever moved, firing one or more meaningless transitions while
stationary at the start position. Fixed by anchoring the timer to the first
real odometry sample instead of node construction (`_state_until` starts as
`None`, seeded lazily on first real data in `_publish()`). Re-verified live
— the plot above now starts flat and clean. Note the flight still holds 4s
at each waypoint, so this same class of effect (a legitimate transition
firing while briefly stationary) can still show up as a small blip at a
gate's hold point — that's expected given a spatial x-axis, not a bug.

The same idea is ported into `build_gps_model.m`'s `ModeSel` chart (**untested
— no MATLAB reachable during this investigation**), using a deterministic
override to the worst-configured `gpsSensor` instance within the radius
rather than a continuous multiplier, since each `gpsSensor`'s accuracy is
fixed at model-build time, not a runtime input. Re-verify against
`mock_fix_publisher.py`'s live-tested behaviour before trusting it blindly.

Disable with `--no-proximity-degrade` to get the original purely time-based
behaviour back; `--degrade-radius` and `--course` are also configurable.
Sanity-checked (no live sim needed) that `COURSE_BUOYS` resolves correctly
for all three courses and that the disabled path degrades safely to no-op.

**Does any of this noise actually matter to the mission? (2026-09-05,
live-verified).** Everything above proves the GPS pipeline works; it doesn't
by itself prove the noise changes anything RobotX actually cares about --
buoy-detection accuracy. `accuracy_verify.py --use-fix` already existed for
exactly this comparison but had never been run live end-to-end (per
`docs/simulink_bridge/README.md`: "code wired ✅, live test needs SITL ❌").
Ran Course 1 once with the full stack up (Gazebo, `GPS1_TYPE=14`,
`mock_fix_publisher.py`, `fix_to_gps_input.py`) and two `accuracy_verify.py`
instances watching the same real detections in parallel — one reading
ArduPilot's own EKF-fused position, one reading the raw `/fix` stream
directly via `--use-fix`:

| Position source | Mean error | Max error | Unmatched detections |
|---|---:|---:|---:|
| MAVLink (EKF-fused, GPS1_TYPE=14) | **0.22m** | 0.81m | 204/635 (32%) |
| `/fix` (raw, unfiltered) | **0.76m** | 2.95m | 454/635 (71%) |

EKF3's fusion/filtering gives a real **3.5x accuracy improvement** over using
the raw injected fix directly — exactly matching the hypothesis recorded in
`docs/11_simulink_sensor_sim.md` back when this was first built ("ArduCopter's
EKF will filter and smooth the noisy GPS, matching what happens in real
flight"), now actually measured instead of assumed. Per-buoy error also grows
gate-by-gate for the raw-`/fix` path (0.34m → 0.90m → 1.69m mean error at
gates 1→2→3) — consistent with the buoy-proximity degradation feature above
compounding as the vehicle spends more time near structures deeper into the
course. `light_buoy` shows 0 detections in both — expected, it's a nadir
black box with no color signature until `light_buoy_cycler.py`'s scan-code
sequence is running, which this test didn't launch.

### IMU sensor noise (2026-09-05, live-verified)

Unlike GPS, there's **no MAVLink injection point for IMU** — `GPS_INPUT`
lets an external process feed ArduPilot a synthetic GPS fix, but there's no
equivalent for accel/gyro. Checked `SIM_JSON.cpp` directly: with
`--model JSON` (this repo's SITL mode), `state.imu.accel_body`/`gyro` are
used completely verbatim from the external simulator's payload — ArduPilot's
own `SIM_ACC*`/`SIM_GYR*` noise params never even run. So a
`mock_fix_publisher.py`-style Python relay (the original plan) would have
been purely cosmetic — a ROS topic nobody's flight behavior reads.

The architecturally correct fix instead: Gazebo's own IMU sensor natively
supports per-axis `<noise type="gaussian">`, which flows through
`ardupilot_gazebo`'s JSON payload as genuine physics-derived sensor noise —
ArduPilot's real EKF fuses it for real. The vehicle model that carries this
sensor (`iris_with_standoffs`) lives in the separately-cloned
`ardupilot_gazebo` toolchain repo, not here, so editing it in place wouldn't
be git-tracked and could vanish on a rebuild. Since `gz_env.sh` already puts
`simulation/gazebo/models` **first** in `GZ_SIM_RESOURCE_PATH`, added a
repo-owned override at `simulation/gazebo/models/iris_with_standoffs/`
(a full copy of the upstream `model.sdf` + `<imu>` noise added to the
`imu_sensor` element) that Gazebo resolves in preference to the toolchain's
copy. Noise values match PX4's published default `gazebo_imu_plugin` sensor
model (MPU-9250-class MEMS IMU, widely reused across robotics sim projects),
converted from noise-density to per-sample stddev at this sensor's 1000Hz:
gyro stddev 0.0059 rad/s (bias_stddev 0.0087), accel stddev 0.19 m/s²
(bias_stddev 0.196).

**Known gap: the override omits the 22MB `meshes/` directory** (visual DAE
+ collision STL) rather than duplicating it into this repo. Confirmed live
that this doesn't affect physics, sensors, arming, flight, or camera-based
buoy detection — none of those load meshes in headless mode. **Not verified:
`--visual`/GUI mode**, which does render — the drone's own body may show as
a missing/placeholder mesh there. If that turns out to matter, the fix is to
either duplicate `meshes/` into the override or symlink it in per-machine
(not git-trackable either way, given the size).

Live-verified on a full Course-1 flight (armed on the first try, flew all 4
waypoints, landed — no physics regression from the override):

![Raw IMU noise, live flight data](imu_noise_verification.png)

62,530 real IMU samples at ~973Hz. High-frequency (noise-isolated, via
first-differencing to remove real flight dynamics) stddev: gyro Z ≈0.0073
rad/s vs. 0.0059 design (Z axis has the least real yaw motion, given
`WP_YAW_BEHAVIOR=0`, so it's the cleanest read on pure sensor noise); accel
Z/X ≈0.24/0.19 m/s² vs. 0.19 design. Raw accel Z mean -9.76 m/s² confirms
gravity is reported with the correct sign/magnitude. The small excess over
design targets is expected — real thrust/attitude changes during flight add
on top of the injected sensor noise; the first-difference estimate isolates
most but not all of that.

**Investigated and dropped: wind/turbulence.** ArduPilot SITL has
`SIM_WIND_SPD`/`SIM_WIND_DIR`/`SIM_WIND_TURB`, but checked `SIM_JSON.cpp`
first and they have **zero effect** in this repo's setup — same root cause
as the IMU finding above: `--model JSON` means Gazebo computes all physics,
and the JSON backend's own comment reads `"wind is not supported yet for
JSON sim, assume zero for now"`. A real version would need a genuine
Gazebo-side wind force (`gz-sim-wind-effects-system` on the vehicle model in
the world SDF) — bigger lift than a param, not attempted here. Recording
this so nobody re-discovers it by trial and error.

**Ideas not yet implemented** (future work, not attempted here):
- Multipath severity that also depends on the vehicle's altitude/look-angle
  to the obstructing structure, not just planar distance (a nadir-hovering
  drone at 10m AGL sees a 0.5m buoy very differently than one at 2m).
- Real wind via a Gazebo world-level force plugin (see above).

**Still open:** the real `.slx` Simulink model itself (as opposed to its
`mock_fix_publisher.py` stand-in) still needs a live run on a machine with
MATLAB + Navigation/ROS/Robotics-System Toolbox — none of the environments
touched during this investigation had MATLAB installed. Everything
downstream of `/fix` (the injection mechanism, the EKF fusion, the sensor
bridge, the full flight) is now confirmed working; what's unverified is
narrowly the `.slx` model's own MATLAB-side execution against a live ROS 2
bridge — everything it would need to plug into is now proven live.

**Known gotchas found along the way** (useful if you're debugging this
further):
- MATLAB's ROS Toolbox wants `nav_msgs/Odometry`, not ROS 2's own
  `nav_msgs/msg/Odometry` — a real convention mismatch, not a typo.
- Simulink `Blank Message` blocks (ros2lib) have **two** separate type
  fields, `entityType` and `messageType` — both must be set to the same
  value or you get an opaque "bus hierarchy doesn't match" error.
- `NavSatStatus.service` is `uint16`, `.status` is `int8` — Simulink's bus
  type-checking will reject anything else with a compile error naming the
  exact port.
- SITL's TCP MAVLink port (5760, when not running with MAVProxy) is
  **single-client** — a second simultaneous connection doesn't error, it
  just silently starves both of telemetry. Only ever have one MAVLink
  client connected at a time.
- ArduPilot won't answer `PARAM_REQUEST_READ` (or apparently most other
  request/response protocol messages) from a client that isn't itself
  streaming heartbeats — same as any real GCS. `param_set` (fire-and-forget)
  works without this; reading values back doesn't.
- The parameter is `GPS1_TYPE`, not `GPS_TYPE` — the latter only exists as
  a backward-compat conversion alias for loading old saved parameter files,
  not for live `PARAM_SET` over MAVLink.
