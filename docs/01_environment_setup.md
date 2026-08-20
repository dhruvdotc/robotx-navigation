# Environment Setup

Everything needed to run the pipeline from scratch on each platform.

---

## Mac (ground station only)

The Mac only runs the ground station receiver - no heavy dependencies.

```bash
# Python 3.10+ with pymavlink
conda create -y -n robotx python=3.10 pip
conda activate robotx
pip install pymavlink

# Clone repo
git clone <repo-url> ~/Downloads/ROBOTX/robotx-navigation
cd ~/Downloads/ROBOTX/robotx-navigation
```

**Camera permission** (if using Mac camera for testing):
System Settings → Privacy & Security → Camera → enable Terminal/iTerm.
If it still fails: `tccutil reset Camera`

---

## Mac (simulation)

Native macOS cannot run the Gazebo/VRX sim stack directly, for three reasons that are all upstream, not this repo:

- Gazebo Harmonic has an open bug where any robot with a rendering sensor (our nadir camera) can crash the sim on macOS - Metal contexts are only allowed on the main thread, but gz-sim's sensor renderer creates them off-thread ([gz-sim#2877](https://github.com/gazebosim/gz-sim/issues/2877)).
- VRX (the animated-ocean plugin) has no confirmed working build for the Harmonic-era branch this repo uses on macOS.
- ROS 2 Humble has no official Apple Silicon build. RoboStack's conda packages exist, but the `ros-gz` bridge for Harmonic has no `arm64` binary at all, only `amd64` ([ros_gz#614](https://github.com/gazebosim/ros_gz/issues/614)).

The reliable path is to run the same Ubuntu 22.04 setup documented below, unmodified, inside a Linux VM - using `amd64` packages via Rosetta so nothing is missing:

1. Install [OrbStack](https://orbstack.dev) (native Apple Silicon, free for personal use) or [UTM](https://mac.getutm.app/).
2. Create an `amd64` Ubuntu 22.04 machine - the `arm64` build of the ROS-Gazebo bridge package doesn't exist, `amd64` does, and Rosetta makes this close to native speed:
   ```bash
   orb create --arch amd64 ubuntu:jammy robotx-sim
   orb -m robotx-sim
   ```
3. Inside the machine, clone the repo and follow the **Ubuntu 22.04 / WSL (simulation)** steps below exactly as written.

Headless runs (`bash simulation/run_course.sh --course 1`, no `--visual`) are the reliable path - the camera sensor renders in software inside the VM, no display needed. The 4-window `--visual` GUI mode is not: it needs an X11 server on the Mac host (XQuartz), and OpenGL apps forwarded that way - which includes Gazebo's own 3D viewer - are known to fail even with software rendering. Stick to headless mode on Mac; use `--visual` on a real Ubuntu box or WSLg if you need the live windows.

Expect slower wall-clock time than native Linux - Rosetta translation plus virtualization adds overhead to the physics/render loop.

---

## Ubuntu 22.04 / WSL (simulation)

### 1. Base Python environment

```bash
conda create -y -n robotx python=3.10 opencv numpy pip
conda activate robotx
pip install pymavlink
```

Or without conda:
```bash
pip3 install opencv-python numpy pymavlink
```

### 2. ROS 2 Humble

```bash
# Standard ROS 2 Humble install - https://docs.ros.org/en/humble/Installation.html
source /opt/ros/humble/setup.bash
```

Verify: `ros2 --version` should print `ros2 cli ... humble`

### 3. Gazebo Harmonic

```bash
# https://gazebosim.org/docs/harmonic/install
export GZ_VERSION=harmonic
gz sim --version   # should print Gazebo Harmonic
```

### 4. ArduPilot SITL

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
./waf configure --board sitl
./waf copter
```

### 5. ardupilot_gazebo plugin

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo
cd ~/ardupilot_gazebo
mkdir build && cd build
cmake ..
make -j$(nproc)
```

The compiled `.so` files land in `~/ardupilot_gazebo/build/`.

### 6. VRX (animated ocean)

```bash
mkdir -p ~/vrx_ws/src
cd ~/vrx_ws/src
git clone https://github.com/osrf/vrx
cd ~/vrx_ws
colcon build
```

VRX install path used by the repo: `~/vrx_ws/install/vrx_gz`.

### 7. ROS–Gazebo image bridge

```bash
sudo apt install ros-humble-ros-gz-image
```

### 8. Verify all paths

The file `simulation/gz_env.sh` is the single source of truth for `GZ_SIM_*` resource and plugin paths. Source it before running anything manually:

```bash
source simulation/gz_env.sh
echo $GZ_VERSION   # should print: harmonic
```

---

## Jetson Orin Nano (onboard detector)

```bash
ssh babydragon@<JETSON_IP>
cd ~/robotx-navigation
bash jetson_setup.sh
```

`jetson_setup.sh` installs: `python3-opencv` (from apt), `numpy`, `pymavlink`, `future`, `ultralytics`, and clones the MAVCore vendor library. It creates `.venv-mavlink` - activate it before running any Python scripts.

> **Note:** the `ultralytics` install pulls a generic PyPI torch wheel, which is not guaranteed to use Jetson's GPU. For real on-device inference speed, install a JetPack-matched PyTorch build first, then `pip install --no-deps ultralytics` to avoid overwriting it.

Make sure `captures/classes/` exists at `~/robotx-navigation/captures/classes/` with reference crops (`red.jpg`, `green.jpg`, `blue.jpg`) - `camera_live_feed.py` loads HSV ranges from this directory at startup.

### WiFi setup (field router)

```bash
bash scripts/jetson_wifi_setup.sh    # connect Jetson to field router
bash scripts/jetson_heartbeat.sh     # keep WiFi alive
bash scripts/jetson_probe_from_laptop.sh  # verify connectivity from Mac
```

---

## Software version pinboard

| Component | Version |
|-----------|---------|
| OS | Ubuntu 22.04 |
| Python | 3.10 |
| ROS 2 | Humble (`/opt/ros/humble`) |
| Gazebo | Harmonic (`GZ_VERSION=harmonic`) |
| ArduPilot SITL | `~/ardupilot` - ArduCopter, `gazebo-iris` frame |
| ardupilot_gazebo | `~/ardupilot_gazebo` (`build/*.so`) |
| VRX | `~/vrx_ws/install/vrx_gz` |
| Render engine | ogre2 |
| SDF spec | 1.9 (models) / 1.10 (worlds) |
| OpenCV | latest compatible with Python 3.10 |
| pymavlink | latest |
