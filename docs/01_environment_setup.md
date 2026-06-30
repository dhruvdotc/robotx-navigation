# Environment Setup

Everything needed to run the pipeline from scratch on each platform.

---

## Mac (ground station only)

The Mac only runs the ground station receiver — no heavy dependencies.

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
# Standard ROS 2 Humble install — https://docs.ros.org/en/humble/Installation.html
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

`jetson_setup.sh` installs: `opencv-python`, `numpy`, `pymavlink`, `ultralytics` (YOLO), and other deps via `pip3`.

Make sure `buoy_best.pt` is present at `~/robotx-navigation/buoy_best.pt` before running the detector.

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
| ArduPilot SITL | `~/ardupilot` — ArduCopter, `gazebo-iris` frame |
| ardupilot_gazebo | `~/ardupilot_gazebo` (`build/*.so`) |
| VRX | `~/vrx_ws/install/vrx_gz` |
| Render engine | ogre2 |
| SDF spec | 1.9 (models) / 1.10 (worlds) |
| OpenCV | latest compatible with Python 3.10 |
| pymavlink | latest |
