# RobotX UAV Sim — ROS 2 Humble + Gazebo Fortress (ARM64 / amd64)
# Headless-only. Tests Layers 1-2b of the Simulink bridge pipeline.
#
# Gazebo version: FORTRESS (ignition-gazebo6 / ign sim)
# Why Fortress, not Harmonic: ros-humble-ros-gz-bridge on Ubuntu 22.04 Jammy
# is compiled against libignition-msgs8 + libignition-transport11 (Fortress).
# Using Harmonic (gz-transport13) causes "Unknown message type" errors in the
# bridge because the two transport versions cannot communicate.
#
# What's included:
#   - Gazebo Fortress (ign sim, OdometryPublisher built-in)
#   - ros_gz_bridge + ros_gz_image (ROS 2 ↔ Gazebo Fortress topic bridge)
#   - Python: pymavlink, opencv-python-headless, rclpy (via ROS)
#
# What's NOT included:
#   - ardupilot_gazebo plugin (not needed for bridge topic test)
#   - ArduPilot SITL binary (not needed for Layer 1-2b)
#   - ultralytics / YOLO (not needed for topic verification)
#
# Build:
#   colima ssh -- docker build -f docker/sim.Dockerfile \
#     -t robotx-sim .
#
# Run e2e test (from repo root):
#   docker run --rm -v $(pwd):/ws/robotx-navigation robotx-sim \
#     bash /ws/robotx-navigation/docker/run_e2e_test.sh

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ── 1. Gazebo Fortress (ignition-gazebo6) ─────────────────────────────────────
# Fortress uses "ign sim" (not "gz sim") and ignition-transport11 which
# matches what ros-humble-ros-gz-bridge links against.
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg lsb-release sudo ca-certificates \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
         -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) \
         signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
         http://packages.osrfoundation.org/gazebo/ubuntu-stable \
         $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
         ignition-fortress \
    && rm -rf /var/lib/apt/lists/*

# ── 2. ROS–Gazebo bridge (compiled against Fortress / libignition-*) ──────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-gz-bridge \
      ros-humble-ros-gz-image \
      ros-humble-nav-msgs \
      ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# ── 3. Python deps ────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir pymavlink opencv-python-headless

# ── 4. Workspace setup ───────────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /ws/robotx-navigation

CMD ["bash"]
