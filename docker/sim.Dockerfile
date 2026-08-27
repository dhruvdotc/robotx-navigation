# RobotX UAV Sim — ROS 2 Humble + Gazebo Harmonic (ARM64 / amd64)
# Headless only: no display required. ArduPilot SITL installed from binaries.
#
# Build:
#   docker build -f docker/sim.Dockerfile -t robotx-sim .
#
# Run (headless Course 1):
#   docker run --rm -it \
#     -v $(pwd):/ws/robotx-navigation \
#     --network host \
#     robotx-sim \
#     bash simulation/run_course.sh --course 1

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ── 1. Gazebo Harmonic ────────────────────────────────────────────────────────
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
         gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# ── 2. ROS–Gazebo bridge packages ────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-gz-bridge \
      ros-humble-ros-gz-image \
      ros-humble-ros-gz-sim \
      ros-humble-nav-msgs \
      ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# ── 3. ArduPilot SITL (pre-built binary for the container arch) ──────────────
# Uses the official ArduPilot Tools PPA binary — no 30-min source build.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-dev git wget xterm \
    && pip3 install --no-cache-dir mavproxy pymavlink \
    && rm -rf /var/lib/apt/lists/*

# ArduPilot SITL binary (ArduCopter, matches the ardupilot_gazebo plugin ABI).
# We pull the latest stable ArduCopter SITL binary for the current arch.
RUN ARCH=$(uname -m) \
    && if [ "$ARCH" = "aarch64" ]; then GH_ARCH="arm64"; else GH_ARCH="x86_64"; fi \
    && mkdir -p /opt/ardupilot \
    && wget -qO /opt/ardupilot/arducopter \
         "https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter" \
         || wget -qO /opt/ardupilot/arducopter \
              "https://firmware.ardupilot.org/Copter/stable/SITL_arm_linux_gnueabihf/arducopter" \
    && chmod +x /opt/ardupilot/arducopter
ENV PATH="/opt/ardupilot:${PATH}"

# ── 4. ardupilot_gazebo plugin (pre-built .so) ───────────────────────────────
# Clones and builds the plugin — required for ArduPilot ↔ Gazebo lockstep.
RUN apt-get update && apt-get install -y --no-install-recommends \
      cmake build-essential libgz-sim8-dev rapidjson-dev \
    && git clone --depth=1 https://github.com/ArduPilot/ardupilot_gazebo \
         /opt/ardupilot_gazebo \
    && cmake -S /opt/ardupilot_gazebo -B /opt/ardupilot_gazebo/build \
         -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /opt/ardupilot_gazebo/build -- -j4 \
    && cmake --install /opt/ardupilot_gazebo/build \
    && rm -rf /var/lib/apt/lists/*

ENV GZ_SIM_SYSTEM_PLUGIN_PATH="/usr/local/lib/gz/sim8/plugins"
ENV GZ_SIM_RESOURCE_PATH="/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds"

# ── 5. Python deps for the detector + training pipeline ──────────────────────
RUN pip3 install --no-cache-dir \
      ultralytics \
      opencv-python-headless \
      numpy \
      albumentations

# ── 6. Workspace setup ───────────────────────────────────────────────────────
WORKDIR /ws/robotx-navigation

# Source ROS on every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

COPY simulation/gz_env.sh /opt/gz_env.sh
RUN echo "source /opt/gz_env.sh" >> /root/.bashrc

CMD ["bash"]
