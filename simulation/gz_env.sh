#!/usr/bin/env bash
# Shared Gazebo-Harmonic environment for the RobotX UAV course.
#
# Source this (don't execute it) to get the GZ_SIM_* resource/plugin paths the
# world needs so its model:// includes + the VRX ocean plugins resolve:
#   - simulation/gazebo/models (THIS repo)   -> iris_uav, gimbal_nadir   [MUST be first]
#   - ardupilot_gazebo/models                -> iris_with_standoffs
#   - ardupilot_gazebo/worlds                -> reference worlds
#   - vrx_gz/share/vrx_gz/models             -> coast_waves (animated ocean)
# plus the plugin dirs (ardupilot_gazebo/build + vrx_gz/lib) and LD_LIBRARY_PATH
# for libWaves.so which libWaveVisual.so dlopen's by soname.
#
# This is the single source of truth for those paths; both
# run_robotx_uav_sitl.sh and run_demo_windows.sh source it so they can't drift.
#
# Override any toolchain location via env before sourcing (ARDUPILOT_GAZEBO,
# ARDUPILOT, VRX_GZ). REPO_ROOT is auto-derived from this file's location if unset.

REPO_ROOT="${REPO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

# Toolchains live OUTSIDE the repo (override via env if relocated).
ARDUPILOT_GAZEBO="${ARDUPILOT_GAZEBO:-$HOME/ardupilot_gazebo}"
ARDUPILOT="${ARDUPILOT:-$HOME/ardupilot}"
# VRX (Virtual RobotX): supplies the animated ocean -- the coast_waves model
# (model://coast_waves) and its plugins libWaveVisual.so + libPublisherPlugin.so.
VRX_GZ="${VRX_GZ:-$HOME/vrx_ws/install/vrx_gz}"

export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ARDUPILOT_GAZEBO}/build:${VRX_GZ}/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_RESOURCE_PATH="${REPO_ROOT}/simulation/gazebo/models:${ARDUPILOT_GAZEBO}/models:${ARDUPILOT_GAZEBO}/worlds:${VRX_GZ}/share/vrx_gz/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
# libWaveVisual.so dlopen's libWaves.so by soname; the dynamic linker needs it on the path.
export LD_LIBRARY_PATH="${VRX_GZ}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
