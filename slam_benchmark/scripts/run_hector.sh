#!/usr/bin/env bash

# Launch Hector SLAM comparison if the ROS package is installed.

set -euo pipefail

if ! rospack find hector_mapping >/dev/null 2>&1; then
  echo "FAIL: hector_mapping package is not installed"
  echo "Install with: sudo apt install ros-noetic-hector-slam"
  exit 1
fi

echo "Command: roslaunch slam_benchmark hector_sim.launch"
roslaunch slam_benchmark hector_sim.launch
