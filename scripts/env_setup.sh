#!/usr/bin/env bash

# Source the fixed ROS Noetic environment for this thesis workspace.
source /opt/ros/noetic/setup.bash

if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
  source "$HOME/catkin_ws/devel/setup.bash"
  echo "[env_setup] ROS Noetic + ~/catkin_ws loaded."
else
  echo "[env_setup] ROS Noetic loaded. ~/catkin_ws/devel/setup.bash not found yet."
fi
