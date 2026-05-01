#!/usr/bin/env bash

# Replay a Stage 7 bag with Hector SLAM only. Gazebo is not launched.

set -euo pipefail

if [ "$#" -lt 1 ]; then
  echo "Usage: rosrun slam_benchmark replay_with_hector.sh <bag_path>"
  exit 1
fi

if ! rospack find hector_mapping >/dev/null 2>&1; then
  echo "FAIL: hector_mapping package is not installed"
  echo "Install with: sudo apt install ros-noetic-hector-slam"
  exit 1
fi

BAG_PATH="$1"
EXIT_AFTER_PLAY="${EXIT_AFTER_PLAY:-false}"
if [ ! -f "${BAG_PATH}" ]; then
  echo "FAIL: bag file not found: ${BAG_PATH}"
  exit 1
fi

cleanup() {
  if [ -n "${LAUNCH_PID:-}" ] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

echo "Command: roslaunch slam_benchmark hector_sim.launch use_sim_time:=true"
roslaunch slam_benchmark hector_sim.launch use_sim_time:=true &
LAUNCH_PID=$!

sleep 3
rosparam set /use_sim_time true
echo "Command: rosbag play --clock ${BAG_PATH}"
rosbag play --clock "${BAG_PATH}"

echo "Replay finished."
echo "Save Hector map with: rosrun slam_benchmark save_map_with_prefix.sh hector /map_hector"
if [ "${EXIT_AFTER_PLAY}" = "true" ]; then
  exit 0
fi

echo "Hector is still running so the map can be saved. Press Ctrl-C here when done."
wait "${LAUNCH_PID}"
