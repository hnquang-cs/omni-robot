#!/usr/bin/env bash

# Replay a SLAM bag with simulated time for repeatable Gmapping tests.

set -euo pipefail

if [ "$#" -lt 1 ]; then
  echo "Usage: rosrun slam_benchmark play_bag_gmapping.sh <bag_path>"
  echo "Terminal 1: roslaunch slam_benchmark gmapping_sim.launch"
  echo "Terminal 2: rosrun slam_benchmark play_bag_gmapping.sh <bag_path>"
  echo "After replay: rosrun slam_benchmark save_map.sh gmapping_replay"
  exit 1
fi

BAG_PATH="$1"

if [ ! -f "${BAG_PATH}" ]; then
  echo "FAIL: bag file not found: ${BAG_PATH}"
  exit 1
fi

if ! rosparam set /use_sim_time true >/dev/null 2>&1; then
  echo "FAIL: ROS master is not reachable. Start roscore or a SLAM launch first."
  exit 1
fi

echo "Playing bag with /clock: ${BAG_PATH}"
echo "Run gmapping_sim.launch in another terminal before or during playback."
rosbag play --clock "${BAG_PATH}"
