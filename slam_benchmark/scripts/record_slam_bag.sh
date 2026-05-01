#!/usr/bin/env bash

# Record lightweight SLAM topics for repeatable benchmarking.

set -euo pipefail

PKG_PATH="$(rospack find slam_benchmark)"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_NAME="${1:-slam_${STAMP}}"
OUTPUT_PATH="${PKG_PATH}/bags/${BAG_NAME}.bag"
TOPICS=(/scan /odom /tf /tf_static /clock /cmd_vel /map)

mkdir -p "${PKG_PATH}/bags"

if ! rostopic list >/dev/null 2>&1; then
  echo "FAIL: ROS master is not reachable"
  exit 1
fi

echo "Recording bag: ${OUTPUT_PATH}"
echo "Command: rosbag record -O ${OUTPUT_PATH} ${TOPICS[*]}"
rosbag record -O "${OUTPUT_PATH}" "${TOPICS[@]}"
