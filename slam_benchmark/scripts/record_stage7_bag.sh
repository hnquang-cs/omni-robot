#!/usr/bin/env bash

# Record lightweight Stage 7 SLAM topics for repeatable benchmarking.

set -euo pipefail

WITH_CAMERA=false
BAG_NAME=""

while [ "$#" -gt 0 ]; do
  case "$1" in
    --with-camera)
      WITH_CAMERA=true
      shift
      ;;
    -h|--help)
      echo "Usage: rosrun slam_benchmark record_stage7_bag.sh [--with-camera] [bag_name]"
      exit 0
      ;;
    *)
      BAG_NAME="$1"
      shift
      ;;
  esac
done

PKG_PATH="$(rospack find slam_benchmark)"
STAMP="$(date +%Y%m%d_%H%M%S)"
if [ -z "${BAG_NAME}" ]; then
  BAG_NAME="stage7_${STAMP}"
fi
OUTPUT_PATH="${PKG_PATH}/bags/${BAG_NAME}.bag"

TOPICS=(
  /scan
  /odom
  /tf
  /tf_static
  /clock
  /cmd_vel
  /gazebo/model_states
)

if [ "${WITH_CAMERA}" = true ]; then
  TOPICS+=(
    /stereo/left/image_raw
    /stereo/right/image_raw
    /stereo/left/camera_info
    /stereo/right/camera_info
    /stereo/points2
  )
fi

mkdir -p "${PKG_PATH}/bags"

if ! rostopic list >/dev/null 2>&1; then
  echo "FAIL: ROS master is not reachable"
  exit 1
fi

echo "Recording bag: ${OUTPUT_PATH}"
echo "Camera topics: ${WITH_CAMERA}"
echo "Command: rosbag record -O ${OUTPUT_PATH} ${TOPICS[*]}"
rosbag record -O "${OUTPUT_PATH}" "${TOPICS[@]}"
