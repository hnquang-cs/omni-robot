#!/usr/bin/env bash

set -euo pipefail

SCENARIO="${1:-manual}"
ALGORITHM="${2:-gmapping_lidar}"
DURATION="${DURATION:-60}"
PKG_PATH="$(rospack find slam_benchmark)"
OUT_DIR="${PKG_PATH}/results/stage9/raw"
mkdir -p "${OUT_DIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PREFIX="${OUT_DIR}/${SCENARIO}_${ALGORITHM}_${STAMP}"
BAG_PATH="${BAG_PREFIX}.bag"
INFO_PATH="${BAG_PREFIX}_info.txt"

TOPICS=(/lidar/scan /odom /map /tf /tf_static /clock /cmd_vel /gazebo/model_states)

echo "Recording ${DURATION}s to ${BAG_PATH}"
timeout "${DURATION}" rosbag record -O "${BAG_PATH}" "${TOPICS[@]}" || status=$?
status="${status:-0}"
if [[ "${status}" != "0" && "${status}" != "124" ]]; then
  echo "FAIL: rosbag record exited with status ${status}"
  exit "${status}"
fi

if [[ ! -s "${BAG_PATH}" ]]; then
  echo "FAIL: bag was not created: ${BAG_PATH}"
  exit 1
fi

rosbag info "${BAG_PATH}" | tee "${INFO_PATH}"
FAILED=0
for topic in /lidar/scan /odom /map /tf /gazebo/model_states; do
  if grep -qE "^[[:space:]]*${topic}[[:space:]]" "${INFO_PATH}"; then
    echo "PASS: ${topic} recorded"
  else
    echo "FAIL: ${topic} missing from bag"
    FAILED=1
  fi
done

echo "Bag: ${BAG_PATH}"
echo "Info: ${INFO_PATH}"
exit "${FAILED}"
