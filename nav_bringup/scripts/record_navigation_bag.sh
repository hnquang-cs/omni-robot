#!/usr/bin/env bash

set -euo pipefail

NAME="${1:-navigation_trial}"
DURATION="${DURATION:-60}"
PKG_PATH="$(rospack find nav_bringup)"
OUT_DIR="${PKG_PATH}/results/stage10/bags"
LOG_DIR="${PKG_PATH}/results/stage10/logs"
mkdir -p "${OUT_DIR}" "${LOG_DIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="${OUT_DIR}/${NAME}_${STAMP}.bag"
INFO="${LOG_DIR}/${NAME}_${STAMP}_bag_info.txt"

TOPICS=(/map /lidar/scan /odom /tf /tf_static /clock /cmd_vel /amcl_pose /particlecloud /move_base/status /move_base/goal /move_base_simple/goal /move_base/global_costmap/costmap /move_base/local_costmap/costmap /gazebo/model_states)
if rostopic list 2>/dev/null | grep -qx /cmd_vel_raw; then
  TOPICS+=("/cmd_vel_raw")
fi
for topic in /move_base/GlobalPlanner/plan /move_base/NavfnROS/plan /move_base/TebLocalPlannerROS/local_plan; do
  if rostopic list 2>/dev/null | grep -qx "${topic}"; then
    TOPICS+=("${topic}")
  fi
done

echo "Recording ${DURATION}s to ${BAG}"
timeout "${DURATION}" rosbag record -O "${BAG}" "${TOPICS[@]}" || status=$?
status="${status:-0}"
if [[ "${status}" != "0" && "${status}" != "124" ]]; then
  echo "FAIL: rosbag record exited with status ${status}"
  exit "${status}"
fi

if [[ ! -s "${BAG}" ]]; then
  echo "FAIL: bag was not created: ${BAG}"
  exit 1
fi

rosbag info "${BAG}" | tee "${INFO}"
echo "Bag: ${BAG}"
echo "Info: ${INFO}"
