#!/usr/bin/env bash

set -euo pipefail

PKG_PATH="$(rospack find nav_bringup)"
BAG_DIR="${PKG_PATH}/results/stage10/bags"
CSV_DIR="${PKG_PATH}/results/stage10/csv"
mkdir -p "${BAG_DIR}" "${CSV_DIR}"

GOAL_X="${GOAL_X:-1.0}"
GOAL_Y="${GOAL_Y:-1.0}"
GOAL_YAW="${GOAL_YAW:-0.0}"
TIMEOUT_SEC="${TIMEOUT_SEC:-90}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="${BAG_DIR}/smooth_motion_goal_${STAMP}.bag"
ANALYSIS_CSV="${CSV_DIR}/cmd_vel_analysis.csv"
SUMMARY_CSV="${CSV_DIR}/smooth_motion_test.csv"

echo "Checking smooth navigation topics"
for topic in /map /amcl_pose /move_base/status; do
  timeout 5 rostopic echo -n 1 "${topic}" >/tmp/stage10_smooth_topic.txt 2>&1 || {
    echo "FAIL: ${topic} has no message"
    exit 1
  }
done

rosrun nav_bringup check_goal_on_map.py --x "${GOAL_X}" --y "${GOAL_Y}" --yaw "${GOAL_YAW}"

topics=(/cmd_vel /odom /tf /amcl_pose /move_base/status /gazebo/model_states /move_base/TebLocalPlannerROS/local_plan)
for optional in /cmd_vel_raw /move_base/GlobalPlanner/plan /move_base/NavfnROS/plan; do
  if rostopic list 2>/dev/null | grep -qx "${optional}"; then
    topics+=("${optional}")
  fi
done

rosbag record -O "${BAG}" "${topics[@]}" >/tmp/stage10_smooth_rosbag.log 2>&1 &
bag_pid=$!
sleep 2

start_epoch="$(date +%s)"
rosrun nav_bringup send_nav_goal.py --x "${GOAL_X}" --y "${GOAL_Y}" --yaw "${GOAL_YAW}"

success=false
status="UNKNOWN"
while (( $(date +%s) - start_epoch < TIMEOUT_SEC )); do
  code="$(timeout 3 rostopic echo -n 1 /move_base/status 2>/dev/null | awk '/status:/{value=$2} END{if (value != "") print value; else print "0"}')"
  case "${code}" in
    3) success=true; status="SUCCEEDED"; break ;;
    4) status="ABORTED"; break ;;
    5) status="REJECTED"; break ;;
    9) status="LOST"; break ;;
    *) status="ACTIVE_OR_PENDING" ;;
  esac
  sleep 1
done
duration="$(( $(date +%s) - start_epoch ))"

kill "${bag_pid}" >/dev/null 2>&1 || true
wait "${bag_pid}" >/dev/null 2>&1 || true
sleep 1

rosrun nav_bringup analyze_cmd_vel_stage10.py --bag "${BAG}" --topic /cmd_vel --output "${ANALYSIS_CSV}"

row="$(awk -F, 'NR==2{print}' "${ANALYSIS_CSV}")"
max_vx="$(printf "%s\n" "${row}" | awk -F, '{print $5}')"
max_vy="$(printf "%s\n" "${row}" | awk -F, '{print $6}')"
max_wz="$(printf "%s\n" "${row}" | awk -F, '{print $7}')"
mean_wz="$(printf "%s\n" "${row}" | awk -F, '{print $10}')"
score="$(printf "%s\n" "${row}" | awk -F, '{print $11}')"

{
  echo "success,duration_sec,max_abs_vx,max_abs_vy,max_abs_wz,mean_abs_wz,yaw_overrotation_score,notes"
  echo "${success},${duration},${max_vx},${max_vy},${max_wz},${mean_wz},${score},${status}; bag=${BAG}"
} >"${SUMMARY_CSV}"

echo "Bag: ${BAG}"
echo "Analysis CSV: ${ANALYSIS_CSV}"
echo "Summary CSV: ${SUMMARY_CSV}"

[[ "${success}" == "true" ]]
