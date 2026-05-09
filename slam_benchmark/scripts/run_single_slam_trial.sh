#!/usr/bin/env bash

set -euo pipefail

SCENARIO="${1:-}"
ALGORITHM="${2:-}"
REPETITION="${3:-}"

if [[ -z "${SCENARIO}" || -z "${ALGORITHM}" || -z "${REPETITION}" ]]; then
  echo "Usage: run_single_slam_trial.sh <scenario_name> <algorithm_name> <repetition_id>"
  exit 2
fi

PKG_PATH="$(rospack find slam_benchmark)"

if [[ "${ALGORITHM}" == "gmapping_lidar" || "${ALGORITHM}" == "hector_lidar" ]]; then
  echo "Stage 9 LiDAR algorithm requested; delegating to safe Stage 9 benchmark runner."
  exec "${PKG_PATH}/scripts/run_stage9_lidar_benchmark.sh" --single "${SCENARIO}" "${ALGORITHM}" "${REPETITION}"
fi

OUT_DIR="${PKG_PATH}/results/stage8/raw/${SCENARIO}/${ALGORITHM}/rep_${REPETITION}"
LOG_FILE="${OUT_DIR}/trial.log"
STATUS_CSV="${OUT_DIR}/runtime_status.csv"
mkdir -p "${OUT_DIR}"

exec > >(tee -a "${LOG_FILE}") 2>&1

SIM_PID=""
SLAM_PID=""
BAG_PID=""
DRIVE_PID=""
START_EPOCH="$(date +%s)"

cleanup() {
  local status=$?
  echo "cleanup: stopping drive/record/launch processes"
  rostopic pub /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1 >/dev/null 2>&1 || true
  rosnode kill /stage8_rosbag_recorder /slam_gmapping /hector_mapping /gazebo /spawn_omni_robot \
    /gazebo_model_cmd_vel /gazebo_truth_odom /pointcloud_to_laserscan /stereo/stereo_image_proc_manager \
    >/dev/null 2>&1 || true
  for pid in "${DRIVE_PID}" "${BAG_PID}" "${SLAM_PID}" "${SIM_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
      kill -INT "${pid}" >/dev/null 2>&1 || true
      sleep 2
      kill -TERM "${pid}" >/dev/null 2>&1 || true
    fi
  done
  pkill -TERM -P "$$" >/dev/null 2>&1 || true
  wait || true
  exit "${status}"
}
trap cleanup INT TERM

scenario_pattern() {
  case "$1" in
    corridor_static) echo "corridor" ;;
    open_room_obstacles) echo "rectangle" ;;
    narrow_turn) echo "narrow_turn" ;;
    *) echo "corridor" ;;
  esac
}

algorithm_launch() {
  case "$1" in
    gmapping_conservative) echo "gmapping_conservative.launch /map map base_footprint" ;;
    gmapping_fast) echo "gmapping_fast.launch /map map base_footprint" ;;
    hector) echo "hector_sim.launch /map_hector hector_map base_footprint" ;;
    *) return 1 ;;
  esac
}

if ! algorithm_launch "${ALGORITHM}" >/dev/null; then
  echo "FAIL: unknown algorithm ${ALGORITHM}"
  exit 2
fi

read -r SLAM_LAUNCH MAP_TOPIC PARENT_FRAME CHILD_FRAME < <(algorithm_launch "${ALGORITHM}")
PATTERN="$(scenario_pattern "${SCENARIO}")"
BAG_PATH="${OUT_DIR}/${SCENARIO}_${ALGORITHM}_rep_${REPETITION}.bag"
MAP_PREFIX="${OUT_DIR}/${ALGORITHM}_${SCENARIO}_rep_${REPETITION}"

echo "Stage 8 trial"
echo "scenario=${SCENARIO} algorithm=${ALGORITHM} repetition=${REPETITION}"
echo "output=${OUT_DIR}"
echo "drive_pattern=${PATTERN} map_topic=${MAP_TOPIC}"

if rostopic list >/dev/null 2>&1; then
  echo "WARN: an existing ROS master is reachable. This script will use it and still start its own launch files."
fi

LAUNCH_CMD=(roslaunch slam_benchmark slam_sim_full.launch use_rviz:=false gazebo_gui:=false use_gmapping:=false use_sim_time:=true)
if [[ -z "${DISPLAY:-}" ]] && command -v xvfb-run >/dev/null 2>&1; then
  LAUNCH_CMD=(xvfb-run -a "${LAUNCH_CMD[@]}")
fi

echo "command: ${LAUNCH_CMD[*]}"
"${LAUNCH_CMD[@]}" &
SIM_PID=$!

echo "waiting for /scan, /odom, /gazebo/model_states"
timeout 75 bash -c 'until rostopic list 2>/dev/null | grep -Fxq /scan && rostopic list | grep -Fxq /odom && rostopic list | grep -Fxq /gazebo/model_states; do sleep 2; done' || {
  echo "FAIL: required simulation topics did not appear"
  echo "success,false" > "${STATUS_CSV}"
  exit 1
}

echo "command: roslaunch slam_benchmark ${SLAM_LAUNCH} use_sim_time:=true"
roslaunch slam_benchmark "${SLAM_LAUNCH}" use_sim_time:=true &
SLAM_PID=$!

echo "waiting for ${MAP_TOPIC}"
timeout 55 bash -c "until rostopic list 2>/dev/null | grep -Fxq '${MAP_TOPIC}'; do sleep 2; done" || {
  echo "WARN: ${MAP_TOPIC} not visible yet; continuing so bag captures failure state"
}

TOPICS=(/scan /odom /tf /tf_static /clock /cmd_vel /gazebo/model_states "${MAP_TOPIC}")
echo "command: rosbag record __name:=stage8_rosbag_recorder -O ${BAG_PATH} ${TOPICS[*]}"
rosbag record __name:=stage8_rosbag_recorder -O "${BAG_PATH}" "${TOPICS[@]}" </dev/null &
BAG_PID=$!
sleep 4

echo "command: ${PKG_PATH}/scripts/drive_mapping_pattern.sh ${PATTERN}"
"${PKG_PATH}/scripts/drive_mapping_pattern.sh" "${PATTERN}" &
DRIVE_PID=$!
wait "${DRIVE_PID}" || true
DRIVE_PID=""

echo "allowing SLAM to settle"
sleep 8

if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" >/dev/null 2>&1; then
  kill -INT "${BAG_PID}" || true
  wait "${BAG_PID}" || true
  BAG_PID=""
fi

MAP_STATUS="FAIL"
if timeout 12 rostopic echo -n 1 "${MAP_TOPIC}" >/dev/null 2>&1; then
  echo "command: rosrun map_server map_saver map:=${MAP_TOPIC} -f ${MAP_PREFIX}"
  if rosrun map_server map_saver map:="${MAP_TOPIC}" -f "${MAP_PREFIX}"; then
    MAP_STATUS="PASS"
  fi
else
  echo "WARN: no map message received on ${MAP_TOPIC}; map not saved"
fi

END_EPOCH="$(date +%s)"
RUNTIME="$((END_EPOCH - START_EPOCH))"
BAG_STATUS="FAIL"
if [[ -s "${BAG_PATH}" || -s "${BAG_PATH}.active" ]]; then
  BAG_STATUS="PASS"
fi

{
  echo "scenario,algorithm,repetition,success,runtime_sec,bag_path,map_yaml,map_topic,parent_frame,child_frame,notes"
  echo "${SCENARIO},${ALGORITHM},${REPETITION},$([[ "${BAG_STATUS}" == "PASS" && "${MAP_STATUS}" == "PASS" ]] && echo true || echo false),${RUNTIME},${BAG_PATH},${MAP_PREFIX}.yaml,${MAP_TOPIC},${PARENT_FRAME},${CHILD_FRAME},bag=${BAG_STATUS};map=${MAP_STATUS}"
} > "${STATUS_CSV}"

echo "runtime_status=${STATUS_CSV}"
echo "bag=${BAG_PATH}"
echo "map=${MAP_PREFIX}.yaml"
echo "trial result: bag=${BAG_STATUS} map=${MAP_STATUS} runtime=${RUNTIME}s"

trap - INT TERM
cleanup
