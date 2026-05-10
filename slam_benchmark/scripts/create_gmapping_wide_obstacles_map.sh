#!/usr/bin/env bash

set -euo pipefail

PKG_PATH="$(rospack find slam_benchmark)"
WS_ROOT="$(cd "${PKG_PATH}/../../.." && pwd)"
LOG_DIR="${PKG_PATH}/results/stage9/logs"
MAP_DIR="${PKG_PATH}/results/stage9/maps"
FINAL_PREFIX="${MAP_DIR}/gmapping_lidar_wide_obstacles_final"
LOG_FILE="${LOG_DIR}/create_gmapping_wide_obstacles_map.log"
SCENARIO="${SCENARIO:-wide_obstacles}"
ALGORITHM="${ALGORITHM:-gmapping_lidar}"
REP="${REP:-final}"
DURATION="${DURATION:-600}"
PATTERN="${PATTERN:-wide_obstacles_safe}"
LAUNCH_SIM="${LAUNCH_SIM:-auto}"
CMD_TOPIC="${CMD_TOPIC:-auto}"

mkdir -p "${LOG_DIR}" "${MAP_DIR}"
: > "${LOG_FILE}"
exec > >(tee -a "${LOG_FILE}") 2>&1

LAUNCH_PID=""

timestamp() { date -Is; }
log() { echo "[$(timestamp)] $*"; }

publish_zero() {
  for topic in /cmd_vel_raw /cmd_vel; do
    rostopic pub -1 "${topic}" geometry_msgs/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
      >/dev/null 2>&1 || true
  done
}

cleanup() {
  publish_zero
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    log "stopping launched wide-obstacles stack pid=${LAUNCH_PID}"
    kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 2
    kill -TERM "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

topic_exists() {
  rostopic list 2>/dev/null | grep -Fxq "$1"
}

topics_ready() {
  topic_exists /lidar/scan && topic_exists /odom && topic_exists /map && topic_exists /gazebo/model_states
}

select_cmd_topic() {
  if [[ "${CMD_TOPIC}" != "auto" ]]; then
    echo "${CMD_TOPIC}"
  elif rosnode list 2>/dev/null | grep -Fxq /cmd_vel_forward_only_filter; then
    echo /cmd_vel_raw
  else
    echo /cmd_vel
  fi
}

log "Stage 9 final Gmapping LiDAR wide-obstacles map creation"
log "final_prefix=${FINAL_PREFIX}"
log "duration=${DURATION} pattern=${PATTERN} launch_sim=${LAUNCH_SIM}"

if [[ "${LAUNCH_SIM}" == "true" ]] || { [[ "${LAUNCH_SIM}" == "auto" ]] && ! topics_ready; }; then
  log "starting slam_lidar_wide_obstacles_full.launch headless"
  roslaunch slam_benchmark slam_lidar_wide_obstacles_full.launch \
    use_rviz:=false gazebo_gui:=false \
    >"${LOG_DIR}/create_gmapping_wide_obstacles_launch.log" 2>&1 &
  LAUNCH_PID=$!
  log "launch_pid=${LAUNCH_PID}"
else
  log "using already running ROS/Gazebo/Gmapping system"
fi

log "waiting for required topics: /lidar/scan /odom /map /gazebo/model_states"
timeout 120 bash -c 'until rostopic list 2>/dev/null | grep -Fxq /lidar/scan && rostopic list | grep -Fxq /odom && rostopic list | grep -Fxq /map && rostopic list | grep -Fxq /gazebo/model_states; do sleep 2; done'

for topic in /lidar/scan /odom /map /gazebo/model_states; do
  log "checking first message on ${topic}"
  timeout 15 rostopic echo -n 1 "${topic}" >/tmp/create_wide_obstacles_topic_check.txt 2>&1
  log "topic ${topic}=PASS"
done

DRIVER_CMD_TOPIC="$(select_cmd_topic)"
log "running safe_mapping_driver scenario=${SCENARIO} rep=${REP} cmd_topic=${DRIVER_CMD_TOPIC}"
set +e
DURATION="${DURATION}" PATTERN="${PATTERN}" CMD_TOPIC="${DRIVER_CMD_TOPIC}" \
  "${PKG_PATH}/scripts/run_safe_mapping_driver.sh" "${SCENARIO}" "${ALGORITHM}" "${REP}"
DRIVER_STATUS=$?
set -e

STATUS_FILE="${PKG_PATH}/results/stage9/raw/${SCENARIO}/${ALGORITHM}/rep_${REP}/safe_driver_status.txt"
if [[ -s "${STATUS_FILE}" ]]; then
  STOP_REASON="$(awk -F= '$1=="stop_reason"{print $2}' "${STATUS_FILE}" | tail -n1)"
  [[ -n "${STOP_REASON}" ]] || STOP_REASON="$(awk -F= '$1=="reason"{print $2}' "${STATUS_FILE}" | tail -n1)"
  DRIVER_ELAPSED="$(awk -F= '$1=="elapsed"{print $2}' "${STATUS_FILE}" | tail -n1)"
  EXPLORED_RATIO="$(awk -F= '$1=="explored_ratio"{print $2}' "${STATUS_FILE}" | tail -n1)"
  UNKNOWN_RATIO="$(awk -F= '$1=="unknown_ratio"{print $2}' "${STATUS_FILE}" | tail -n1)"
  log "safe_driver_exit=${DRIVER_STATUS}"
  log "stop_reason=${STOP_REASON:-unknown}"
  log "elapsed=${DRIVER_ELAPSED:-unknown}"
  log "explored_ratio=${EXPLORED_RATIO:-unknown}"
  log "unknown_ratio=${UNKNOWN_RATIO:-unknown}"
else
  log "WARN: missing safe driver status at ${STATUS_FILE}"
  log "safe_driver_exit=${DRIVER_STATUS}"
fi

log "publishing zero velocity"
publish_zero
sleep 3

rm -f "${FINAL_PREFIX}.yaml" "${FINAL_PREFIX}.pgm"
log "saving final map to ${FINAL_PREFIX}.yaml/.pgm"
rosrun map_server map_saver -f "${FINAL_PREFIX}" map:=/map

log "evaluating final map"
rosrun slam_benchmark evaluate_map_basic.py \
  "${FINAL_PREFIX}.yaml" -o "${MAP_DIR}/gmapping_lidar_wide_obstacles_final_metrics.csv"

NAV_MAP_DIR="${WS_ROOT}/src/omni-robot/nav_bringup/maps"
if [[ -d "${NAV_MAP_DIR}" ]]; then
  log "copying final map to nav_bringup"
  cp -f "${FINAL_PREFIX}.pgm" "${NAV_MAP_DIR}/lidar_baseline_wide_obstacles.pgm"
  {
    echo "image: lidar_baseline_wide_obstacles.pgm"
    grep -v '^image:' "${FINAL_PREFIX}.yaml"
  } > "${NAV_MAP_DIR}/lidar_baseline_wide_obstacles.yaml"
  log "nav_map_yaml=${NAV_MAP_DIR}/lidar_baseline_wide_obstacles.yaml"
  log "nav_map_pgm=${NAV_MAP_DIR}/lidar_baseline_wide_obstacles.pgm"
else
  log "WARN: nav_bringup/maps not found at ${NAV_MAP_DIR}"
fi

log "PASS: final_map=${FINAL_PREFIX}.yaml"
log "PASS: metrics=${MAP_DIR}/gmapping_lidar_wide_obstacles_final_metrics.csv"
log "PASS: log=${LOG_FILE}"

if [[ "${DRIVER_STATUS}" -ne 0 ]]; then
  log "FAIL: safe_mapping_driver exited with ${DRIVER_STATUS}"
  exit "${DRIVER_STATUS}"
fi
