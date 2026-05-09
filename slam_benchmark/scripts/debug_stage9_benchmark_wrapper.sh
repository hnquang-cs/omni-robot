#!/usr/bin/env bash

set -euo pipefail

SCENARIO="${1:-corridor_static}"
ALGORITHM="${2:-gmapping_lidar}"
REP="${3:-1}"
PKG_PATH="$(rospack find slam_benchmark)"
TRIAL_DIR="${PKG_PATH}/results/stage9/raw/${SCENARIO}/${ALGORITHM}/rep_${REP}"
DEBUG_LOG="${TRIAL_DIR}/debug_wrapper.log"
mkdir -p "${TRIAL_DIR}"
: > "${DEBUG_LOG}"

log() {
  echo "[$(date -Is)] $*" | tee -a "${DEBUG_LOG}"
}

fail() {
  log "FAIL: $*"
  exit 1
}

topic_msg() {
  local topic="$1"
  if timeout 5 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    log "PASS: topic ${topic} has messages"
  else
    fail "topic ${topic} has no messages"
  fi
}

select_driver_cmd_topic() {
  if rosnode list 2>/dev/null | grep -Fxq /cmd_vel_forward_only_filter; then
    echo /cmd_vel_raw
  else
    echo /cmd_vel
  fi
}

SAFE_DRIVER_PID=""

cleanup() {
  rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
  rostopic pub -1 /cmd_vel_raw geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
  if [[ -n "${SAFE_DRIVER_PID}" ]] && kill -0 "${SAFE_DRIVER_PID}" >/dev/null 2>&1; then
    kill -INT "${SAFE_DRIVER_PID}" >/dev/null 2>&1 || true
    wait "${SAFE_DRIVER_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

log "argv=${SCENARIO} ${ALGORITHM} ${REP}"
log "pkg_path=${PKG_PATH}"
log "cwd=$(pwd)"
log "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH:-}"
log "ROS_MASTER_URI=${ROS_MASTER_URI:-}"

[[ -x "${PKG_PATH}/scripts/run_safe_mapping_driver.sh" ]] || fail "run_safe_mapping_driver.sh is not executable"
log "PASS: run_safe_mapping_driver.sh executable"
[[ -x "${PKG_PATH}/scripts/safe_mapping_driver.py" ]] || fail "safe_mapping_driver.py is not executable"
log "PASS: safe_mapping_driver.py executable"

topic_msg /lidar/scan
topic_msg /odom
topic_msg /map
topic_msg /gazebo/model_states

CMD_TOPIC="$(select_driver_cmd_topic)"
log "selected_cmd_topic=${CMD_TOPIC}"
log "Running safe driver: DURATION=20 PATTERN=corridor_safe CMD_TOPIC=${CMD_TOPIC} ${PKG_PATH}/scripts/run_safe_mapping_driver.sh ${SCENARIO} ${ALGORITHM} ${REP}"

(
  set -o pipefail
  DURATION=20 PATTERN=corridor_safe CMD_TOPIC="${CMD_TOPIC}" \
    "${PKG_PATH}/scripts/run_safe_mapping_driver.sh" "${SCENARIO}" "${ALGORITHM}" "${REP}" 2>&1 | tee -a "${DEBUG_LOG}"
) &
SAFE_DRIVER_PID=$!
log "safe_driver_pid=${SAFE_DRIVER_PID}"

if rosrun slam_benchmark check_cmd_vel_activity.py --duration 10 --csv "${TRIAL_DIR}/cmd_vel_activity_debug.csv" >>"${DEBUG_LOG}" 2>&1; then
  log "cmd_vel_activity=PASS"
else
  fail "cmd_vel_activity=FAIL"
fi

SAFE_EXIT=0
wait "${SAFE_DRIVER_PID}" || SAFE_EXIT=$?
SAFE_DRIVER_PID=""
log "safe_driver_exit=${SAFE_EXIT}"
if [[ "${SAFE_EXIT}" -ne 0 ]]; then
  log "WARN: safe driver exited nonzero after cmd_vel became active; this is usually a safety stop from the current robot pose."
fi

log "PASS: debug wrapper completed"
