#!/usr/bin/env bash

set -euo pipefail

PKG_PATH="$(rospack find slam_benchmark)"
WS_ROOT="$(cd "${PKG_PATH}/../../.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${PKG_PATH}/results/stage9/logs"
MAP_DIR="${PKG_PATH}/results/stage9/maps"
LOG_FILE="${LOG_DIR}/create_wide_safe_map_${STAMP}.log"
MAP_PREFIX="${MAP_DIR}/lidar_wide_safe_${STAMP}"
SCENARIO="${SCENARIO:-wide_safe_map}"
ALGORITHM="${ALGORITHM:-gmapping_lidar}"
REP="${REP:-${STAMP}}"
DURATION="${DURATION:-120}"
LAUNCH_SIM="${LAUNCH_SIM:-true}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel_raw}"

mkdir -p "${LOG_DIR}" "${MAP_DIR}"
exec > >(tee "${LOG_FILE}") 2>&1

LAUNCH_PID=""

cleanup() {
  rostopic pub -1 /cmd_vel_raw geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
  rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 2
    kill -TERM "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "Stage 9 wide safe map creation"
echo "timestamp=${STAMP}"
echo "map_prefix=${MAP_PREFIX}"

if [[ "${LAUNCH_SIM}" == "true" ]]; then
  roslaunch slam_benchmark slam_lidar_wide_safe_full.launch use_rviz:=false gazebo_gui:=false >"${LOG_DIR}/create_wide_safe_map_launch_${STAMP}.log" 2>&1 &
  LAUNCH_PID=$!
fi

echo "waiting for /lidar/scan /odom /map"
timeout 90 bash -c 'until rostopic list 2>/dev/null | grep -Fxq /lidar/scan && rostopic list | grep -Fxq /odom && rostopic list | grep -Fxq /map; do sleep 2; done'

echo "running safe mapping driver for ${DURATION}s"
DURATION="${DURATION}" PATTERN="${PATTERN:-explore_safe}" CMD_TOPIC="${CMD_TOPIC}" \
  "${PKG_PATH}/scripts/run_safe_mapping_driver.sh" "${SCENARIO}" "${ALGORITHM}" "${REP}"

echo "saving map to ${MAP_PREFIX}"
rosrun map_server map_saver -f "${MAP_PREFIX}" map:=/map
rosrun slam_benchmark evaluate_map_basic.py "${MAP_PREFIX}.yaml" -o "${MAP_PREFIX}_metrics.csv"

NAV_MAP_DIR="${WS_ROOT}/src/omni-robot/nav_bringup/maps"
if [[ -d "${NAV_MAP_DIR}" ]]; then
  cp -f "${MAP_PREFIX}.yaml" "${NAV_MAP_DIR}/lidar_baseline_wide_safe_${STAMP}.yaml"
  cp -f "${MAP_PREFIX}.pgm" "${NAV_MAP_DIR}/lidar_baseline_wide_safe_${STAMP}.pgm"
  pushd "${NAV_MAP_DIR}" >/dev/null
  ln -sfn "lidar_baseline_wide_safe_${STAMP}.yaml" lidar_baseline_wide_safe.yaml
  ln -sfn "lidar_baseline_wide_safe_${STAMP}.pgm" lidar_baseline_wide_safe.pgm
  popd >/dev/null
  echo "nav_bringup map symlinks updated in ${NAV_MAP_DIR}"
fi

echo "PASS: map=${MAP_PREFIX}.yaml"
echo "PASS: metrics=${MAP_PREFIX}_metrics.csv"
echo "PASS: log=${LOG_FILE}"
