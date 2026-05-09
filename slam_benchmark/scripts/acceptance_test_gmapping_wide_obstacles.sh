#!/usr/bin/env bash

set -euo pipefail

PKG_PATH="$(rospack find slam_benchmark)"
WS_ROOT="$(cd "${PKG_PATH}/../../.." && pwd)"
LOG_DIR="${PKG_PATH}/results/stage9/logs"
LOG_FILE="${LOG_DIR}/acceptance_test_gmapping_wide_obstacles.log"
MAP_DIR="${PKG_PATH}/results/stage9/maps"
NAV_MAP_DIR="${WS_ROOT}/src/omni-robot/nav_bringup/maps"
RUN_MAP_CREATION="${RUN_MAP_CREATION:-true}"
RUN_BENCHMARK_REP1="${RUN_BENCHMARK_REP1:-true}"

mkdir -p "${LOG_DIR}"
: > "${LOG_FILE}"
exec > >(tee -a "${LOG_FILE}") 2>&1

timestamp() { date -Is; }
log() { echo "[$(timestamp)] $*"; }

PASS_KEYS=()
FAIL_KEYS=()

record_check() {
  local key="$1" status="$2" notes="${3:-}"
  if [[ "${status}" == "PASS" ]]; then
    PASS_KEYS+=("${key}")
  else
    FAIL_KEYS+=("${key}")
  fi
  log "${key}=${status}${notes:+ ${notes}}"
}

topic_message_check() {
  local topic="$1" timeout_sec="$2"
  timeout "${timeout_sec}" rostopic echo -n 1 "${topic}" >/tmp/wide_obstacles_acceptance_topic.txt 2>&1
}

log "building workspace"
cd "${WS_ROOT}"
catkin_make
source devel/setup.bash

log "verifying world"
if rosrun slam_benchmark verify_wide_obstacles_world.py; then
  record_check world_has_6_or_7_boxes PASS
else
  record_check world_has_6_or_7_boxes FAIL
fi

log "checking launch file resolution"
if roslaunch --files robot_description gazebo_lidar_wide_obstacles.launch >/tmp/wide_obstacles_launch_files.txt 2>&1; then
  record_check robot_world_launch_exists PASS
else
  record_check robot_world_launch_exists FAIL
fi
if roslaunch --files slam_benchmark slam_lidar_wide_obstacles_full.launch >/tmp/wide_obstacles_full_launch_files.txt 2>&1; then
  record_check slam_full_launch_exists PASS
else
  record_check slam_full_launch_exists FAIL
fi

if [[ "${RUN_MAP_CREATION}" == "true" ]]; then
  log "creating final map with create_gmapping_wide_obstacles_map.sh"
  LAUNCH_SIM="${LAUNCH_SIM:-auto}" "${PKG_PATH}/scripts/create_gmapping_wide_obstacles_map.sh"
else
  log "RUN_MAP_CREATION=false, expecting user to have launched and created the map"
fi

if topic_message_check /lidar/scan 10; then record_check lidar_scan_ok PASS; else record_check lidar_scan_ok FAIL; fi
if topic_message_check /map 15; then record_check gmapping_map_ok PASS; else record_check gmapping_map_ok FAIL; fi

if [[ -s "${MAP_DIR}/gmapping_lidar_wide_obstacles_final.yaml" && -s "${MAP_DIR}/gmapping_lidar_wide_obstacles_final.pgm" ]]; then
  record_check map_saved_ok PASS
else
  record_check map_saved_ok FAIL
fi

if [[ -s "${NAV_MAP_DIR}/lidar_baseline_wide_obstacles.yaml" && -s "${NAV_MAP_DIR}/lidar_baseline_wide_obstacles.pgm" ]]; then
  record_check nav_map_copied_ok PASS
else
  record_check nav_map_copied_ok FAIL
fi

if [[ "${RUN_BENCHMARK_REP1}" == "true" ]]; then
  log "running one wide_obstacles benchmark trial"
  cd "${PKG_PATH}"
  ./scripts/run_stage9_lidar_benchmark.sh --launch-system --single wide_obstacles gmapping_lidar 1 || true
fi

REP1_DIR="${PKG_PATH}/results/stage9/raw/wide_obstacles/gmapping_lidar/rep_1"
if [[ -s "${REP1_DIR}/trial.bag" && -s "${REP1_DIR}/map.yaml" && -s "${REP1_DIR}/trajectory_metrics.csv" && -s "${REP1_DIR}/map_metrics.csv" ]]; then
  record_check benchmark_rep1_ok PASS
else
  record_check benchmark_rep1_ok FAIL
fi

log "aggregating Stage 9 results"
rosrun slam_benchmark aggregate_stage9_results.py || true

echo
echo "Acceptance summary"
for key in "${PASS_KEYS[@]}"; do
  echo "PASS: ${key}"
done
for key in "${FAIL_KEYS[@]}"; do
  echo "FAIL: ${key}"
done

if [[ "${#FAIL_KEYS[@]}" -eq 0 ]]; then
  echo "PASS: acceptance_test_gmapping_wide_obstacles"
  exit 0
fi

echo "FAIL: acceptance_test_gmapping_wide_obstacles"
exit 1
