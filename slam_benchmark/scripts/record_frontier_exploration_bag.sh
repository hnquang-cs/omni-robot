#!/usr/bin/env bash
# Stage 11: record an exploration bag with all topics needed for evaluation.
#
# Usage: record_frontier_exploration_bag.sh [duration_sec] [bag_basename]
#   duration_sec   defaults to 300
#   bag_basename   defaults to frontier_exploration_<timestamp>

set -uo pipefail

DURATION="${1:-300}"
PKG_PATH="$(rospack find slam_benchmark 2>/dev/null || true)"
if [[ -z "${PKG_PATH}" ]]; then
  PKG_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

BAG_DIR="${PKG_PATH}/results/stage11_frontier/bags"
LOG_DIR="${PKG_PATH}/results/stage11_frontier/logs"
mkdir -p "${BAG_DIR}" "${LOG_DIR}"

STAMP="$(date +%Y%m%d_%H%M%S)"
BASENAME="${2:-frontier_exploration_${STAMP}}"
BAG_PATH="${BAG_DIR}/${BASENAME}.bag"
INFO_PATH="${BAG_DIR}/${BASENAME}_info.txt"
LOG_PATH="${LOG_DIR}/${BASENAME}_record.log"

# Required topics (must exist).
REQUIRED_TOPICS=(
  /map
  /lidar/scan
  /odom
  /tf
  /tf_static
  /clock
  /cmd_vel
  /move_base/status
)

# Optional topics: only included if currently advertised.
OPTIONAL_TOPICS=(
  /cmd_vel_raw
  /gazebo/model_states
  /move_base/goal
  /move_base_simple/goal
  /move_base/result
  /move_base/GlobalPlanner/plan
  /move_base/TebLocalPlannerROS/local_plan
  /move_base/global_costmap/costmap
  /move_base/local_costmap/costmap
  /explore/frontiers
  /simple_frontiers
)

if ! rostopic list >/dev/null 2>&1; then
  echo "FAIL  ROS master not reachable" | tee "${LOG_PATH}"
  exit 1
fi

ADVERTISED="$(rostopic list 2>/dev/null || true)"
TOPICS_TO_RECORD=()
for t in "${REQUIRED_TOPICS[@]}"; do
  TOPICS_TO_RECORD+=("${t}")
done
for t in "${OPTIONAL_TOPICS[@]}"; do
  if grep -Fxq "${t}" <<<"${ADVERTISED}"; then
    TOPICS_TO_RECORD+=("${t}")
  else
    echo "INFO  optional topic ${t} not advertised, skipping" | tee -a "${LOG_PATH}"
  fi
done

{
  echo "=== Stage 11 Bag Record ==="
  echo "Timestamp:    $(date -Iseconds)"
  echo "Duration:     ${DURATION}s"
  echo "Bag path:     ${BAG_PATH}"
  echo "Topic count:  ${#TOPICS_TO_RECORD[@]}"
  printf '  %s\n' "${TOPICS_TO_RECORD[@]}"
} | tee -a "${LOG_PATH}"

set +e
timeout "${DURATION}" rosbag record -O "${BAG_PATH}" "${TOPICS_TO_RECORD[@]}" \
  >>"${LOG_PATH}" 2>&1
status=$?
set -e

# 124 = timeout reached normally; 0 = clean exit; treat both as success.
if [[ "${status}" != "0" && "${status}" != "124" ]]; then
  echo "FAIL  rosbag record exited with status ${status}" | tee -a "${LOG_PATH}"
  exit "${status}"
fi

if [[ ! -s "${BAG_PATH}" ]]; then
  echo "FAIL  bag was not created or is empty: ${BAG_PATH}" | tee -a "${LOG_PATH}"
  exit 1
fi

rosbag info "${BAG_PATH}" | tee "${INFO_PATH}"

FAILED=0
for topic in /lidar/scan /odom /map /tf; do
  if grep -qE "^[[:space:]]*${topic}[[:space:]]" "${INFO_PATH}"; then
    echo "PASS  ${topic} recorded" | tee -a "${LOG_PATH}"
  else
    echo "FAIL  ${topic} missing from bag" | tee -a "${LOG_PATH}"
    FAILED=1
  fi
done

echo "Bag:  ${BAG_PATH}"
echo "Info: ${INFO_PATH}"
echo "Log:  ${LOG_PATH}"
exit "${FAILED}"
