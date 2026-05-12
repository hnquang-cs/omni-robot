#!/usr/bin/env bash
# Stage 11 runtime check for an active frontier exploration session.
# Run while frontier_exploration_full.launch is up.

set -uo pipefail

PKG_PATH="$(rospack find slam_benchmark 2>/dev/null || true)"
if [[ -z "${PKG_PATH}" ]]; then
  PKG_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi
LOG_DIR="${PKG_PATH}/results/stage11_frontier/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/frontier_runtime_check.log"

PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0

# Check that a topic publishes at least one message within timeout seconds.
check_topic_msg() {
  local topic="$1"
  local timeout_sec="${2:-5}"
  if ! rostopic list 2>/dev/null | grep -Fxq "${topic}"; then
    echo "FAIL  topic ${topic} not advertised"
    FAIL_COUNT=$((FAIL_COUNT + 1))
    return 1
  fi
  if timeout "${timeout_sec}" rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "PASS  topic ${topic} has messages"
    PASS_COUNT=$((PASS_COUNT + 1))
    return 0
  else
    echo "FAIL  topic ${topic} advertised but silent (>${timeout_sec}s)"
    FAIL_COUNT=$((FAIL_COUNT + 1))
    return 1
  fi
}

check_topic_optional() {
  local topic="$1"
  local timeout_sec="${2:-3}"
  if ! rostopic list 2>/dev/null | grep -Fxq "${topic}"; then
    echo "WARN  optional topic ${topic} not advertised"
    WARN_COUNT=$((WARN_COUNT + 1))
    return 1
  fi
  if timeout "${timeout_sec}" rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "PASS  optional topic ${topic} has messages"
    PASS_COUNT=$((PASS_COUNT + 1))
    return 0
  else
    echo "WARN  optional topic ${topic} silent"
    WARN_COUNT=$((WARN_COUNT + 1))
    return 1
  fi
}

check_tf() {
  local from="$1"
  local to="$2"
  if timeout 5 rosrun tf tf_echo "${from}" "${to}" >/dev/null 2>&1; then
    echo "PASS  TF ${from} -> ${to}"
    PASS_COUNT=$((PASS_COUNT + 1))
  else
    echo "FAIL  TF ${from} -> ${to} not available"
    FAIL_COUNT=$((FAIL_COUNT + 1))
  fi
}

check_node_present() {
  local node="$1"
  if rosnode list 2>/dev/null | grep -Fxq "${node}"; then
    echo "PASS  node ${node} running"
    PASS_COUNT=$((PASS_COUNT + 1))
    return 0
  else
    echo "FAIL  node ${node} NOT running"
    FAIL_COUNT=$((FAIL_COUNT + 1))
    return 1
  fi
}

check_node_absent() {
  local pattern="$1"
  local label="$2"
  if rosnode list 2>/dev/null | grep -E "${pattern}" >/dev/null 2>&1; then
    echo "FAIL  ${label} unexpectedly running"
    FAIL_COUNT=$((FAIL_COUNT + 1))
  else
    echo "PASS  ${label} not running (as required)"
    PASS_COUNT=$((PASS_COUNT + 1))
  fi
}

{
  echo "=== Stage 11 Frontier Runtime Check ==="
  echo "Timestamp: $(date -Iseconds)"
  echo

  if ! rostopic list >/dev/null 2>&1; then
    echo "FAIL  ROS master is not reachable. Aborting."
    exit 2
  fi

  echo "-- Required topics --"
  check_topic_msg /map 10
  check_topic_msg /lidar/scan 5
  check_topic_msg /odom 5
  check_topic_msg /move_base/status 10

  echo
  echo "-- Optional topics --"
  check_topic_optional /move_base/goal 5
  check_topic_optional /move_base_simple/goal 3
  check_topic_optional /cmd_vel 5
  check_topic_optional /cmd_vel_raw 5
  check_topic_optional /explore/frontiers 5
  check_topic_optional /move_base/GlobalPlanner/plan 5
  check_topic_optional /move_base/TebLocalPlannerROS/local_plan 5

  echo
  echo "-- TF tree --"
  check_tf map odom
  # tf_echo waits forever if a frame is missing — every probe MUST go through
  # `timeout`. Probe odom -> base_link first; if it has no TF, fall back to
  # base_footprint.
  if timeout 3 rosrun tf tf_echo odom base_link >/dev/null 2>&1; then
    check_tf odom base_link
    BASE_FRAME=base_link
  else
    check_tf odom base_footprint
    BASE_FRAME=base_footprint
  fi
  if timeout 3 rosrun tf tf_echo "${BASE_FRAME}" lidar_link >/dev/null 2>&1; then
    check_tf "${BASE_FRAME}" lidar_link
  else
    check_tf base_link lidar_link
  fi

  echo
  echo "-- Required nodes --"
  check_node_present /move_base
  if ! check_node_present /explore; then
    echo "INFO  explore node missing. Either explore_lite is not installed,"
    echo "      or the fallback simple_frontier_detector.py is being used."
  fi

  echo
  echo "-- Forbidden / conflicting nodes --"
  check_node_absent '/amcl(_|$)' "AMCL"
  check_node_absent 'safe_mapping_driver' "safe_mapping_driver"
  check_node_absent '/map_server' "map_server"

  echo
  echo "-- Summary --"
  echo "PASS:  ${PASS_COUNT}"
  echo "WARN:  ${WARN_COUNT}"
  echo "FAIL:  ${FAIL_COUNT}"

  if [[ "${FAIL_COUNT}" -gt 0 ]]; then
    echo "OVERALL: FAIL"
  else
    echo "OVERALL: PASS"
  fi
} 2>&1 | tee "${LOG_FILE}"

if [[ "${FAIL_COUNT}" -gt 0 ]]; then
  exit 1
fi
exit 0
