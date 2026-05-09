#!/usr/bin/env bash

set -euo pipefail

SCENARIO="${1:-corridor_static}"
ALGORITHM="${2:-gmapping_lidar}"
REP="${3:-1}"

default_duration() {
  case "$1" in
    wide_obstacles) echo 150 ;;
    *) echo 90 ;;
  esac
}

default_pattern() {
  case "$1" in
    wide_obstacles) echo wide_obstacles_safe ;;
    *) echo corridor_safe ;;
  esac
}

default_max_rotate_attempts() {
  case "$1" in
    wide_obstacles) echo 12 ;;
    *) echo 6 ;;
  esac
}

default_blocked_timeout() {
  case "$1" in
    wide_obstacles) echo 45.0 ;;
    *) echo 30.0 ;;
  esac
}

DURATION="${DURATION:-$(default_duration "${SCENARIO}")}"
PATTERN="${PATTERN:-$(default_pattern "${SCENARIO}")}"
FORWARD_SPEED="${FORWARD_SPEED:-0.08}"
ROTATE_SPEED="${ROTATE_SPEED:-0.15}"
SAFETY_STOP_DISTANCE="${SAFETY_STOP_DISTANCE:-0.80}"
CRITICAL_STOP_DISTANCE="${CRITICAL_STOP_DISTANCE:-0.35}"
FRONT_ANGLE_DEG="${FRONT_ANGLE_DEG:-25}"
MAX_ROTATE_ATTEMPTS="${MAX_ROTATE_ATTEMPTS:-$(default_max_rotate_attempts "${SCENARIO}")}"
MAX_ROTATE_SEGMENT="${MAX_ROTATE_SEGMENT:-6.0}"
BLOCKED_TIMEOUT="${BLOCKED_TIMEOUT:-$(default_blocked_timeout "${SCENARIO}")}"
USE_DISCRETE_45="${USE_DISCRETE_45:-false}"
PRIMITIVE_TOPIC="${PRIMITIVE_TOPIC:-/motion_primitive_cmd}"
PRIMITIVE_STATE_TOPIC="${PRIMITIVE_STATE_TOPIC:-/motion_primitive_state}"
if [[ -n "${CMD_TOPIC:-}" ]]; then
  CMD_TOPIC="${CMD_TOPIC}"
elif rostopic list 2>/dev/null | grep -Fxq /cmd_vel_raw; then
  CMD_TOPIC="/cmd_vel_raw"
else
  CMD_TOPIC="/cmd_vel"
fi

PKG_PATH="$(rospack find slam_benchmark)"

EXTRA_ARGS=()
if [[ "${USE_DISCRETE_45}" == "true" ]]; then
  EXTRA_ARGS+=(--use-discrete-45 --primitive-topic "${PRIMITIVE_TOPIC}" --primitive-state-topic "${PRIMITIVE_STATE_TOPIC}")
fi

exec "${PKG_PATH}/scripts/safe_mapping_driver.py" \
  --scenario "${SCENARIO}" \
  --algorithm "${ALGORITHM}" \
  --rep "${REP}" \
  --duration "${DURATION}" \
  --pattern "${PATTERN}" \
  --front-angle-deg "${FRONT_ANGLE_DEG}" \
  --forward-speed "${FORWARD_SPEED}" \
  --rotate-speed "${ROTATE_SPEED}" \
  --safety-stop-distance "${SAFETY_STOP_DISTANCE}" \
  --critical-stop-distance "${CRITICAL_STOP_DISTANCE}" \
  --cmd-topic "${CMD_TOPIC}" \
  --max-rotate-attempts "${MAX_ROTATE_ATTEMPTS}" \
  --max-rotate-segment "${MAX_ROTATE_SEGMENT}" \
  --blocked-timeout "${BLOCKED_TIMEOUT}" \
  "${EXTRA_ARGS[@]}"
