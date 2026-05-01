#!/usr/bin/env bash

set -u

SCAN_TOPIC="${SCAN_TOPIC:-/lidar/scan}"
LIDAR_FRAME="${LIDAR_FRAME:-lidar_link}"
BASE_FRAME="${BASE_FRAME:-base_link}"
SLAM_BASE_FRAME="${SLAM_BASE_FRAME:-base_footprint}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
TIMEOUT="${TIMEOUT:-8}"

pass() { echo "PASS: $*"; }
fail() { echo "FAIL: $*"; FAILED=1; }
check_msg() {
  local topic="$1"
  local label="$2"
  if timeout "${TIMEOUT}" rostopic echo -n 1 "${topic}" >/tmp/stage9_${label}.txt 2>&1; then
    pass "${topic} has messages"
    return 0
  fi
  fail "${topic} has no message within ${TIMEOUT}s"
  return 1
}
check_tf() {
  local parent="$1"
  local child="$2"
  local label="$3"
  local output="/tmp/stage9_${label}_tf.txt"
  timeout "${TIMEOUT}" rosrun tf tf_echo "${parent}" "${child}" >"${output}" 2>&1 || true
  if grep -q "Translation:" "${output}"; then
    pass "TF ${parent} -> ${child} exists"
  else
    fail "TF ${parent} -> ${child} missing"
  fi
}

FAILED=0

if rostopic list 2>/dev/null | grep -qx "${SCAN_TOPIC}"; then
  pass "${SCAN_TOPIC} exists"
else
  fail "${SCAN_TOPIC} is missing"
fi

if check_msg "${SCAN_TOPIC}" scan; then
  frame_id="$(awk -F': ' '/frame_id:/ {gsub(/"/, "", $2); print $2; exit}' /tmp/stage9_scan.txt)"
  if [[ "${frame_id}" == "${LIDAR_FRAME}" ]]; then
    pass "${SCAN_TOPIC} frame_id is ${LIDAR_FRAME}"
  else
    fail "${SCAN_TOPIC} frame_id expected ${LIDAR_FRAME}, got ${frame_id:-empty}"
  fi
fi

if timeout "${TIMEOUT}" rostopic hz "${SCAN_TOPIC}" -w 3 2>/tmp/stage9_scan_hz.err | tee /tmp/stage9_scan_hz.txt | grep -q "average rate:"; then
  pass "${SCAN_TOPIC} rate > 0"
else
  fail "${SCAN_TOPIC} rate check failed"
fi

check_tf "${BASE_FRAME}" "${LIDAR_FRAME}" base_lidar

check_msg /odom odom || true

if check_msg "${MAP_TOPIC}" map; then
  check_tf map odom map_odom
else
  echo "WARN: ${MAP_TOPIC} not active; start Gmapping before requiring map checks"
fi

check_tf odom "${SLAM_BASE_FRAME}" odom_base

exit "${FAILED}"
