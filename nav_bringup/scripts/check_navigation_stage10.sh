#!/usr/bin/env bash

set -u

TIMEOUT="${TIMEOUT:-8}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
FAILED=0

pass() { echo "PASS: $*"; }
fail() { echo "FAIL: $*"; FAILED=1; }

check_msg() {
  local topic="$1"
  local label="$2"
  if timeout "${TIMEOUT}" rostopic echo -n 1 "${topic}" >/tmp/stage10_${label}.txt 2>&1; then
    pass "${topic} has messages"
  else
    fail "${topic} has no message within ${TIMEOUT}s"
  fi
}

check_tf() {
  local parent="$1"
  local child="$2"
  local label="$3"
  timeout "${TIMEOUT}" rosrun tf tf_echo "${parent}" "${child}" >/tmp/stage10_${label}_tf.txt 2>&1 || true
  if grep -q "Translation:" /tmp/stage10_${label}_tf.txt; then
    pass "TF ${parent} -> ${child} exists"
  else
    fail "TF ${parent} -> ${child} missing"
  fi
}

check_msg /map map
check_msg /lidar/scan scan
check_msg /odom odom
check_msg /amcl_pose amcl_pose
check_msg /particlecloud particlecloud
check_msg /move_base/status move_base_status
check_msg /move_base/global_costmap/costmap global_costmap
check_msg /move_base/local_costmap/costmap local_costmap

check_tf map odom map_odom
check_tf odom "${BASE_FRAME}" odom_base
check_tf base_link lidar_link base_lidar

exit "${FAILED}"
