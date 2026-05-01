#!/usr/bin/env bash

# Check the Stage 6 SLAM graph without hanging indefinitely.

set -euo pipefail

pass_count=0
fail_count=0

pass() {
  echo "PASS: $1"
  pass_count=$((pass_count + 1))
}

fail() {
  echo "FAIL: $1"
  fail_count=$((fail_count + 1))
}

topic_has_message() {
  local topic="$1"
  rostopic list | grep -Fxq "${topic}" && timeout 5 rostopic echo -n 1 "${topic}" >/dev/null 2>&1
}

tf_available() {
  local parent="$1"
  local child="$2"
  timeout 8 python3 - "${parent}" "${child}" >/dev/null 2>&1 <<'PY'
import sys

import rospy
import tf

parent = sys.argv[1]
child = sys.argv[2]

rospy.init_node("check_slam_tf_once", anonymous=True, disable_signals=True)
listener = tf.TransformListener()

try:
    listener.waitForTransform(parent, child, rospy.Time(0), rospy.Duration(5.0))
except Exception:
    sys.exit(1)

sys.exit(0)
PY
}

if ! rostopic list >/dev/null 2>&1; then
  echo "FAIL: ROS master is not reachable"
  exit 1
fi

if topic_has_message /scan; then pass "/scan exists and has data"; else fail "/scan missing or silent"; fi
if topic_has_message /odom; then pass "/odom exists and has data"; else fail "/odom missing or silent"; fi
if topic_has_message /map; then pass "/map exists and has data"; else fail "/map missing or silent"; fi

if tf_available map odom; then pass "TF map -> odom available"; else fail "TF map -> odom unavailable"; fi

if tf_available odom base_footprint; then
  pass "TF odom -> base_footprint available"
elif tf_available odom base_link; then
  pass "TF odom -> base_link available"
else
  fail "TF odom -> base_footprint/base_link unavailable"
fi

if rosnode list | grep -Fxq /slam_gmapping; then
  pass "node /slam_gmapping is running"
else
  fail "node /slam_gmapping is not running"
fi

echo "Summary: PASS=${pass_count} FAIL=${fail_count}"

if [ "${fail_count}" -gt 0 ]; then
  exit 1
fi
