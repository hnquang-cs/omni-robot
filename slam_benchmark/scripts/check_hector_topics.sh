#!/usr/bin/env bash

# Check Hector SLAM topics and transforms without hanging indefinitely.

set -euo pipefail

MAP_TOPIC="${1:-/map_hector}"
MAP_FRAME="${2:-hector_map}"
ODOM_FRAME="${3:-odom}"

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

rospy.init_node("check_hector_tf_once", anonymous=True, disable_signals=True)
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

if topic_has_message /scan; then pass "/scan has data"; else fail "/scan missing or silent"; fi
if topic_has_message "${MAP_TOPIC}"; then pass "${MAP_TOPIC} has data"; else fail "${MAP_TOPIC} missing or silent"; fi
if tf_available "${MAP_FRAME}" "${ODOM_FRAME}"; then pass "TF ${MAP_FRAME} -> ${ODOM_FRAME} available"; else fail "TF ${MAP_FRAME} -> ${ODOM_FRAME} unavailable"; fi

if rosnode list | grep -Fxq /hector_mapping; then
  pass "node /hector_mapping is running"
else
  fail "node /hector_mapping is not running"
fi

echo "Summary: PASS=${pass_count} FAIL=${fail_count}"

if [ "${fail_count}" -gt 0 ]; then
  exit 1
fi
