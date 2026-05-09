#!/usr/bin/env bash

set -uo pipefail

PKG_PATH="$(rospack find slam_benchmark)"
LOG_DIR="${PKG_PATH}/results/stage9/logs"
LOG_FILE="${LOG_DIR}/diagnose_stage9_collision.log"
mkdir -p "${LOG_DIR}"

exec > >(tee "${LOG_FILE}") 2>&1

FAIL=0

pass() {
  echo "PASS: $*"
}

fail() {
  echo "FAIL: $*"
  FAIL=1
}

warn() {
  echo "WARN: $*"
}

topic_has_msg() {
  local topic="$1"
  if timeout 5 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    pass "topic ${topic} has messages"
  else
    fail "topic ${topic} has no message within 5s"
  fi
}

check_tf() {
  local parent="$1"
  local child="$2"
  timeout 5 rosrun tf tf_echo "${parent}" "${child}" >/tmp/stage9_tf_check.txt 2>&1 || true
  if grep -q "Translation:" /tmp/stage9_tf_check.txt 2>/dev/null; then
    pass "TF ${parent} -> ${child}"
  else
    fail "TF ${parent} -> ${child} unavailable"
    sed -n '1,5p' /tmp/stage9_tf_check.txt 2>/dev/null || true
  fi
}

scan_stats() {
  python3 - "$@" <<'PY'
import math
import sys
import rospy
from sensor_msgs.msg import LaserScan

topic = sys.argv[1]
front_angle = math.radians(float(sys.argv[2]))
rospy.init_node("diagnose_stage9_scan_sample", anonymous=True, disable_signals=True)
try:
    msg = rospy.wait_for_message(topic, LaserScan, timeout=5.0)
except Exception as exc:
    print(f"ERROR {exc}")
    sys.exit(2)

front = []
all_ranges = []
angle = msg.angle_min
for value in msg.ranges:
    if math.isfinite(value) and msg.range_min <= value <= msg.range_max:
        all_ranges.append(value)
        if abs(angle) <= front_angle:
            front.append(value)
    angle += msg.angle_increment

front_min = min(front) if front else float("inf")
global_min = min(all_ranges) if all_ranges else float("inf")
print(f"front_min={front_min:.4f}")
print(f"global_min={global_min:.4f}")
if front_min < 0.6:
    print("WARN_FRONT_TOO_CLOSE")
if global_min < 0.25:
    print("COLLISION_RISK")
PY
}

cmd_stats() {
  python3 - <<'PY'
import rospy
from geometry_msgs.msg import Twist

rospy.init_node("diagnose_stage9_cmd_sample", anonymous=True, disable_signals=True)
samples = []
deadline = rospy.Time.now() + rospy.Duration(5.0)
while len(samples) < 5 and rospy.Time.now() < deadline:
    try:
        samples.append(rospy.wait_for_message("/cmd_vel", Twist, timeout=1.0))
    except Exception:
        break

if not samples:
    print("ERROR no /cmd_vel samples")
    raise SystemExit(2)

for i, msg in enumerate(samples, 1):
    print(f"cmd_vel[{i}]: linear.x={msg.linear.x:.4f} linear.y={msg.linear.y:.4f} angular.z={msg.angular.z:.4f}")

if any(msg.linear.x > 0.15 for msg in samples):
    print("WARN_LINEAR_X_TOO_LARGE")
if any(abs(msg.angular.z) > 0.25 for msg in samples):
    print("WARN_ANGULAR_Z_TOO_LARGE")
if any(abs(msg.linear.y) > 1e-6 for msg in samples):
    print("WARN_LINEAR_Y_NONZERO")
PY
}

echo "Stage 9 collision diagnosis"
echo "timestamp=$(date -Is)"
echo "log=${LOG_FILE}"

for topic in /lidar/scan /odom /tf /gazebo/model_states /cmd_vel; do
  topic_has_msg "${topic}"
done

timeout 5 rosrun tf tf_echo odom base_link >/tmp/stage9_tf_odom_base.txt 2>&1 || true
if grep -q "Translation:" /tmp/stage9_tf_odom_base.txt 2>/dev/null; then
  pass "TF odom -> base_link"
else
  timeout 5 rosrun tf tf_echo odom base_footprint >/tmp/stage9_tf_odom_base.txt 2>&1 || true
  if grep -q "Translation:" /tmp/stage9_tf_odom_base.txt 2>/dev/null; then
  pass "TF odom -> base_footprint"
  else
  fail "TF odom -> base_link/base_footprint unavailable"
  fi
fi
check_tf base_link lidar_link
timeout 5 rosrun tf tf_echo map odom >/tmp/stage9_tf_map_odom.txt 2>&1 || true
if grep -q "Translation:" /tmp/stage9_tf_map_odom.txt 2>/dev/null; then
  pass "TF map -> odom"
else
  warn "TF map -> odom unavailable; expected if Gmapping is not running yet"
fi

echo "Scan safety sample:"
SCAN_OUTPUT="$(scan_stats /lidar/scan 20 2>&1)"
echo "${SCAN_OUTPUT}"
if grep -q '^ERROR' <<<"${SCAN_OUTPUT}"; then
  fail "could not read /lidar/scan"
fi
if grep -q 'WARN_FRONT_TOO_CLOSE' <<<"${SCAN_OUTPUT}"; then
  warn "robot front range is below 0.6 m"
fi
if grep -q 'COLLISION_RISK' <<<"${SCAN_OUTPUT}"; then
  fail "COLLISION_RISK: min scan range below 0.25 m"
fi

echo "Command velocity samples:"
CMD_OUTPUT="$(cmd_stats 2>&1)"
echo "${CMD_OUTPUT}"
if grep -q '^ERROR' <<<"${CMD_OUTPUT}"; then
  fail "could not read /cmd_vel"
fi
if grep -q 'WARN_' <<<"${CMD_OUTPUT}"; then
  warn "cmd_vel sample exceeded one or more Stage 9 safety expectations"
fi

if [[ "${FAIL}" -eq 0 ]]; then
  echo "OVERALL: PASS"
else
  echo "OVERALL: FAIL"
fi
exit "${FAIL}"
