#!/usr/bin/env bash

set -euo pipefail

PKG_PATH="$(rospack find nav_bringup)"
LOG_DIR="${PKG_PATH}/results/stage10/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/cmd_vel_forward_only_check.log"
DURATION="${DURATION:-5.0}"
MAX_WZ="${MAX_WZ:-0.30}"
EPS="${EPS:-0.000001}"

{
  echo "Stage 10 forward-only /cmd_vel check"
  echo "Timestamp: $(date --iso-8601=seconds)"

  if rostopic list 2>/dev/null | grep -qx /cmd_vel_raw; then
    echo "PASS /cmd_vel_raw exists"
  else
    echo "FAIL /cmd_vel_raw missing"
    exit 1
  fi

  if rostopic list 2>/dev/null | grep -qx /cmd_vel; then
    echo "PASS /cmd_vel exists"
  else
    echo "FAIL /cmd_vel missing"
    exit 1
  fi

  python3 - "${DURATION}" "${MAX_WZ}" "${EPS}" <<'PY'
import sys
import rospy
from geometry_msgs.msg import Twist

duration = float(sys.argv[1])
max_wz = float(sys.argv[2])
eps = float(sys.argv[3])
samples = []

def callback(msg):
    samples.append((msg.linear.x, msg.linear.y, msg.angular.z))

rospy.init_node("check_cmd_vel_forward_only", anonymous=True)
rospy.Subscriber("/cmd_vel", Twist, callback, queue_size=100)
rospy.sleep(duration)

if not samples:
    print("FAIL /cmd_vel no samples received")
    sys.exit(1)

for i, (vx, vy, wz) in enumerate(samples[:5]):
    print(f"sample[{i}] linear.x={vx:.6f} linear.y={vy:.6f} angular.z={wz:.6f}")

bad_reverse = [s for s in samples if s[0] < -eps]
bad_lateral = [s for s in samples if abs(s[1]) > eps]
bad_wz = [s for s in samples if abs(s[2]) > max_wz + eps]

print(f"samples={len(samples)}")
print(f"min_linear_x={min(s[0] for s in samples):.6f}")
print(f"max_abs_linear_y={max(abs(s[1]) for s in samples):.6f}")
print(f"max_abs_angular_z={max(abs(s[2]) for s in samples):.6f}")

if bad_reverse:
    print("FAIL linear.x negative command detected")
    sys.exit(1)
if bad_lateral:
    print("FAIL linear.y non-zero command detected")
    sys.exit(1)
if bad_wz:
    print("FAIL angular.z exceeds configured limit")
    sys.exit(1)

print("PASS linear.x >= 0")
print("PASS linear.y == 0")
print("PASS abs(angular.z) <= max_vel_theta")
PY
} | tee "${LOG_FILE}"
