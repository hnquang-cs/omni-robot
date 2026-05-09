#!/usr/bin/env bash

set -euo pipefail

MAX_X="${MAX_X:-0.20}"
MAX_WZ="${MAX_WZ:-0.30}"
EPS="${EPS:-0.02}"

run_case() {
  python3 - "$@" <<'PY'
import sys
import time
import rospy
from geometry_msgs.msg import Twist

mode = sys.argv[1]
vx_in = float(sys.argv[2])
vy_in = float(sys.argv[3])
wz_in = float(sys.argv[4])
max_x = float(sys.argv[5])
max_wz = float(sys.argv[6])
eps = float(sys.argv[7])
samples = []

def cb(msg):
    samples.append((msg.linear.x, msg.linear.y, msg.angular.z))

rospy.init_node("test_forward_only_filter_sample", anonymous=True)
pub = rospy.Publisher("/cmd_vel_raw", Twist, queue_size=10)
rospy.Subscriber("/cmd_vel", Twist, cb, queue_size=100)
time.sleep(0.2)

msg = Twist()
msg.linear.x = vx_in
msg.linear.y = vy_in
msg.angular.z = wz_in
deadline = time.monotonic() + 0.8
while time.monotonic() < deadline and not rospy.is_shutdown():
    pub.publish(msg)
    time.sleep(0.05)

time.sleep(0.2)
if not samples:
    print(f"FAIL {mode}: no /cmd_vel samples")
    sys.exit(1)

vx, vy, wz = max(samples, key=lambda s: abs(s[0]) + abs(s[1]) + abs(s[2]))
print(f"{mode}: linear.x={vx:.6f} linear.y={vy:.6f} angular.z={wz:.6f}")

if vx < -1e-6:
    print(f"FAIL {mode}: output linear.x is negative")
    sys.exit(1)
if abs(vy) > 1e-6:
    print(f"FAIL {mode}: output linear.y is non-zero")
    sys.exit(1)
if abs(wz) > max_wz + 1e-6:
    print(f"FAIL {mode}: output angular.z exceeds limit")
    sys.exit(1)

if mode == "forward" and vx <= 0.0:
    print("FAIL forward: expected positive linear.x")
    sys.exit(1)
if mode == "reverse" and vx > eps:
    print("FAIL reverse: expected reverse command to clamp near zero")
    sys.exit(1)
if mode == "lateral" and abs(vy) > 1e-6:
    print("FAIL lateral: expected linear.y to clamp to zero")
    sys.exit(1)
if mode == "turn" and abs(wz) <= 0.0:
    print("FAIL turn: expected non-zero angular.z")
    sys.exit(1)
if mode == "overspeed" and (vx > max_x + 1e-6 or abs(wz) > max_wz + 1e-6):
    print("FAIL overspeed: expected clamp to configured max")
    sys.exit(1)

print(f"PASS {mode}")
PY
}

for topic in /cmd_vel_raw /cmd_vel; do
  if ! rostopic list 2>/dev/null | grep -qx "${topic}"; then
    echo "FAIL: ${topic} missing. Start roslaunch omni_base_controller cmd_vel_forward_only_filter.launch first."
    exit 1
  fi
done

echo "Testing forward-only filter"

run_case forward 0.1 0.0 0.0 "${MAX_X}" "${MAX_WZ}" "${EPS}"
run_case reverse -0.1 0.0 0.0 "${MAX_X}" "${MAX_WZ}" "${EPS}"
run_case lateral 0.0 0.1 0.0 "${MAX_X}" "${MAX_WZ}" "${EPS}"
run_case turn 0.0 0.0 0.2 "${MAX_X}" "${MAX_WZ}" "${EPS}"
run_case overspeed 1.0 0.2 2.0 "${MAX_X}" "${MAX_WZ}" "${EPS}"

rostopic pub -1 /cmd_vel_raw geometry_msgs/Twist \
  "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" >/dev/null
echo "OVERALL: PASS"
