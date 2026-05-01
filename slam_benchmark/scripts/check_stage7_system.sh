#!/usr/bin/env bash

# End-to-end Stage 7 runtime check. Always sends a stop command before exit.

set -euo pipefail

MODEL_NAME="${1:-omni_robot}"
pass_count=0
fail_count=0

stop_robot() {
  rostopic pub /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1 >/dev/null 2>&1 || true
}
trap stop_robot EXIT

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
  rostopic list | grep -Fxq "${topic}" && timeout 6 rostopic echo -n 1 "${topic}" >/dev/null 2>&1
}

if ! rostopic list >/dev/null 2>&1; then
  echo "FAIL: ROS master is not reachable"
  exit 1
fi

if topic_has_message /gazebo/model_states; then pass "/gazebo/model_states has data"; else fail "/gazebo/model_states missing or silent"; fi
if topic_has_message /scan; then pass "/scan has data"; else fail "/scan missing or silent"; fi
if topic_has_message /map; then pass "/map has data"; else fail "/map missing or silent"; fi

if timeout 12 python3 - "${MODEL_NAME}" <<'PY'
import math
import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

model_name = sys.argv[1]
latest_model = {"value": None}
latest_odom = {"value": None}


def model_cb(msg):
    latest_model["value"] = msg


def odom_cb(msg):
    latest_odom["value"] = msg


def yaw_from_quaternion(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def sample_model(timeout_sec=5.0):
    start = time.time()
    while time.time() - start < timeout_sec and not rospy.is_shutdown():
        msg = latest_model["value"]
        if msg and model_name in msg.name:
            index = msg.name.index(model_name)
            pose = msg.pose[index]
            return pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)
        time.sleep(0.05)
    return None


def sample_odom(timeout_sec=5.0):
    start = time.time()
    while time.time() - start < timeout_sec and not rospy.is_shutdown():
        msg = latest_odom["value"]
        if msg:
            pose = msg.pose.pose
            return pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)
        time.sleep(0.05)
    return None


def angle_delta(after, before):
    return math.atan2(math.sin(after - before), math.cos(after - before))


rospy.init_node("check_stage7_motion", anonymous=True, disable_signals=True)
rospy.Subscriber("/gazebo/model_states", ModelStates, model_cb, queue_size=1)
rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=1)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

before_model = sample_model()
before_odom = sample_odom()
cmd = Twist()
cmd.angular.z = 0.25
rate = rospy.Rate(10)
start = time.time()
while time.time() - start < 2.0 and not rospy.is_shutdown():
    pub.publish(cmd)
    rate.sleep()

stop = Twist()
for _ in range(5):
    pub.publish(stop)
    time.sleep(0.05)

after_model = sample_model()
after_odom = sample_odom()
if before_model is None or after_model is None:
    print("MODEL_FAIL no model sample")
    sys.exit(2)
if before_odom is None or after_odom is None:
    print("ODOM_FAIL no odom sample")
    sys.exit(3)

model_dyaw = abs(angle_delta(after_model[2], before_model[2]))
odom_dyaw = abs(angle_delta(after_odom[2], before_odom[2]))
model_moved = model_dyaw > 0.05 or math.hypot(after_model[0] - before_model[0], after_model[1] - before_model[1]) > 0.02
odom_moved = odom_dyaw > 0.05 or math.hypot(after_odom[0] - before_odom[0], after_odom[1] - before_odom[1]) > 0.02
print(f"model_dyaw={model_dyaw:.4f} odom_dyaw={odom_dyaw:.4f}")
if not model_moved:
    sys.exit(4)
if not odom_moved:
    sys.exit(5)
sys.exit(0)
PY
then
  pass "Gazebo model and /odom changed after /cmd_vel yaw command"
else
  fail "Gazebo model or /odom did not change after /cmd_vel yaw command"
fi

if timeout 8 python3 - >/dev/null 2>&1 <<'PY'
import rospy
import tf

rospy.init_node("check_stage7_tf_once", anonymous=True, disable_signals=True)
listener = tf.TransformListener()
try:
    listener.waitForTransform("odom", "base_footprint", rospy.Time(0), rospy.Duration(5.0))
except Exception:
    listener.waitForTransform("odom", "base_link", rospy.Time(0), rospy.Duration(5.0))
PY
then
  pass "TF odom -> base_footprint/base_link available"
else
  fail "TF odom -> base_footprint/base_link unavailable"
fi

if timeout 8 python3 - >/dev/null 2>&1 <<'PY'
import rospy
import tf

rospy.init_node("check_stage7_map_tf_once", anonymous=True, disable_signals=True)
listener = tf.TransformListener()
listener.waitForTransform("map", "odom", rospy.Time(0), rospy.Duration(5.0))
PY
then
  pass "TF map -> odom available"
else
  fail "TF map -> odom unavailable"
fi

echo "Summary: PASS=${pass_count} FAIL=${fail_count}"

if [ "${fail_count}" -gt 0 ]; then
  exit 1
fi
