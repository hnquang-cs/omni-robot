#!/usr/bin/env bash

set -euo pipefail

PKG_PATH="$(rospack find nav_bringup)"
CSV_DIR="${PKG_PATH}/results/stage10/csv"
mkdir -p "${CSV_DIR}"
CSV_FILE="${CSV_DIR}/test_goal_forward_only_result.csv"

GOAL_X="${GOAL_X:-1.0}"
GOAL_Y="${GOAL_Y:-1.0}"
GOAL_YAW="${GOAL_YAW:-0.0}"
TIMEOUT_SEC="${TIMEOUT_SEC:-90}"
MAX_WZ="${MAX_WZ:-0.30}"

write_csv() {
  local goal_free="$1"
  local mb_status="$2"
  local raw_seen="$3"
  local cmd_seen="$4"
  local reverse_blocked="$5"
  local lateral_blocked="$6"
  local success="$7"
  local duration="$8"
  local notes="$9"
  {
    echo "goal_x,goal_y,goal_yaw,goal_free,move_base_status,cmd_vel_raw_seen,cmd_vel_seen,reverse_command_blocked,lateral_command_blocked,success,duration_sec,notes"
    echo "${GOAL_X},${GOAL_Y},${GOAL_YAW},${goal_free},${mb_status},${raw_seen},${cmd_seen},${reverse_blocked},${lateral_blocked},${success},${duration},${notes}"
  } >"${CSV_FILE}"
  echo "Result CSV: ${CSV_FILE}"
}

require_topic_msg() {
  local topic="$1"
  timeout 8 rostopic echo -n 1 "${topic}" >/tmp/stage10_forward_topic.txt 2>&1 || {
    echo "FAIL: ${topic} has no message"
    write_csv false "missing_${topic}" false false false false false 0 "required_topic_missing"
    exit 1
  }
  echo "PASS topic ${topic}"
}

require_topic_exists() {
  local topic="$1"
  if rostopic list 2>/dev/null | grep -qx "${topic}"; then
    echo "PASS topic ${topic} exists"
  else
    echo "FAIL: ${topic} missing"
    write_csv false "missing_${topic}" false false false false false 0 "required_topic_missing"
    exit 1
  fi
}

status_code_name() {
  case "$1" in
    1) echo "ACTIVE" ;;
    2) echo "PREEMPTED" ;;
    3) echo "SUCCEEDED" ;;
    4) echo "ABORTED" ;;
    5) echo "REJECTED" ;;
    8) echo "RECALLED" ;;
    9) echo "LOST" ;;
    *) echo "UNKNOWN_${1}" ;;
  esac
}

read_move_base_status() {
  (timeout 3 rostopic echo -n 1 /move_base/status 2>/dev/null || true) \
    | awk '/status:/{value=$2} END{if (value != "") print value; else print "0"}'
}

monitor_file="$(mktemp)"
python3 - "${TIMEOUT_SEC}" "${MAX_WZ}" "${monitor_file}" <<'PY' &
import signal
import sys
import rospy
from geometry_msgs.msg import Twist

timeout = float(sys.argv[1])
max_wz = float(sys.argv[2])
out_path = sys.argv[3]
raw_seen = False
cmd_seen = False
bad_reverse = False
bad_lateral = False
bad_wz = False
raw_reverse_seen = False
raw_lateral_seen = False
samples = 0

def write_result(*_args):
    with open(out_path, "w") as handle:
        handle.write(f"raw_seen={str(raw_seen).lower()}\n")
        handle.write(f"cmd_seen={str(cmd_seen).lower()}\n")
        handle.write(f"bad_reverse={str(bad_reverse).lower()}\n")
        handle.write(f"bad_lateral={str(bad_lateral).lower()}\n")
        handle.write(f"bad_wz={str(bad_wz).lower()}\n")
        handle.write(f"raw_reverse_seen={str(raw_reverse_seen).lower()}\n")
        handle.write(f"raw_lateral_seen={str(raw_lateral_seen).lower()}\n")
        handle.write(f"samples={samples}\n")
    sys.exit(0)

def raw_cb(msg):
    global raw_seen, raw_reverse_seen, raw_lateral_seen
    raw_seen = True
    raw_reverse_seen = raw_reverse_seen or msg.linear.x < -1e-6
    raw_lateral_seen = raw_lateral_seen or abs(msg.linear.y) > 1e-6

def cmd_cb(msg):
    global cmd_seen, bad_reverse, bad_lateral, bad_wz, samples
    cmd_seen = True
    samples += 1
    bad_reverse = bad_reverse or msg.linear.x < -1e-6
    bad_lateral = bad_lateral or abs(msg.linear.y) > 1e-6
    bad_wz = bad_wz or abs(msg.angular.z) > max_wz + 1e-6

signal.signal(signal.SIGTERM, write_result)
signal.signal(signal.SIGINT, write_result)
rospy.init_node("test_goal_forward_only_monitor", anonymous=True)
rospy.Subscriber("/cmd_vel_raw", Twist, raw_cb, queue_size=100)
rospy.Subscriber("/cmd_vel", Twist, cmd_cb, queue_size=100)
rospy.sleep(timeout)
write_result()
PY
monitor_pid=$!

cleanup() {
  kill "${monitor_pid}" >/dev/null 2>&1 || true
  wait "${monitor_pid}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

for topic in /map /lidar/scan /odom /amcl_pose /move_base/status; do
  require_topic_msg "${topic}"
done
require_topic_exists /cmd_vel_raw
require_topic_exists /cmd_vel

set +e
goal_output="$(rosrun nav_bringup check_goal_on_map.py --x "${GOAL_X}" --y "${GOAL_Y}" --yaw "${GOAL_YAW}" 2>&1)"
goal_rc=$?
set -e
echo "${goal_output}"
if [[ "${goal_rc}" != "0" ]]; then
  write_csv false "NOT_SENT" false false false false false 0 "goal_not_free"
  exit 1
fi

start_epoch="$(date +%s)"
rosrun nav_bringup send_nav_goal.py --x "${GOAL_X}" --y "${GOAL_Y}" --yaw "${GOAL_YAW}"

final_status="0"
success=false
notes="timeout"
while (( $(date +%s) - start_epoch < TIMEOUT_SEC )); do
  status="$(read_move_base_status)"
  final_status="${status}"
  case "${status}" in
    3) success=true; notes="move_base_succeeded"; break ;;
    4|5|9) notes="move_base_terminal_failure"; break ;;
  esac
  sleep 1
done
duration="$(( $(date +%s) - start_epoch ))"

cleanup
source "${monitor_file}" || true
rm -f "${monitor_file}"

raw_seen="${raw_seen:-false}"
cmd_seen="${cmd_seen:-false}"
bad_reverse="${bad_reverse:-true}"
bad_lateral="${bad_lateral:-true}"
bad_wz="${bad_wz:-true}"
raw_reverse_seen="${raw_reverse_seen:-false}"
raw_lateral_seen="${raw_lateral_seen:-false}"

reverse_blocked=true
lateral_blocked=true
if [[ "${bad_reverse}" == "true" ]]; then
  reverse_blocked=false
fi
if [[ "${bad_lateral}" == "true" ]]; then
  lateral_blocked=false
fi
if [[ "${raw_reverse_seen}" == "false" ]]; then
  reverse_blocked="not_needed"
fi
if [[ "${raw_lateral_seen}" == "false" ]]; then
  lateral_blocked="not_needed"
fi

status_name="$(status_code_name "${final_status}")"
if [[ "${bad_wz}" == "true" ]]; then
  notes="${notes}; angular_limit_violation"
  success=false
fi

write_csv true "${status_name}" "${raw_seen}" "${cmd_seen}" "${reverse_blocked}" "${lateral_blocked}" "${success}" "${duration}" "${notes}"

if [[ "${success}" == "true" && "${cmd_seen}" == "true" && "${bad_reverse}" == "false" && "${bad_lateral}" == "false" && "${bad_wz}" == "false" ]]; then
  echo "PASS: forward-only goal test"
  exit 0
fi

echo "FAIL: forward-only goal test (${status_name}, notes=${notes})"
exit 1
