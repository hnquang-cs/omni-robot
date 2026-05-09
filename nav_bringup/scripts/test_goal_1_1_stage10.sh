#!/usr/bin/env bash

set -euo pipefail

PKG_PATH="$(rospack find nav_bringup)"
CSV_DIR="${PKG_PATH}/results/stage10/csv"
mkdir -p "${CSV_DIR}"
CSV_FILE="${CSV_DIR}/test_goal_1_1_result.csv"

GOAL_X="${GOAL_X:-1.0}"
GOAL_Y="${GOAL_Y:-1.0}"
GOAL_YAW="${GOAL_YAW:-0.0}"
TIMEOUT_SEC="${TIMEOUT_SEC:-90}"

write_csv() {
  local goal_status="$1"
  local mb_status="$2"
  local cmd_vel="$3"
  local success="$4"
  local duration="$5"
  local notes="$6"
  {
    echo "goal_x,goal_y,goal_yaw,goal_cell_status,move_base_status,cmd_vel_published,success,duration_sec,notes"
    echo "${GOAL_X},${GOAL_Y},${GOAL_YAW},${goal_status},${mb_status},${cmd_vel},${success},${duration},${notes}"
  } >"${CSV_FILE}"
  echo "Result CSV: ${CSV_FILE}"
}

require_topic() {
  local topic="$1"
  if timeout 5 rostopic echo -n 1 "${topic}" >/tmp/stage10_test_topic.txt 2>&1; then
    echo "PASS topic ${topic}"
  else
    echo "FAIL topic ${topic}: no message"
    write_csv "N/A" "missing_${topic}" "false" "false" "0" "required_topic_missing"
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
  timeout 3 rostopic echo -n 1 /move_base/status 2>/dev/null \
    | awk '/status:/{value=$2} END{if (value != "") print value; else print "0"}'
}

echo "Stage 10 goal test: x=${GOAL_X}, y=${GOAL_Y}, yaw=${GOAL_YAW}"
require_topic /map
require_topic /amcl_pose
require_topic /move_base/status

set +e
goal_output="$(rosrun nav_bringup check_goal_on_map.py --x "${GOAL_X}" --y "${GOAL_Y}" --yaw "${GOAL_YAW}" 2>&1)"
goal_rc=$?
set -e
echo "${goal_output}"
goal_status="$(printf "%s\n" "${goal_output}" | awk -F': ' '/^status:/{print $2; exit}')"
goal_status="${goal_status:-UNKNOWN}"

if [[ "${goal_rc}" != "0" ]]; then
  echo "FAIL: goal is ${goal_status}; not sending goal."
  write_csv "${goal_status}" "NOT_SENT" "false" "false" "0" "goal_not_free"
  exit 1
fi

cmd_tmp="$(mktemp)"
timeout 10 rostopic echo -n 1 /cmd_vel >"${cmd_tmp}" 2>&1 &
cmd_pid=$!

start_epoch="$(date +%s)"
rosrun nav_bringup send_nav_goal.py --x "${GOAL_X}" --y "${GOAL_Y}" --yaw "${GOAL_YAW}"

final_status="0"
success=false
notes="timeout"
while (( $(date +%s) - start_epoch < TIMEOUT_SEC )); do
  status="$(read_move_base_status)"
  final_status="${status}"
  if [[ "${status}" == "3" ]]; then
    success=true
    notes="move_base_succeeded"
    break
  fi
  if [[ "${status}" == "4" || "${status}" == "5" || "${status}" == "9" ]]; then
    notes="move_base_terminal_failure"
    break
  fi
  sleep 1
done

wait "${cmd_pid}" >/dev/null 2>&1 || true
if [[ -s "${cmd_tmp}" ]] && grep -q "linear:" "${cmd_tmp}"; then
  cmd_vel_published=true
else
  cmd_vel_published=false
fi
rm -f "${cmd_tmp}"

duration="$(( $(date +%s) - start_epoch ))"
status_name="$(status_code_name "${final_status}")"
write_csv "${goal_status}" "${status_name}" "${cmd_vel_published}" "${success}" "${duration}" "${notes}"

if [[ "${success}" == "true" ]]; then
  echo "PASS: goal reached"
  exit 0
fi

echo "FAIL: goal did not succeed (${status_name}, cmd_vel_published=${cmd_vel_published})"
exit 1
