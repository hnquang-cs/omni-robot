#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 4 ]]; then
  echo "Usage: $0 <scenario_name> <goal_x> <goal_y> <goal_yaw>"
  exit 2
fi

SCENARIO="$1"
GOAL_X="$2"
GOAL_Y="$3"
GOAL_YAW="$4"
TIMEOUT_SEC="${TIMEOUT_SEC:-120}"
PKG_PATH="$(rospack find nav_bringup)"
OUT="${PKG_PATH}/results/stage10/csv/navigation_trials.csv"
mkdir -p "$(dirname "${OUT}")"

if [[ ! -f "${OUT}" ]]; then
  echo "trial,goal_x,goal_y,goal_yaw,start_time,finish_time,runtime_sec,success,status_code,notes" > "${OUT}"
fi

start_epoch="$(date +%s)"
start_iso="$(date -Is)"
rosrun nav_bringup send_nav_goal.py --x "${GOAL_X}" --y "${GOAL_Y}" --yaw "${GOAL_YAW}"

success=false
status_code="N/A"
notes="timeout"
deadline=$((start_epoch + TIMEOUT_SEC))
while [[ "$(date +%s)" -lt "${deadline}" ]]; do
  line="$(timeout 3 rostopic echo -n 1 /move_base/status 2>/dev/null | awk '/^[[:space:]]+status:/ {status=$2} END {print status}' || true)"
  if [[ -n "${line}" ]]; then
    status_code="${line}"
    if [[ "${status_code}" == "3" ]]; then
      success=true
      notes="SUCCEEDED"
      break
    fi
    if [[ "${status_code}" == "4" || "${status_code}" == "5" ]]; then
      notes="ABORTED_OR_REJECTED"
      break
    fi
  fi
  sleep 2
done

finish_epoch="$(date +%s)"
finish_iso="$(date -Is)"
runtime=$((finish_epoch - start_epoch))
echo "${SCENARIO},${GOAL_X},${GOAL_Y},${GOAL_YAW},${start_iso},${finish_iso},${runtime},${success},${status_code},${notes}" >> "${OUT}"
echo "Trial result: scenario=${SCENARIO} success=${success} status=${status_code} runtime=${runtime}s"
