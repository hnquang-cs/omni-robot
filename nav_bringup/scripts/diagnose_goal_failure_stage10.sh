#!/usr/bin/env bash

set -euo pipefail

PKG_PATH="$(rospack find nav_bringup)"
LOG_DIR="${PKG_PATH}/results/stage10/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/diagnose_goal_failure_stage10.log"

GOAL_X="${GOAL_X:-1.0}"
GOAL_Y="${GOAL_Y:-1.0}"
GOAL_YAW="${GOAL_YAW:-0.0}"

pass=true

log() {
  echo "$*"
}

check_topic() {
  local topic="$1"
  if timeout 5 rostopic echo -n 1 "${topic}" >/tmp/stage10_topic_check.txt 2>&1; then
    log "PASS topic ${topic}: message received"
  else
    log "FAIL topic ${topic}: no message within 5s"
    pass=false
  fi
}

check_tf() {
  local parent="$1"
  local child="$2"
  set +e
  timeout 5 rosrun tf tf_echo "${parent}" "${child}" >/tmp/stage10_tf_check.txt 2>&1
  local status=$?
  set -e
  if grep -q "Translation:" /tmp/stage10_tf_check.txt; then
    log "PASS TF ${parent} -> ${child}"
  else
    log "FAIL TF ${parent} -> ${child} (tf_echo exit ${status})"
    pass=false
  fi
}

print_param() {
  local name="$1"
  if rosparam get "${name}" >/tmp/stage10_param_check.txt 2>&1; then
    log "PASS param ${name}: $(tr '\n' ' ' </tmp/stage10_param_check.txt)"
  else
    log "FAIL param ${name}: not set"
  fi
}

topic_exists() {
  rostopic list 2>/dev/null | grep -qx "$1"
}

check_plan_topic() {
  local topic="$1"
  if topic_exists "${topic}"; then
    log "PASS planner topic ${topic}: exists"
    if timeout 3 rostopic echo -n 1 "${topic}" >/tmp/stage10_plan_check.txt 2>&1; then
      log "PASS planner topic ${topic}: message received"
    else
      log "WARN planner topic ${topic}: exists but no message within 3s"
    fi
  else
    log "FAIL planner topic ${topic}: missing"
  fi
}

check_amcl_pose_on_map() {
  local pose x y
  pose="$(timeout 5 rostopic echo -n 1 /amcl_pose 2>/dev/null || true)"
  x="$(printf "%s\n" "${pose}" | awk '/position:/{seen=1} seen && /^[[:space:]]*x:/{print $2; exit}')"
  y="$(printf "%s\n" "${pose}" | awk '/position:/{seen=1} seen && /^[[:space:]]*y:/{print $2; exit}')"
  if [[ -z "${x}" || -z "${y}" ]]; then
    log "FAIL robot map cell: could not parse /amcl_pose"
    pass=false
    return
  fi
  log "Robot AMCL pose: x=${x}, y=${y}"
  if rosrun nav_bringup check_goal_on_map.py --x "${x}" --y "${y}" --yaw 0.0 >/tmp/stage10_robot_cell.txt 2>&1; then
    sed 's/^/  /' /tmp/stage10_robot_cell.txt
    log "PASS robot current cell: FREE"
  else
    sed 's/^/  /' /tmp/stage10_robot_cell.txt
    log "FAIL robot current cell: not FREE"
    pass=false
  fi
}

main() {
  {
    log "Stage 10 goal failure diagnosis"
    log "Timestamp: $(date --iso-8601=seconds)"
    log "Goal: x=${GOAL_X}, y=${GOAL_Y}, yaw=${GOAL_YAW}"
    log ""

    log "== Basic topics =="
    for topic in \
      /map \
      /lidar/scan \
      /odom \
      /amcl_pose \
      /move_base/status \
      /move_base/global_costmap/costmap \
      /move_base/local_costmap/costmap; do
      check_topic "${topic}"
    done

    log ""
    log "== TF =="
    check_tf map odom
    if rosrun tf tf_echo odom base_footprint >/tmp/stage10_tf_base_footprint.txt 2>&1 & pid=$!; then
      sleep 3
      kill "${pid}" >/dev/null 2>&1 || true
      wait "${pid}" >/dev/null 2>&1 || true
    fi
    if grep -q "Translation:" /tmp/stage10_tf_base_footprint.txt 2>/dev/null; then
      log "PASS TF odom -> base_footprint"
      check_tf base_footprint base_link
    else
      log "WARN TF odom -> base_footprint unavailable; checking odom -> base_link"
      check_tf odom base_link
    fi
    check_tf base_link lidar_link

    log ""
    log "== Goal validity on /map =="
    if rosrun nav_bringup check_goal_on_map.py --x "${GOAL_X}" --y "${GOAL_Y}" --yaw "${GOAL_YAW}" >/tmp/stage10_goal_cell.txt 2>&1; then
      sed 's/^/  /' /tmp/stage10_goal_cell.txt
      log "PASS goal cell: FREE"
      goal_free=true
    else
      sed 's/^/  /' /tmp/stage10_goal_cell.txt
      log "FAIL goal cell: not valid for planning"
      goal_free=false
      pass=false
    fi
    check_amcl_pose_on_map

    log ""
    log "== Costmap parameters =="
    for param in \
      /move_base/global_costmap/footprint \
      /move_base/local_costmap/footprint \
      /move_base/global_costmap/inflation_layer/inflation_radius \
      /move_base/local_costmap/inflation_layer/inflation_radius \
      /move_base/local_costmap/obstacle_layer/observation_sources \
      /move_base/local_costmap/obstacle_layer/lidar \
      /move_base/local_costmap/obstacle_layer/lidar_scan; do
      print_param "${param}"
    done

    log ""
    log "== Planner topics =="
    if topic_exists /move_base/GlobalPlanner/plan; then
      check_plan_topic /move_base/GlobalPlanner/plan
    else
      check_plan_topic /move_base/NavfnROS/plan
    fi
    check_plan_topic /move_base/TebLocalPlannerROS/local_plan

    log ""
    log "== /cmd_vel after test goal =="
    if [[ "${goal_free}" == "true" ]]; then
      rosrun nav_bringup send_nav_goal.py --x "${GOAL_X}" --y "${GOAL_Y}" --yaw "${GOAL_YAW}" >/tmp/stage10_send_goal.txt 2>&1 || true
      sed 's/^/  /' /tmp/stage10_send_goal.txt
      if timeout 10 rostopic echo -n 1 /cmd_vel >/tmp/stage10_cmd_vel_check.txt 2>&1; then
        log "PASS /cmd_vel: message received after goal"
      else
        log "FAIL /cmd_vel: no message within 10s after goal"
        pass=false
      fi
    else
      log "SKIP /cmd_vel goal test: goal is not FREE"
    fi

    log ""
    if [[ "${pass}" == "true" ]]; then
      log "OVERALL: PASS"
    else
      log "OVERALL: FAIL"
    fi
    log "Log: ${LOG_FILE}"
  } | tee "${LOG_FILE}"

  [[ "${pass}" == "true" ]]
}

main "$@"
