#!/usr/bin/env bash

set -euo pipefail

pass=true

get_param() {
  local name="$1"
  if rosparam get "${name}" >/tmp/stage10_teb_param.txt 2>&1; then
    tr -d '\n' </tmp/stage10_teb_param.txt
  else
    echo "__MISSING__"
  fi
}

warn() {
  echo "WARN $*"
}

fail() {
  echo "FAIL $*"
  pass=false
}

check_numeric() {
  local name="$1"
  local op="$2"
  local ref="$3"
  local message="$4"
  local value
  value="$(get_param "${name}")"
  echo "${name}: ${value}"
  if [[ "${value}" == "__MISSING__" ]]; then
    fail "${name} is missing"
    return
  fi
  python3 - "$value" "$op" "$ref" <<'PY' || warn "$message"
import operator
import sys
value = float(sys.argv[1])
op = sys.argv[2]
ref = float(sys.argv[3])
ops = {
    "<=": operator.le,
    ">": operator.gt,
    ">=": operator.ge,
    "<": operator.lt,
}
sys.exit(0 if ops[op](value, ref) else 1)
PY
}

echo "Stage 10 TEB motion debug"
for param in \
  /move_base/TebLocalPlannerROS/max_vel_x \
  /move_base/TebLocalPlannerROS/max_vel_y \
  /move_base/TebLocalPlannerROS/max_vel_theta \
  /move_base/TebLocalPlannerROS/acc_lim_x \
  /move_base/TebLocalPlannerROS/acc_lim_y \
  /move_base/TebLocalPlannerROS/acc_lim_theta \
  /move_base/TebLocalPlannerROS/weight_kinematics_nh \
  /move_base/TebLocalPlannerROS/weight_kinematics_forward_drive \
  /move_base/TebLocalPlannerROS/weight_kinematics_turning_radius \
  /move_base/TebLocalPlannerROS/global_plan_overwrite_orientation \
  /move_base/TebLocalPlannerROS/allow_init_with_backwards_motion \
  /move_base/TebLocalPlannerROS/yaw_goal_tolerance; do
  echo "${param}: $(get_param "${param}")"
done

echo ""
echo "Heuristic checks"
check_numeric /move_base/TebLocalPlannerROS/max_vel_y ">" 0.0 "max_vel_y <= 0 disables holonomic lateral motion"
check_numeric /move_base/TebLocalPlannerROS/weight_kinematics_nh "<=" 5.0 "weight_kinematics_nh is high for an omni base"
check_numeric /move_base/TebLocalPlannerROS/max_vel_theta "<=" 0.5 "max_vel_theta is high for smooth debug navigation"
check_numeric /move_base/TebLocalPlannerROS/acc_lim_theta "<=" 0.8 "acc_lim_theta is high and can create yaw spikes"
check_numeric /move_base/TebLocalPlannerROS/yaw_goal_tolerance ">=" 0.2 "yaw_goal_tolerance is tight and may force final over-rotation"

overwrite="$(get_param /move_base/TebLocalPlannerROS/global_plan_overwrite_orientation)"
if [[ "${overwrite}" == "true" ]]; then
  warn "global_plan_overwrite_orientation=true can make TEB rotate the body along the path"
fi

echo ""
echo "Sample /cmd_vel"
if timeout 5 rostopic echo /cmd_vel -n 5; then
  echo "PASS /cmd_vel sample received"
else
  warn "No /cmd_vel samples received in 5s"
fi

if [[ "${pass}" == "true" ]]; then
  echo "OVERALL: PASS"
else
  echo "OVERALL: FAIL"
  exit 1
fi
