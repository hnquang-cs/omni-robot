#!/usr/bin/env bash

set -uo pipefail

PKG_PATH="$(rospack find slam_benchmark)"
WS_ROOT="$(cd "${PKG_PATH}/../../.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${PKG_PATH}/results/stage9/logs"
TRIAL_DIR="${PKG_PATH}/results/stage9/raw/corridor_static/gmapping_lidar/rep_acceptance_${STAMP}"
MAP_PREFIX="${TRIAL_DIR}/map"
LOG_FILE="${LOG_DIR}/acceptance_stage9_safe_mapping_${STAMP}.log"
mkdir -p "${LOG_DIR}" "${TRIAL_DIR}"

exec > >(tee "${LOG_FILE}") 2>&1

FAIL=0
LAUNCH_PID=""

mark_pass() { echo "PASS: $*"; }
mark_fail() { echo "FAIL: $*"; FAIL=1; }

cleanup() {
  for _ in 1 2 3 4 5; do
    rostopic pub -1 /cmd_vel_raw geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
    rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
  done
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 2
    kill -TERM "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

topic_check() {
  local topic="$1"
  if timeout 8 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    mark_pass "${topic}"
  else
    mark_fail "${topic}"
  fi
}

echo "Stage 9 safe mapping acceptance test"
echo "workspace=${WS_ROOT}"
echo "log=${LOG_FILE}"

cd "${WS_ROOT}" || exit 1
if catkin_make; then
  mark_pass "catkin_make"
else
  mark_fail "catkin_make"
  exit 1
fi
source "${WS_ROOT}/devel/setup.bash"

roslaunch slam_benchmark slam_lidar_wide_safe_full.launch use_rviz:=false gazebo_gui:=false >"${LOG_DIR}/acceptance_launch_${STAMP}.log" 2>&1 &
LAUNCH_PID=$!

timeout 90 bash -c 'until rostopic list 2>/dev/null | grep -Fxq /lidar/scan && rostopic list | grep -Fxq /odom && rostopic list | grep -Fxq /map && rostopic list | grep -Fxq /gazebo/model_states; do sleep 2; done'

topic_check /lidar/scan
topic_check /odom
topic_check /map
topic_check /gazebo/model_states
timeout 8 rosrun tf tf_echo base_link lidar_link >/tmp/stage9_acceptance_tf.txt 2>&1 || true
if grep -q "Translation:" /tmp/stage9_acceptance_tf.txt 2>/dev/null; then
  mark_pass "TF base_link -> lidar_link"
else
  mark_fail "TF base_link -> lidar_link"
fi

DURATION=30 PATTERN=explore_safe CMD_TOPIC=/cmd_vel_raw "${PKG_PATH}/scripts/run_safe_mapping_driver.sh" corridor_static gmapping_lidar "acceptance_${STAMP}"
SAFE_LOG="${PKG_PATH}/results/stage9/raw/corridor_static/gmapping_lidar/rep_acceptance_${STAMP}/safe_driver_log.csv"
STATUS_FILE="${PKG_PATH}/results/stage9/raw/corridor_static/gmapping_lidar/rep_acceptance_${STAMP}/safe_driver_status.txt"

CMD_Y_CHECK="$(python3 - <<'PY'
import rospy
from geometry_msgs.msg import Twist
rospy.init_node("acceptance_cmd_vel_y_check", anonymous=True, disable_signals=True)
try:
    msg = rospy.wait_for_message("/cmd_vel", Twist, timeout=3.0)
    print(abs(msg.linear.y) <= 1e-6)
except Exception:
    print("False")
PY
)"

if [[ -s "${SAFE_LOG}" ]]; then
  if awk -F, 'NR > 1 && (($3 != "inf" && $3 + 0 < 0.35) || ($4 != "inf" && $4 + 0 < 0.25)) { bad=1 } END { exit bad ? 1 : 0 }' "${SAFE_LOG}"; then
    mark_pass "no_collision"
  else
    mark_fail "no_collision"
  fi
  if awk -F, 'NR > 1 && ($5 + 0) < -0.0001 { bad=1 } END { exit bad ? 1 : 0 }' "${SAFE_LOG}"; then
    mark_pass "cmd_vel linear.x >= 0"
  else
    mark_fail "cmd_vel linear.x >= 0"
  fi
  if awk -F, 'NR > 1 && (($6 + 0) > 0.2501 || ($6 + 0) < -0.2501) { bad=1 } END { exit bad ? 1 : 0 }' "${SAFE_LOG}"; then
    mark_pass "angular.z clamp"
  else
    mark_fail "angular.z clamp"
  fi
  if [[ "${CMD_Y_CHECK}" == "True" ]]; then
    mark_pass "cmd_vel linear.y = 0"
  else
    mark_fail "cmd_vel linear.y = 0"
  fi
else
  mark_fail "safe_driver_log.csv"
fi

if [[ -s "${STATUS_FILE}" ]] && grep -q '^result=PASS' "${STATUS_FILE}"; then
  mark_pass "safe_driver_status"
else
  mark_fail "safe_driver_status"
fi

if rosrun map_server map_saver -f "${MAP_PREFIX}" map:=/map; then
  mark_pass "map_saved"
else
  mark_fail "map_saved"
fi

if [[ -s "${MAP_PREFIX}.yaml" && -s "${MAP_PREFIX}.pgm" ]]; then
  mark_pass "map files exist"
else
  mark_fail "map files exist"
fi

if [[ -f "$(rospack find robot_description)/worlds/test_arena_stage9_fixed.world" ]] && grep -q "missing_wall_stage9_fixed" "$(rospack find robot_description)/worlds/test_arena_stage9_fixed.world"; then
  mark_pass "wall_fixed_world_exists"
else
  mark_fail "wall_fixed_world_exists"
fi

if rosrun slam_benchmark evaluate_map_basic.py "${MAP_PREFIX}.yaml" -o "${TRIAL_DIR}/map_metrics.csv"; then
  mark_pass "map_metrics_created"
else
  mark_fail "map_metrics_created"
fi

if [[ -s "${SAFE_LOG}" && -s "${MAP_PREFIX}.yaml" && -s "${TRIAL_DIR}/map_metrics.csv" ]]; then
  mark_pass "benchmark_artifacts_created"
else
  mark_fail "benchmark_artifacts_created"
fi

if [[ "${FAIL}" -eq 0 ]]; then
  echo "OVERALL: PASS"
else
  echo "OVERALL: FAIL"
fi
exit "${FAIL}"
