#!/usr/bin/env bash

set -euo pipefail

PKG_PATH="$(rospack find slam_benchmark)"
LOG_DIR="${PKG_PATH}/results/stage9/logs"
mkdir -p "${LOG_DIR}"

publish_zero() {
  for topic in /cmd_vel_raw /cmd_vel; do
    rostopic pub -1 "${topic}" geometry_msgs/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
      >/dev/null 2>&1 || true
  done
}

reset_robot_pose() {
  if rosservice list 2>/dev/null | grep -Fxq /gazebo/set_model_state; then
    rosservice call /gazebo/set_model_state \
      "model_state:
         model_name: 'omni_robot'
         pose:
           position: {x: 0.0, y: 0.0, z: 0.05}
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
         twist:
           linear:  {x: 0.0, y: 0.0, z: 0.0}
           angular: {x: 0.0, y: 0.0, z: 0.0}
         reference_frame: 'world'" >/dev/null 2>&1 || true
    sleep 1
  fi
}

cleanup() {
  publish_zero
}
trap cleanup EXIT INT TERM

echo "[INFO] Requires a running Stage 9 stack:"
echo "[INFO]   roslaunch slam_benchmark slam_lidar_wide_obstacles_full.launch"

if ! rostopic list >/dev/null 2>&1; then
  echo "FAIL: ROS master is not reachable"
  exit 1
fi

if ! rostopic list 2>/dev/null | grep -Fxq /lidar/scan; then
  echo "FAIL: /lidar/scan missing. Start slam_lidar_wide_obstacles_full.launch first."
  exit 1
fi

"${PKG_PATH}/scripts/audit_stage9_speed_limits.sh"

run_profile_check() {
  local profile="$1"
  local rep="$2"
  local output="${LOG_DIR}/stage9_runtime_speed_check_${profile}.csv"
  local checker_pid="" driver_status=0 checker_status=0

  echo "[INFO] running speed profile=${profile} rep=${rep}"
  publish_zero
  reset_robot_pose
  "${PKG_PATH}/scripts/check_stage9_runtime_speed.py" \
    --duration 20 \
    --profile "${profile}" \
    --output "${output}" &
  checker_pid=$!

  sleep 1
  DURATION=20 "${PKG_PATH}/scripts/run_safe_mapping_driver.sh" \
    wide_obstacles gmapping_lidar "${rep}" "${profile}" || driver_status=$?

  wait "${checker_pid}" || checker_status=$?
  publish_zero

  if [[ "${driver_status}" -ne 0 ]]; then
    echo "FAIL: safe driver failed for profile=${profile} status=${driver_status}"
    return "${driver_status}"
  fi
  if [[ "${checker_status}" -ne 0 ]]; then
    echo "FAIL: runtime speed check failed for profile=${profile}; output=${output}"
    return "${checker_status}"
  fi

  python3 - "${output}" "${profile}" <<'PY'
import csv
import sys

path, profile = sys.argv[1], sys.argv[2]
with open(path, newline="") as f:
    row = next(csv.DictReader(f))
print(
    f"[INFO] {profile} max_linear_x={row['max_linear_x']} "
    f"max_angular_z_abs={row['max_angular_z_abs']} "
    f"any_negative_x={row['any_negative_x']} any_nonzero_y={row['any_nonzero_y']}"
)
PY
}

run_profile_check normal speed_test
run_profile_check fast speed_test_fast

echo "PASS: Stage 9 speed profiles normal and fast met runtime thresholds"
