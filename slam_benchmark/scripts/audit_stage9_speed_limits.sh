#!/usr/bin/env bash

set -euo pipefail

PKG_PATH="$(rospack find slam_benchmark)"
WS_ROOT="$(cd "${PKG_PATH}/../../.." && pwd)"
LOG_DIR="${PKG_PATH}/results/stage9/logs"
LOG_FILE="${LOG_DIR}/audit_stage9_speed_limits.log"

mkdir -p "${LOG_DIR}"
: > "${LOG_FILE}"
exec > >(tee -a "${LOG_FILE}") 2>&1

timestamp() { date -Is; }
log() { echo "[$(timestamp)] $*"; }

SLAM="${WS_ROOT}/src/omni-robot/slam_benchmark"
OMNI="${WS_ROOT}/src/omni-robot/omni_base_controller"

FILES=(
  "${SLAM}/scripts/safe_mapping_driver.py"
  "${SLAM}/scripts/run_safe_mapping_driver.sh"
  "${SLAM}/scripts/create_gmapping_wide_obstacles_map.sh"
  "${SLAM}/scripts/run_stage9_lidar_benchmark.sh"
  "${SLAM}/scripts/run_single_slam_trial.sh"
  "${OMNI}/scripts/gazebo_model_cmd_vel.py"
  "${OMNI}/config/gazebo_model_controller.yaml"
  "${OMNI}/scripts/cmd_vel_forward_only_filter.py"
  "${OMNI}/config/cmd_vel_forward_only_filter.yaml"
  "${SLAM}/launch/slam_lidar_wide_obstacles_full.launch"
  "${SLAM}/config/stage9_speed_profiles.yaml"
)

PATTERN='forward_speed|rotate_speed|max_forward_speed|max_rotate_speed|max_linear_velocity|max_angular_velocity|max_vel_x|max_vel_theta|max_acc_x|max_acc_theta|safety_stop_distance|critical_stop_distance|cmd_vel|cmd_vel_raw|cmd_topic|cmd_vel_topic'

log "Stage 9 speed limit audit"
log "workspace=${WS_ROOT}"
log "log=${LOG_FILE}"

for file in "${FILES[@]}"; do
  if [[ -f "${file}" ]]; then
    log "FILE ${file}"
    grep -nE "${PATTERN}" "${file}" || log "no matching speed/cmd_vel keys"
  else
    log "MISSING ${file}"
  fi
done

log "STATIC SUMMARY"
if grep -q 'cmd_vel_forward_only_filter.launch' "${SLAM}/launch/slam_lidar_wide_obstacles_full.launch"; then
  log "cmd_vel_forward_only_filter_used_in_stage9=true"
  log "safe_mapping_driver publishes /cmd_vel_raw when /cmd_vel_forward_only_filter is running; otherwise /cmd_vel"
else
  log "cmd_vel_forward_only_filter_not_used_in_stage9"
  log "safe_mapping_driver publishes /cmd_vel directly"
fi

python3 - "${SLAM}" "${OMNI}" <<'PY'
import re
import sys
from pathlib import Path

slam = Path(sys.argv[1])
omni = Path(sys.argv[2])

def read(path):
    try:
        return path.read_text()
    except OSError:
        return ""

def find_float(text, key):
    patterns = [
        rf"{re.escape(key)}\s*:\s*([0-9.]+)",
        rf'{re.escape(key)}"\s+value="([0-9.]+)"',
        rf"{re.escape(key)}[^\n]*default=\"([0-9.]+)\"",
    ]
    for pattern in patterns:
        m = re.search(pattern, text)
        if m:
            return float(m.group(1))
    return None

launch = read(slam / "launch" / "slam_lidar_wide_obstacles_full.launch")
controller = read(omni / "config" / "gazebo_model_controller.yaml")
filter_cfg = read(omni / "config" / "cmd_vel_forward_only_filter.yaml")
profiles = read(slam / "config" / "stage9_speed_profiles.yaml")

normal_fwd = find_float(profiles.split("normal:", 1)[-1].split("fast:", 1)[0], "forward_speed")
normal_rot = find_float(profiles.split("normal:", 1)[-1].split("fast:", 1)[0], "rotate_speed")
filter_x = find_float(launch, "max_vel_x") or find_float(filter_cfg, "max_vel_x")
filter_theta = find_float(launch, "max_vel_theta") or find_float(filter_cfg, "max_vel_theta")
controller_x = find_float(controller, "max_linear_velocity")
controller_theta = find_float(launch, "max_angular_velocity") or find_float(controller, "max_angular_velocity")

print(f"normal_profile_forward_speed={normal_fwd}")
print(f"normal_profile_rotate_speed={normal_rot}")
print(f"stage9_filter_max_vel_x={filter_x}")
print(f"stage9_filter_max_vel_theta={filter_theta}")
print(f"gazebo_controller_max_linear_velocity={controller_x}")
print(f"gazebo_controller_max_angular_velocity={controller_theta}")
if normal_fwd is not None and filter_x is not None:
    print(f"filter_clamps_normal_forward={'true' if filter_x < normal_fwd else 'false'}")
if normal_rot is not None and filter_theta is not None:
    print(f"filter_clamps_normal_rotate={'true' if filter_theta < normal_rot else 'false'}")
if normal_fwd is not None and controller_x is not None:
    print(f"controller_clamps_normal_forward={'true' if controller_x < normal_fwd else 'false'}")
if normal_rot is not None and controller_theta is not None:
    print(f"controller_clamps_normal_rotate={'true' if controller_theta < normal_rot else 'false'}")
PY

log "RUNTIME SUMMARY"
if ! rostopic list >/tmp/stage9_audit_topics.txt 2>/tmp/stage9_audit_rostopic.err; then
  log "ROS runtime not reachable; skipped rostopic checks"
  sed -n '1,8p' /tmp/stage9_audit_rostopic.err || true
  log "PASS: audit written to ${LOG_FILE}"
  exit 0
fi

log "rostopic info /cmd_vel"
rostopic info /cmd_vel || true

log "rostopic echo /cmd_vel -n 3 timeout=5s"
timeout 5 rostopic echo /cmd_vel -n 3 || log "WARN: no /cmd_vel samples within timeout"

log "rostopic hz /cmd_vel timeout=6s"
if timeout 6 rostopic hz /cmd_vel >/tmp/stage9_audit_cmd_vel_hz.txt 2>&1; then
  cat /tmp/stage9_audit_cmd_vel_hz.txt
  log "rostopic_hz_cmd_vel=PASS"
else
  cat /tmp/stage9_audit_cmd_vel_hz.txt || true
  if grep -q "average rate" /tmp/stage9_audit_cmd_vel_hz.txt 2>/dev/null; then
    log "rostopic_hz_cmd_vel=PASS samples_collected_before_timeout"
  else
    log "WARN: could not measure /cmd_vel hz within timeout"
  fi
fi

if grep -Fxq /cmd_vel_raw /tmp/stage9_audit_topics.txt; then
  log "rostopic info /cmd_vel_raw"
  rostopic info /cmd_vel_raw || true
else
  log "cmd_vel_raw topic not present at runtime"
fi

log "PASS: audit written to ${LOG_FILE}"
