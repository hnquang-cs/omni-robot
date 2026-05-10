#!/usr/bin/env bash
# Stage 9 LiDAR SLAM benchmark: Gmapping vs Hector.
# Usage:
#   run_stage9_lidar_benchmark.sh [--yes] [--dry-run] [--launch-system] [--attach]
#   run_stage9_lidar_benchmark.sh --single <scenario> <algorithm> <rep>

set -euo pipefail

ORIGINAL_CMD="$0 $*"
YES=false
DRY_RUN=false
SINGLE=false
LAUNCH_SYSTEM=false
FORCE_ATTACH=false
SCENARIOS=(corridor_static open_room_obstacles narrow_turn)
ALGORITHMS=(gmapping_lidar hector_lidar)
REPETITIONS=(1 2 3)
MODEL_NAME="${MODEL_NAME:-auto}"
GAZEBO_MODEL_NAME="${GAZEBO_MODEL_NAME:-omni_robot}"
USE_CMD_VEL_SAFETY_FILTER="${USE_CMD_VEL_SAFETY_FILTER:-true}"
# Spawn pose (matches gazebo_lidar_stage9_fixed.launch defaults)
SPAWN_X="${SPAWN_X:-0.0}"
SPAWN_Y="${SPAWN_Y:-0.0}"
SPAWN_Z="${SPAWN_Z:-0.02}"
SPAWN_YAW="${SPAWN_YAW:-1.5708}"    # 90 deg = facing +Y
WIDE_OBSTACLES_SPAWN_X="${WIDE_OBSTACLES_SPAWN_X:-0.0}"
WIDE_OBSTACLES_SPAWN_Y="${WIDE_OBSTACLES_SPAWN_Y:-0.0}"
WIDE_OBSTACLES_SPAWN_Z="${WIDE_OBSTACLES_SPAWN_Z:-0.05}"
WIDE_OBSTACLES_SPAWN_YAW="${WIDE_OBSTACLES_SPAWN_YAW:-0.0}"  # facing open lobby through +X corridor

while [[ $# -gt 0 ]]; do
  case "$1" in
    --yes|-y) YES=true; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    --launch-system) LAUNCH_SYSTEM=true; shift ;;
    --attach) FORCE_ATTACH=true; shift ;;
    --single)
      if [[ $# -lt 4 || -z "${2:-}" || -z "${3:-}" || -z "${4:-}" ]]; then
        echo "FAIL: --single requires <scenario> <algorithm> <rep>"
        exit 2
      fi
      SINGLE=true
      SCENARIOS=("$2")
      ALGORITHMS=("$3")
      REPETITIONS=("$4")
      shift 4
      ;;
    *)
      echo "Unknown option: $1"
      exit 2
      ;;
  esac
done

PKG_PATH="$(rospack find slam_benchmark)"
BASE="${PKG_PATH}/results/stage9"
RAW="${BASE}/raw"
CSV="${BASE}/csv"
LATEX="${BASE}/latex"
MARKDOWN="${BASE}/markdown"
PLOTS="${BASE}/plots"
MAPS="${BASE}/maps"
mkdir -p "${RAW}" "${CSV}" "${LATEX}" "${MARKDOWN}" "${PLOTS}" "${MAPS}"

TOPICS=(/lidar/scan /odom /map /tf /tf_static /clock /cmd_vel /gazebo/model_states)
RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_LOG="${BASE}/stage9_run_${RUN_ID}.log"

if [[ "${SINGLE}" == true ]]; then
  echo "[INFO] mode=single scenario=${SCENARIOS[0]} algorithm=${ALGORITHMS[0]} rep=${REPETITIONS[0]}" | tee -a "${RUN_LOG}"
fi

if [[ "${YES}" != true && "${SINGLE}" != true && "${DRY_RUN}" != true ]]; then
  read -r -p "Run Stage 9 safe LiDAR benchmark now? Type YES: " answer
  [[ "${answer}" == "YES" ]] || { echo "Aborted."; exit 0; }
fi

# ── helpers ──────────────────────────────────────────────────────────────────

timestamp() { date -Is; }

log_line() {
  local log_file="$1"; shift
  echo "[$(timestamp)] $*" | tee -a "${log_file}"
}

scenario_duration() {
  case "$1" in
    corridor_static) echo 45 ;;
    open_room_obstacles) echo 120 ;;
    narrow_turn) echo 105 ;;
    wide_obstacles) echo 300 ;;
    *) echo 90 ;;
  esac
}

scenario_pattern() {
  case "$1" in
    corridor_static) echo corridor_safe ;;
    open_room_obstacles) echo explore_safe ;;
    narrow_turn) echo explore_safe ;;
    wide_obstacles) echo wide_obstacles_safe ;;
    *) echo corridor_safe ;;
  esac
}

scenario_full_launch() {
  case "$1" in
    wide_obstacles) echo slam_lidar_wide_obstacles_full.launch ;;
    *) echo slam_lidar_stage9_fixed_full.launch ;;
  esac
}

# Fields: display_name, map_tf_frame, base_frame, use_gmapping
algorithm_fields() {
  case "$1" in
    gmapping_lidar) echo "Gmapping,map,base_footprint,true" ;;
    hector_lidar)   echo "Hector,hector_map,base_footprint,false" ;;
    *) echo "Unknown,map,base_footprint,true" ;;
  esac
}

append_status() {
  local path="$1" success="$2" runtime="$3" notes="$4"
  {
    echo "success,runtime_sec,notes"
    echo "${success},${runtime},${notes}"
  } > "${path}"
}

write_na_trajectory_metrics() {
  local path="$1" reason="${2:-na}"
  {
    echo "metric,value"
    echo "ate_rmse,N/A"
    echo "ate_mean,N/A"
    echo "ate_max,N/A"
    echo "ate_std,N/A"
    echo "rpe_rmse,N/A"
    echo "rpe_mean,N/A"
    echo "rpe_max,N/A"
    echo "rpe_std,N/A"
    echo "trajectory_metrics_source,none"
    echo "na_reason,${reason}"
  } > "${path}"
}

write_na_map_metrics() {
  local path="$1" reason="${2:-na}"
  {
    echo "map_yaml,image,width,height,resolution,area_m2,occupied_cell_count,free_cell_count,unknown_cell_count,total_cell_count,occupied_ratio,free_ratio,unknown_ratio,occupied_iou,na_reason"
    echo "N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A,${reason}"
  } > "${path}"
}

stop_pid() {
  local pid="${1:-}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill -INT "${pid}" >/dev/null 2>&1 || true
    sleep 2
    kill -TERM "${pid}" >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
  fi
}

publish_zero() {
  for _ in 1 2 3 4 5; do
    rostopic pub -1 /cmd_vel geometry_msgs/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
      >/dev/null 2>&1 || true
    rostopic pub -1 /cmd_vel_raw geometry_msgs/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
      >/dev/null 2>&1 || true
  done
}

wait_for_topic_message() {
  local topic="$1" timeout_sec="$2" log_file="$3"
  log_line "${log_file}" "checking topic ${topic} timeout=${timeout_sec}s"
  if timeout "${timeout_sec}" rostopic echo -n 1 "${topic}" >/tmp/stage9_topic_check.txt 2>&1; then
    log_line "${log_file}" "topic_check ${topic}=PASS"
    return 0
  fi
  log_line "${log_file}" "topic_check ${topic}=FAIL"
  sed -n '1,8p' /tmp/stage9_topic_check.txt | tee -a "${log_file}" >/dev/null || true
  return 1
}

ros_topic_list_has() {
  local topic="$1"
  rostopic list 2>/dev/null | grep -Fxq "${topic}"
}

system_topics_present() {
  ros_topic_list_has /lidar/scan && ros_topic_list_has /odom && ros_topic_list_has /map
}

wait_for_system_topics_present() {
  local timeout_sec="$1" log_file="$2"
  log_line "${log_file}" "waiting_for_system_topics timeout=${timeout_sec}s"
  if timeout "${timeout_sec}" bash -c 'until rostopic list 2>/dev/null | grep -Fxq /lidar/scan && rostopic list 2>/dev/null | grep -Fxq /odom && rostopic list 2>/dev/null | grep -Fxq /map && rostopic list 2>/dev/null | grep -Fxq /gazebo/model_states; do sleep 2; done'; then
    log_line "${log_file}" "waiting_for_system_topics=PASS"
    return 0
  fi
  log_line "${log_file}" "waiting_for_system_topics=FAIL"
  return 1
}

select_system_mode() {
  if [[ "${LAUNCH_SYSTEM}" == true ]]; then
    echo "launch"
  elif [[ "${FORCE_ATTACH}" == true ]] || system_topics_present; then
    echo "attach"
  else
    echo "missing"
  fi
}

select_driver_cmd_topic() {
  if rosnode list 2>/dev/null | grep -Fxq /cmd_vel_forward_only_filter; then
    echo /cmd_vel_raw
  else
    echo /cmd_vel
  fi
}

front_min_range() {
  python3 - "$1" <<'PY'
import math, sys
import rospy
from sensor_msgs.msg import LaserScan
topic = sys.argv[1]
rospy.init_node("stage9_front_range_check", anonymous=True, disable_signals=True)
msg = rospy.wait_for_message(topic, LaserScan, timeout=8.0)
front_angle = math.radians(20.0)
front = []
angle = msg.angle_min
for value in msg.ranges:
    if math.isfinite(value) and msg.range_min <= value <= msg.range_max and abs(angle) <= front_angle:
        front.append(value)
    angle += msg.angle_increment
print(f"{(min(front) if front else float('inf')):.4f}")
PY
}

archive_existing_trial_files() {
  local trial_dir="$1"
  [[ -d "${trial_dir}" ]] || return 0
  shopt -s nullglob
  local files=("${trial_dir}"/*)
  shopt -u nullglob
  [[ "${#files[@]}" -gt 0 ]] || return 0
  local archive="${trial_dir}/archive_${RUN_ID}"
  mkdir -p "${archive}"
  for path in "${files[@]}"; do
    [[ "$(basename "${path}")" == archive_* ]] && continue
    mv "${path}" "${archive}/"
  done
}

check_hector_available() {
  if ! rospack find hector_mapping >/dev/null 2>&1; then
    echo "WARN: hector_mapping package not found."
    echo "      Install: sudo apt install ros-noetic-hector-slam"
    return 1
  fi
  return 0
}

# ── robot pose reset ──────────────────────────────────────────────────────────

reset_robot_pose() {
  local log_file="$1" scenario="${2:-}"
  local qz qw reset_x reset_y reset_z reset_yaw
  if [[ "${scenario}" == "wide_obstacles" ]]; then
    reset_x="${WIDE_OBSTACLES_SPAWN_X}"
    reset_y="${WIDE_OBSTACLES_SPAWN_Y}"
    reset_z="${WIDE_OBSTACLES_SPAWN_Z}"
    reset_yaw="${WIDE_OBSTACLES_SPAWN_YAW}"
  else
    reset_x="${SPAWN_X}"
    reset_y="${SPAWN_Y}"
    reset_z="${SPAWN_Z}"
    reset_yaw="${SPAWN_YAW}"
  fi
  # yaw → quaternion (rotation around Z)
  qz="$(python3 -c "import math; print(f'{math.sin(${reset_yaw}/2):.6f}')")"
  qw="$(python3 -c "import math; print(f'{math.cos(${reset_yaw}/2):.6f}')")"

  log_line "${log_file}" "reset_robot: model=${GAZEBO_MODEL_NAME} x=${reset_x} y=${reset_y} z=${reset_z} yaw=${reset_yaw}"
  if rosservice call /gazebo/set_model_state \
    "model_state:
       model_name: '${GAZEBO_MODEL_NAME}'
       pose:
         position: {x: ${reset_x}, y: ${reset_y}, z: ${reset_z}}
         orientation: {x: 0.0, y: 0.0, z: ${qz}, w: ${qw}}
       twist:
         linear:  {x: 0.0, y: 0.0, z: 0.0}
         angular: {x: 0.0, y: 0.0, z: 0.0}
       reference_frame: 'world'" \
    >/dev/null 2>&1; then
    log_line "${log_file}" "reset_robot=PASS"
    sleep 1.5   # let Gazebo settle before bag recording
  else
    log_line "${log_file}" "WARN: reset_robot=FAIL (gazebo service unavailable — robot NOT reset)"
  fi
}

# ── SLAM node validator ───────────────────────────────────────────────────────

check_slam_node() {
  local algorithm="$1" log_file="$2"
  local gmapping_running hector_running
  gmapping_running="$(rosnode list 2>/dev/null | grep -c slam_gmapping || true)"
  hector_running="$(rosnode list 2>/dev/null | grep -c hector_mapping || true)"

  case "${algorithm}" in
    gmapping_lidar)
      if [[ "${hector_running}" -gt 0 ]]; then
        log_line "${log_file}" "WARN: hector_mapping is running while algorithm=gmapping_lidar — /map TF conflict possible"
      fi
      if [[ "${gmapping_running}" -eq 0 ]]; then
        log_line "${log_file}" "WARN: slam_gmapping node not found — /map may not be published"
        echo ""
        echo "  ┌─ WARN ─────────────────────────────────────────────────────────┐"
        echo "  │ gmapping not running. Start it:                                │"
        echo "  │   roslaunch slam_benchmark slam_lidar_wide_safe_full.launch    │"
        echo "  └────────────────────────────────────────────────────────────────┘"
        echo ""
      fi
      ;;
    hector_lidar)
      if [[ "${gmapping_running}" -gt 0 ]]; then
        log_line "${log_file}" "WARN: slam_gmapping is running — kill before hector trial"
        echo ""
        echo "  ┌─ WARN ─────────────────────────────────────────────────────────┐"
        echo "  │ gmapping is STILL running. Kill it first:                      │"
        echo "  │   rosnode kill /slam_gmapping                                  │"
        echo "  │ Then start hector:                                             │"
        echo "  │   roslaunch slam_benchmark hector_lidar.launch \               │"
        echo "  │       map_frame:=hector_map map_topic:=/map                    │"
        echo "  └────────────────────────────────────────────────────────────────┘"
        echo ""
      fi
      if [[ "${hector_running}" -eq 0 ]]; then
        log_line "${log_file}" "WARN: hector_mapping node not found — start hector_lidar.launch"
        echo ""
        echo "  ┌─ WARN ─────────────────────────────────────────────────────────┐"
        echo "  │ hector_mapping not running. Start it:                          │"
        echo "  │   roslaunch slam_benchmark hector_lidar.launch \               │"
        echo "  │       map_frame:=hector_map map_topic:=/map                    │"
        echo "  └────────────────────────────────────────────────────────────────┘"
        echo ""
      fi
      ;;
  esac
}

# Extract estimated trajectory with algorithm-aware frame fallback
extract_estimated() {
  local bag="$1" map_frame="$2" base_frame="$3" algorithm="$4" out="$5" log="$6"
  local extract_log="${log%.log}_tf_extract.log"

  # Build comma-separated fallback child list
  local child_frames="${base_frame},base_link,base_footprint"

  rosrun slam_benchmark extract_slam_tf_trajectory.py \
    "${bag}" "${map_frame}" "${child_frames}" "${out}" \
    --algorithm "${algorithm}" \
    --log "${extract_log}" \
    --notes-file "${out%.tum}_notes.txt" \
    >>"${log}" 2>&1 && {
      log_line "${log}" "estimated_frame=${map_frame}->*"
      return 0
    }

  log_line "${log}" "WARN: could not extract estimated trajectory for ${map_frame}->${child_frames}"
  return 1
}

# ── main trial function ───────────────────────────────────────────────────────

run_trial() {
  local scenario="$1" algorithm="$2" rep="$3"
  local slam_name map_frame base_frame use_gmapping fields
  fields="$(algorithm_fields "${algorithm}")"
  IFS=',' read -r slam_name map_frame base_frame use_gmapping <<< "${fields}"
  if [[ "${slam_name}" == "Unknown" ]]; then
    echo "FAIL: unknown algorithm ${algorithm}"
    return 2
  fi

  local trial_dir="${RAW}/${scenario}/${algorithm}/rep_${rep}"
  archive_existing_trial_files "${trial_dir}"
  mkdir -p "${trial_dir}"
  local trial_log="${trial_dir}/trial.log"
  : > "${trial_log}"

  local duration pattern start end runtime notes success system_mode
  local full_launch
  duration="$(scenario_duration "${scenario}")"
  pattern="$(scenario_pattern "${scenario}")"
  full_launch="$(scenario_full_launch "${scenario}")"
  start="$(date +%s)"
  system_mode="$(select_system_mode)"

  log_line "${trial_log}" "command_received=${ORIGINAL_CMD}"
  log_line "${trial_log}" "cwd=$(pwd)"
  log_line "${trial_log}" "ROS_MASTER_URI=${ROS_MASTER_URI:-}"
  log_line "${trial_log}" "mode=$([[ "${SINGLE}" == true ]] && echo single || echo batch)"
  log_line "${trial_log}" "system_mode=${system_mode}"
  log_line "${trial_log}" "scenario=${scenario}"
  log_line "${trial_log}" "algorithm=${algorithm}"
  log_line "${trial_log}" "repetition=${rep}"
  log_line "${trial_log}" "output_dir=${trial_dir}"
  log_line "${trial_log}" "map_frame=${map_frame}"
  log_line "${trial_log}" "base_frame=${base_frame}"
  log_line "${trial_log}" "model_name=${MODEL_NAME}"
  log_line "${trial_log}" "full_launch=${full_launch}"
  echo "=== ${scenario}/${algorithm}/rep_${rep} ===" | tee -a "${RUN_LOG}"

  # ── dry run ──────────────────────────────────────────────────────────────
  if [[ "${DRY_RUN}" == true ]]; then
    append_status "${trial_dir}/runtime_status.csv" "false" "0" "dry_run_not_executed"
    write_na_trajectory_metrics "${trial_dir}/trajectory_metrics.csv" "dry_run"
    write_na_map_metrics "${trial_dir}/map_metrics.csv" "dry_run"
    return 0
  fi

  # ── hector dependency check ──────────────────────────────────────────────
  if [[ "${algorithm}" == "hector_lidar" ]]; then
    if ! check_hector_available; then
      log_line "${trial_log}" "FAIL: dependency_missing hector_mapping not installed"
      append_status "${trial_dir}/runtime_status.csv" "false" "0" "dependency_missing:hector_mapping"
      write_na_trajectory_metrics "${trial_dir}/trajectory_metrics.csv" "hector_not_installed"
      write_na_map_metrics "${trial_dir}/map_metrics.csv" "hector_not_installed"
      return 1
    fi
  fi

  # ── system check ────────────────────────────────────────────────────────
  if [[ "${system_mode}" == "missing" ]]; then
    log_line "${trial_log}" "FAIL: no running Stage 9 system detected."
    log_line "${trial_log}" "  Start: roslaunch slam_benchmark ${full_launch}"
    append_status "${trial_dir}/runtime_status.csv" "false" "0" "missing_running_system"
    write_na_trajectory_metrics "${trial_dir}/trajectory_metrics.csv" "missing_running_system"
    write_na_map_metrics "${trial_dir}/map_metrics.csv" "missing_running_system"
    publish_zero
    return 1
  fi

  # ── SLAM node validator (attach mode) ────────────────────────────────────
  if [[ "${system_mode}" == "attach" ]]; then
    check_slam_node "${algorithm}" "${trial_log}"
  fi

  local launch_pid="" hector_pid="" bag_pid="" safe_driver_pid=""
  local driver_status=0 activity_status=0

  cleanup_trial() {
    publish_zero
    stop_pid "${safe_driver_pid}"
    stop_pid "${bag_pid}"
    if [[ "${system_mode}" == "launch" ]]; then
      stop_pid "${hector_pid}"
      stop_pid "${launch_pid}"
    fi
  }
  trap cleanup_trial EXIT INT TERM

  # ── launch system if needed ──────────────────────────────────────────────
  if [[ "${system_mode}" == "launch" ]]; then
    log_line "${trial_log}" "step_start launch_system"
    roslaunch slam_benchmark "${full_launch}" \
      use_rviz:=false gazebo_gui:=false use_gmapping:="${use_gmapping}" \
      use_cmd_vel_safety_filter:="${USE_CMD_VEL_SAFETY_FILTER}" \
      map_frame:="${map_frame}" model_name:="${GAZEBO_MODEL_NAME}" \
      >"${trial_dir}/roslaunch.log" 2>&1 &
    launch_pid=$!
    log_line "${trial_log}" "launch_pid=${launch_pid}"
    if [[ "${algorithm}" == "hector_lidar" ]]; then
      roslaunch slam_benchmark hector_lidar.launch \
        map_frame:=hector_map map_topic:=/map base_frame:=base_footprint \
        >"${trial_dir}/hector.log" 2>&1 &
      hector_pid=$!
      log_line "${trial_log}" "hector_pid=${hector_pid}"
    fi
    wait_for_system_topics_present 90 "${trial_log}" || true
    log_line "${trial_log}" "step_end launch_system"
  else
    log_line "${trial_log}" "attach_mode=PASS no_new_gazebo_or_slam_launched"

    # For hector in attach mode: check /map topic is alive (may be from hector already)
    if [[ "${algorithm}" == "hector_lidar" ]]; then
      log_line "${trial_log}" "hector_attach: expecting hector_mapping already running"
      if ! rostopic list 2>/dev/null | grep -Fxq /map; then
        log_line "${trial_log}" "WARN: /map not present — hector may not be running"
      fi
    fi
  fi

  # ── topic preflight ──────────────────────────────────────────────────────
  log_line "${trial_log}" "step_start topic_checks"
  local topic_failed=0
  wait_for_topic_message /lidar/scan 10 "${trial_log}" || topic_failed=1
  wait_for_topic_message /odom 10 "${trial_log}" || topic_failed=1
  wait_for_topic_message /map 15 "${trial_log}" || topic_failed=1
  wait_for_topic_message /gazebo/model_states 10 "${trial_log}" || topic_failed=1
  if rostopic pub -1 /cmd_vel geometry_msgs/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
      >/dev/null 2>&1; then
    log_line "${trial_log}" "cmd_vel_publish_check=PASS"
  else
    log_line "${trial_log}" "cmd_vel_publish_check=FAIL"
    topic_failed=1
  fi
  log_line "${trial_log}" "step_end topic_checks"

  if [[ "${topic_failed}" -ne 0 ]]; then
    append_status "${trial_dir}/runtime_status.csv" "false" "0" "missing_required_topic"
    write_na_trajectory_metrics "${trial_dir}/trajectory_metrics.csv" "missing_required_topic"
    write_na_map_metrics "${trial_dir}/map_metrics.csv" "missing_required_topic"
    cleanup_trial; trap - EXIT INT TERM; return 1
  fi

  # ── robot pose reset ─────────────────────────────────────────────────────
  # Always reset to spawn position before each trial so every rep starts from
  # the same place regardless of where the previous trial ended.
  log_line "${trial_log}" "step_start robot_reset"
  reset_robot_pose "${trial_log}" "${scenario}"
  log_line "${trial_log}" "step_end robot_reset"

  # ── spawn safety check ───────────────────────────────────────────────────
  local front_min="inf"
  if front_min="$(front_min_range /lidar/scan 2>>"${trial_log}")"; then
    log_line "${trial_log}" "spawn_front_min=${front_min}"
  else
    log_line "${trial_log}" "FAIL: could not sample /lidar/scan for spawn safety"
    append_status "${trial_dir}/runtime_status.csv" "false" "0" "spawn_scan_unavailable"
    write_na_trajectory_metrics "${trial_dir}/trajectory_metrics.csv" "spawn_scan_unavailable"
    write_na_map_metrics "${trial_dir}/map_metrics.csv" "spawn_scan_unavailable"
    cleanup_trial; trap - EXIT INT TERM; return 1
  fi
  if awk -v value="${front_min}" 'BEGIN { exit !(value <= 0.70) }'; then
    log_line "${trial_log}" "FAIL: spawn_too_close_to_obstacle front_min=${front_min}"
    append_status "${trial_dir}/runtime_status.csv" "false" "0" "spawn_too_close_to_obstacle"
    write_na_trajectory_metrics "${trial_dir}/trajectory_metrics.csv" "spawn_too_close_to_obstacle"
    write_na_map_metrics "${trial_dir}/map_metrics.csv" "spawn_too_close_to_obstacle"
    cleanup_trial; trap - EXIT INT TERM; return 1
  fi

  # ── rosbag record ────────────────────────────────────────────────────────
  log_line "${trial_log}" "step_start rosbag_record"
  rosbag record __name:=stage9_rosbag_recorder \
    -O "${trial_dir}/trial.bag" "${TOPICS[@]}" >>"${trial_log}" 2>&1 &
  bag_pid=$!
  log_line "${trial_log}" "rosbag_pid=${bag_pid}"
  sleep 2

  # ── safe driver ──────────────────────────────────────────────────────────
  local cmd_topic
  cmd_topic="$(select_driver_cmd_topic)"
  log_line "${trial_log}" "cmd_topic=${cmd_topic}"
  (
    set -o pipefail
    DURATION="${duration}" PATTERN="${pattern}" CMD_TOPIC="${cmd_topic}" \
      "${PKG_PATH}/scripts/run_safe_mapping_driver.sh" "${scenario}" "${algorithm}" "${rep}" \
      2>&1 | tee -a "${trial_log}"
  ) &
  safe_driver_pid=$!
  log_line "${trial_log}" "safe_driver_pid=${safe_driver_pid}"

  log_line "${trial_log}" "step_start cmd_vel_activity_check"
  if rosrun slam_benchmark check_cmd_vel_activity.py \
      --duration 10 --csv "${trial_dir}/cmd_vel_activity.csv" >>"${trial_log}" 2>&1; then
    log_line "${trial_log}" "cmd_vel_activity=PASS"
    activity_status=0
  else
    log_line "${trial_log}" "cmd_vel_activity=FAIL"
    activity_status=1
    stop_pid "${safe_driver_pid}"; safe_driver_pid=""
  fi
  log_line "${trial_log}" "step_end cmd_vel_activity_check"

  if [[ -n "${safe_driver_pid}" ]]; then
    wait "${safe_driver_pid}" || driver_status=$?
    safe_driver_pid=""
  else
    driver_status=1
  fi
  log_line "${trial_log}" "safe_driver_exit=${driver_status}"
  local safe_status_file="${trial_dir}/safe_driver_status.txt"
  local stop_reason="" driver_elapsed="" global_explored_ratio="" active_explored_ratio=""
  local active_unknown_ratio="" explored_area_m2="" explored_area_growth_m2="" active_bbox_growth_m2=""
  if [[ -s "${safe_status_file}" ]]; then
    stop_reason="$(awk -F= '$1=="stop_reason"{print $2}' "${safe_status_file}" | tail -n1)"
    [[ -n "${stop_reason}" ]] || stop_reason="$(awk -F= '$1=="reason"{print $2}' "${safe_status_file}" | tail -n1)"
    driver_elapsed="$(awk -F= '$1=="elapsed"{print $2}' "${safe_status_file}" | tail -n1)"
    global_explored_ratio="$(awk -F= '$1=="global_explored_ratio"{print $2}' "${safe_status_file}" | tail -n1)"
    active_explored_ratio="$(awk -F= '$1=="active_explored_ratio"{print $2}' "${safe_status_file}" | tail -n1)"
    active_unknown_ratio="$(awk -F= '$1=="active_unknown_ratio"{print $2}' "${safe_status_file}" | tail -n1)"
    explored_area_m2="$(awk -F= '$1=="explored_area_m2"{print $2}' "${safe_status_file}" | tail -n1)"
    explored_area_growth_m2="$(awk -F= '$1=="explored_area_growth_m2"{print $2}' "${safe_status_file}" | tail -n1)"
    active_bbox_growth_m2="$(awk -F= '$1=="active_bbox_growth_m2"{print $2}' "${safe_status_file}" | tail -n1)"
    log_line "${trial_log}" "stop_reason=${stop_reason:-unknown}"
    log_line "${trial_log}" "elapsed=${driver_elapsed:-unknown}"
    log_line "${trial_log}" "global_explored_ratio=${global_explored_ratio:-unknown}"
    log_line "${trial_log}" "active_explored_ratio=${active_explored_ratio:-unknown}"
    log_line "${trial_log}" "active_unknown_ratio=${active_unknown_ratio:-unknown}"
    log_line "${trial_log}" "explored_area_m2=${explored_area_m2:-unknown}"
    log_line "${trial_log}" "explored_area_growth_m2=${explored_area_growth_m2:-unknown}"
    log_line "${trial_log}" "active_bbox_growth_m2=${active_bbox_growth_m2:-unknown}"
  else
    log_line "${trial_log}" "stop_reason=missing_safe_driver_status"
  fi

  # ── stop & save bag ──────────────────────────────────────────────────────
  publish_zero; sleep 2
  stop_pid "${bag_pid}"; bag_pid=""
  if [[ -s "${trial_dir}/trial.bag" ]]; then
    log_line "${trial_log}" "rosbag_record=PASS"
  else
    log_line "${trial_log}" "rosbag_record=FAIL missing_or_empty_bag"
  fi
  log_line "${trial_log}" "step_end rosbag_record"

  # ── map save ─────────────────────────────────────────────────────────────
  log_line "${trial_log}" "step_start map_save"
  local map_topic="/map"
  if timeout 20 rostopic echo -n 1 "${map_topic}" >/dev/null 2>&1; then
    if rosrun map_server map_saver -f "${trial_dir}/map" "map:=${map_topic}" \
        >>"${trial_log}" 2>&1; then
      log_line "${trial_log}" "map_save=PASS"
      local map_stamp; map_stamp="$(date +%Y%m%d_%H%M%S)"
      cp -f "${trial_dir}/map.yaml" "${MAPS}/${scenario}_${algorithm}_rep${rep}_${map_stamp}.yaml" 2>/dev/null || true
      cp -f "${trial_dir}/map.pgm"  "${MAPS}/${scenario}_${algorithm}_rep${rep}_${map_stamp}.pgm"  2>/dev/null || true
    else
      log_line "${trial_log}" "map_save=FAIL map_saver_failed"
    fi
  else
    log_line "${trial_log}" "map_save=FAIL no_map_message_on_${map_topic}"
  fi
  log_line "${trial_log}" "step_end map_save"

  # ── stop SLAM if launched ────────────────────────────────────────────────
  if [[ "${system_mode}" == "launch" ]]; then
    stop_pid "${hector_pid}"; hector_pid=""
    stop_pid "${launch_pid}"; launch_pid=""
  fi

  end="$(date +%s)"
  runtime="$((end - start))"

  # ── trajectory extraction ────────────────────────────────────────────────
  if [[ -s "${trial_dir}/trial.bag" ]]; then
    log_line "${trial_log}" "step_start trajectory_extraction"
    rosrun slam_benchmark extract_gazebo_ground_truth.py \
      "${trial_dir}/trial.bag" "${MODEL_NAME}" "${trial_dir}/groundtruth.tum" \
      --log "${trial_dir}/trajectory_extract.log" \
      >>"${trial_log}" 2>&1 || true
    extract_estimated \
      "${trial_dir}/trial.bag" "${map_frame}" "${base_frame}" \
      "${algorithm}" \
      "${trial_dir}/estimated.tum" "${trial_log}" || true
    log_line "${trial_log}" "step_end trajectory_extraction"
  else
    log_line "${trial_log}" "WARN: trial.bag missing or empty; trajectory extraction skipped"
  fi

  # ── trajectory metrics (evo + Python fallback) ───────────────────────────
  log_line "${trial_log}" "step_start trajectory_metrics"
  if [[ -s "${trial_dir}/groundtruth.tum" && -s "${trial_dir}/estimated.tum" ]]; then
    rosrun slam_benchmark compute_ate_rpe.sh \
      "${trial_dir}/groundtruth.tum" \
      "${trial_dir}/estimated.tum" \
      "${trial_dir}" >>"${trial_log}" 2>&1 || true
  else
    write_na_trajectory_metrics "${trial_dir}/trajectory_metrics.csv" \
      "missing_trajectories:gt=$(test -s "${trial_dir}/groundtruth.tum" && echo yes || echo no) est=$(test -s "${trial_dir}/estimated.tum" && echo yes || echo no)"
    log_line "${trial_log}" "trajectory_metrics=N/A missing_tum_files"
  fi
  log_line "${trial_log}" "step_end trajectory_metrics"

  # ── map metrics ──────────────────────────────────────────────────────────
  log_line "${trial_log}" "step_start map_metrics"
  rosrun slam_benchmark evaluate_map_basic.py \
    "${trial_dir}/map.yaml" -o "${trial_dir}/map_metrics.csv" \
    >>"${trial_log}" 2>&1 || true
  rosrun slam_benchmark evaluate_map_coverage.py \
    "${trial_dir}/map.yaml" -o "${trial_dir}/map_coverage_metrics.csv" \
    >>"${trial_log}" 2>&1 || true
  # evaluate_map_basic.py always writes the file (N/A if map missing)
  if [[ ! -s "${trial_dir}/map_metrics.csv" ]]; then
    write_na_map_metrics "${trial_dir}/map_metrics.csv" "evaluate_map_script_failed"
    log_line "${trial_log}" "map_metrics=FAIL script_did_not_write_file"
  fi
  log_line "${trial_log}" "step_end map_metrics"

  # ── success determination ─────────────────────────────────────────────────
  success="false"
  notes=""
  [[ "${driver_status}" -eq 0 ]] || notes+="safe_driver_failed;"
  [[ "${activity_status}" -eq 0 ]] || notes+="cmd_vel_inactive;"

  if [[ -s "${trial_dir}/safe_driver_status.txt" ]]; then
    local reason
    reason="${stop_reason}"
    [[ -n "${reason}" ]] || reason="$(awk -F= '$1=="reason"{print $2}' "${trial_dir}/safe_driver_status.txt" | tail -n1)"
    if [[ -n "${reason}" && "${reason}" != DONE_duration && "${reason}" != DONE_coverage_stable ]]; then
      notes+="${reason};"
    fi
  else
    notes+="missing_safe_driver_status;"
  fi

  [[ -s "${trial_dir}/trial.bag" ]] || notes+="missing_trial_bag;"
  [[ -s "${trial_dir}/map.yaml" && -s "${trial_dir}/map.pgm" ]] || notes+="missing_map;"
  [[ -s "${trial_dir}/groundtruth.tum" ]] || notes+="missing_groundtruth_tum;"
  [[ -s "${trial_dir}/estimated.tum" ]] || notes+="missing_estimated_tum;"
  [[ -s "${trial_dir}/trajectory_metrics.csv" ]] || notes+="missing_trajectory_metrics;"
  [[ -s "${trial_dir}/map_metrics.csv" ]] || notes+="missing_map_metrics;"
  [[ -s "${trial_dir}/cmd_vel_activity.csv" ]] || notes+="missing_cmd_vel_activity;"

  # Append odom-fallback note if present
  if [[ -f "${trial_dir}/estimated_notes.txt" ]]; then
    local extra_notes
    extra_notes="$(cat "${trial_dir}/estimated_notes.txt" 2>/dev/null || true)"
    [[ -z "${extra_notes}" ]] || notes+="${extra_notes};"
  fi

  # Determine success: all mandatory artifacts must exist; driver must not have failed hard
  local mandatory_ok=true
  for f in trial.bag map.yaml map.pgm groundtruth.tum estimated.tum trajectory_metrics.csv map_metrics.csv; do
    [[ -s "${trial_dir}/${f}" ]] || { mandatory_ok=false; break; }
  done

  if [[ "${mandatory_ok}" == true && "${driver_status}" -eq 0 ]]; then
    success="true"
    [[ -z "${notes}" ]] && notes="complete" || notes="${notes%;}"
  else
    [[ -z "${notes}" ]] && notes="failed" || notes="${notes%;}"
  fi

  append_status "${trial_dir}/runtime_status.csv" "${success}" "${runtime}" "${notes}"
  log_line "${trial_log}" "trial result success=${success} runtime=${runtime}s notes=${notes}"
  echo "trial result success=${success} runtime=${runtime}s notes=${notes}" | tee -a "${RUN_LOG}"

  cleanup_trial
  trap - EXIT INT TERM
  [[ "${success}" == "true" ]]
}

# ── run all trials ────────────────────────────────────────────────────────────

for scenario in "${SCENARIOS[@]}"; do
  for algorithm in "${ALGORITHMS[@]}"; do
    for rep in "${REPETITIONS[@]}"; do
      run_trial "${scenario}" "${algorithm}" "${rep}" || \
        echo "WARN: trial ${scenario}/${algorithm}/rep_${rep} returned non-zero" | tee -a "${RUN_LOG}"
    done
  done
done

rosrun slam_benchmark aggregate_stage9_results.py
rosrun slam_benchmark plot_stage9_results.py 2>/dev/null || true

echo "Stage 9 results: ${BASE}"
echo "Raw CSV:  ${CSV}/slam_comparison_raw.csv"
echo "Mean CSV: ${CSV}/slam_comparison_mean.csv"
