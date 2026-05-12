#!/usr/bin/env bash
# Stage 11: orchestrate one frontier-exploration trial.
#
# A separate monitor node (frontier_exploration_monitor.py) decides when to
# stop. Its verdict is ONE of:
#   DONE_duration         elapsed >= DURATION
#   DONE_coverage_stable  no growth + min search attempts reached
#   ERROR_collision_risk  global_min_range < 0.30 m sustained 2 s
#   ERROR_stuck           displacement < STUCK_DIST sustained STUCK_DURATION s
#                         (only after MIN_DURATION elapses)
#
# DONE_*  -> trial valid (exit 0)
# ERROR_* -> trial fail  (exit 2)
#
# Env vars:
#   DURATION              hard duration cap (sec, default 300)
#   MIN_DURATION          minimum duration before stable / stuck can fire (default 120)
#   GROWTH_WINDOW         growth window (sec, default 30)
#   GROWTH_THRESHOLD      m^2 growth required to keep going (default 1.0)
#   MIN_SEARCH_ATTEMPTS   frontier goals required for DONE_coverage_stable (default 5)
#   COLLISION_RANGE       m, lidar min for collision risk (default 0.30)
#   COLLISION_DURATION    sec sustained for ERROR (default 2.0)
#   STUCK_DIST            m, displacement threshold (default 0.05)
#   STUCK_DURATION        sec without motion (default 30)
#   AUTO_LAUNCH           if "1", roslaunch frontier_exploration_full in background
#   TRIAL_NAME            override trial directory name

set -uo pipefail

DURATION="${DURATION:-300}"
MIN_DURATION="${MIN_DURATION:-120}"
GROWTH_WINDOW="${GROWTH_WINDOW:-30}"
COLLISION_RANGE="${COLLISION_RANGE:-0.30}"
COLLISION_DURATION="${COLLISION_DURATION:-2.0}"
STUCK_DIST="${STUCK_DIST:-0.05}"
STUCK_DURATION="${STUCK_DURATION:-30}"
STUCK_COUNT_THRESHOLD="${STUCK_COUNT_THRESHOLD:-5}"
CONFIRM_NO_FRONTIER="${CONFIRM_NO_FRONTIER:-5}"
MIN_FRONTIER_CELLS="${MIN_FRONTIER_CELLS:-20}"
AUTO_LAUNCH="${AUTO_LAUNCH:-0}"

PKG_PATH="$(rospack find slam_benchmark 2>/dev/null || true)"
if [[ -z "${PKG_PATH}" ]]; then
  PKG_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
TRIAL_NAME="${TRIAL_NAME:-frontier_trial_${STAMP}}"
TRIAL_DIR="${PKG_PATH}/results/stage11_frontier/raw/${TRIAL_NAME}"
LOG_DIR="${PKG_PATH}/results/stage11_frontier/logs"
mkdir -p "${TRIAL_DIR}" "${LOG_DIR}"
TRIAL_LOG="${TRIAL_DIR}/trial.log"
STATE_FILE="${TRIAL_DIR}/monitor_state.json"
MONITOR_LOG="${TRIAL_DIR}/monitor.jsonl"

LAUNCH_PID=""
RECORD_PID=""
MONITOR_PID=""
STOP_REASON=""
REPORT_PRINTED=0

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${TRIAL_LOG}"; }

print_final_report() {
  # Always print the verdict block. Idempotent — safe to call from both the
  # normal flow and the exit trap.
  if [[ "${REPORT_PRINTED}" -eq 1 ]]; then
    return
  fi
  REPORT_PRINTED=1

  # If the monitor wrote a state file, prefer its stop_reason.
  if [[ -z "${STOP_REASON}" && -s "${STATE_FILE}" ]]; then
    STOP_REASON="$(python3 -c "
import json
try:
    print(json.load(open('${STATE_FILE}')).get('stop_reason') or '')
except Exception:
    pass
" 2>/dev/null || true)"
  fi
  if [[ -z "${STOP_REASON}" ]]; then
    STOP_REASON="INTERRUPTED"
  fi

  log "===================="
  log "Stop verdict        : ${STOP_REASON}"
  if [[ -s "${STATE_FILE}" ]]; then
    python3 - "${STATE_FILE}" <<'PY' 2>&1 | tee -a "${TRIAL_LOG}"
import json, sys
try:
    d = json.load(open(sys.argv[1]))
except Exception as e:
    print(f"(state file unreadable: {e})")
    sys.exit(0)
def fmt(v):
    return f"{v:.6f}" if isinstance(v, float) else ("N/A" if v is None else str(v))
keys = [
    "global_explored_ratio",
    "active_explored_ratio",
    "active_unknown_ratio",
    "explored_area_m2",
    "explored_area_growth_m2",
    "active_bbox_growth_m2",
    "frontier_cells",
    "search_attempt_count",
    "global_min_range",
    "stuck_count",
    "no_frontier_for_sec",
    "collision_for_sec",
    "elapsed_sec",
]
for k in keys:
    print(f"{k:24s}: {fmt(d.get(k))}")
PY
  else
    log "(no monitor state file at ${STATE_FILE})"
  fi
  log "===================="
}

# Make the robot actually stop when this script exits.
#
# Even with AUTO_LAUNCH=0 (the user owns roslaunch), we MUST cut the goal
# source — otherwise explore_lite keeps publishing goals to move_base every
# 1/planner_frequency seconds and the robot resumes motion the moment our
# zero /cmd_vel stops.
#
# Sequence (idempotent, defensive):
#   1. rosnode kill any frontier-goal source we know of: /explore and
#      /simple_frontier_detector. Their launch files set respawn=false, so
#      they stay dead until the user re-launches.
#   2. Cancel every active move_base goal (3x to survive a missed message).
#   3. Wait briefly for move_base to settle.
#   4. Spam zero Twist on /cmd_vel AND /cmd_vel_raw at 20 Hz for 3 s, in
#      parallel, so any in-flight planner output is overridden.
GOAL_SOURCE_NODES=("/explore" "/simple_frontier_detector")
safe_stop_robot() {
  log "safe_stop: killing goal-source nodes"
  for nd in "${GOAL_SOURCE_NODES[@]}"; do
    if rosnode list 2>/dev/null | grep -Fxq "${nd}"; then
      log "  rosnode kill ${nd}"
      timeout 3 rosnode kill "${nd}" >/dev/null 2>&1 || true
    fi
  done

  log "safe_stop: cancelling all move_base goals (3x)"
  if rostopic list 2>/dev/null | grep -Fxq "/move_base/cancel"; then
    for _ in 1 2 3; do
      timeout 1 rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}' \
        >/dev/null 2>&1 || true
    done
  fi

  sleep 1   # give move_base a chance to react

  log "safe_stop: spamming zero velocity for 3s on /cmd_vel + /cmd_vel_raw"
  local pids=()
  for t in /cmd_vel /cmd_vel_raw; do
    if rostopic list 2>/dev/null | grep -Fxq "${t}"; then
      ( timeout 3 rostopic pub -r 20 "${t}" geometry_msgs/Twist \
          '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
          >/dev/null 2>&1 || true ) &
      pids+=("$!")
    fi
  done
  if [[ "${#pids[@]}" -gt 0 ]]; then
    wait "${pids[@]}" 2>/dev/null || true
  fi

  log "safe_stop: done"
}

cleanup() {
  log "cleanup: starting"
  if [[ -n "${MONITOR_PID:-}" ]] && kill -0 "${MONITOR_PID}" 2>/dev/null; then
    log "cleanup: SIGINT monitor PID=${MONITOR_PID}"
    kill -INT "${MONITOR_PID}" 2>/dev/null || true
  fi
  if [[ -n "${RECORD_PID:-}" ]] && kill -0 "${RECORD_PID}" 2>/dev/null; then
    log "cleanup: SIGINT rosbag PID=${RECORD_PID}, then SIGTERM 2s later"
    kill -INT "${RECORD_PID}" 2>/dev/null || true
    sleep 2
    kill -TERM "${RECORD_PID}" 2>/dev/null || true
  fi
  log "cleanup: safe_stop_robot"
  safe_stop_robot
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    log "cleanup: SIGINT roslaunch PID=${LAUNCH_PID}"
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
  fi
  log "cleanup: done"
}

trap_exit() {
  local rc=$?
  # Disable traps immediately so that `exit` below doesn't re-enter trap_exit
  # via the EXIT trap, which would run cleanup twice and log "Trial exit"
  # twice.
  trap - EXIT INT TERM
  log "trap_exit captured rc=${rc}"
  cleanup
  print_final_report
  log "Trial exit rc=${rc}"
  exit "${rc}"
}
trap trap_exit EXIT INT TERM

log "Trial dir:              ${TRIAL_DIR}"
log "Duration cap:           ${DURATION}s"
log "Min duration:           ${MIN_DURATION}s"
log "Growth window (report): ${GROWTH_WINDOW}s"
log "Collision:              ${COLLISION_RANGE} m / ${COLLISION_DURATION}s"
log "Stuck window:           ${STUCK_DIST} m / ${STUCK_DURATION}s"
log "Stuck count threshold:  ${STUCK_COUNT_THRESHOLD} ticks"
log "Confirm-no-frontier:    ${CONFIRM_NO_FRONTIER}s"
log "Min frontier cells:     ${MIN_FRONTIER_CELLS}"
log "AUTO_LAUNCH=${AUTO_LAUNCH}"

# ---- 0. Optional auto-launch -----------------------------------------------
if [[ "${AUTO_LAUNCH}" == "1" ]]; then
  log "Launching frontier_exploration_full.launch in background"
  ROSLAUNCH_LOG="${TRIAL_DIR}/roslaunch.log"
  roslaunch slam_benchmark frontier_exploration_full.launch \
      use_rviz:=false gazebo_gui:=false >"${ROSLAUNCH_LOG}" 2>&1 &
  LAUNCH_PID=$!
  log "roslaunch PID=${LAUNCH_PID}, waiting up to 30s for ROS master"
  for _ in $(seq 1 30); do
    if rostopic list >/dev/null 2>&1; then break; fi
    sleep 1
  done
fi

if ! rostopic list >/dev/null 2>&1; then
  log "FAIL  ROS master is not reachable. Start frontier_exploration_full.launch first or set AUTO_LAUNCH=1."
  exit 2
fi

# ---- 1. Wait for required topics (parallel, single deadline) -------------
WAIT_TIMEOUT="${WAIT_TIMEOUT:-180}"
log "Waiting up to ${WAIT_TIMEOUT}s for required topics (parallel)..."
if ! python3 "${PKG_PATH}/scripts/wait_for_topics.py" \
       --timeout "${WAIT_TIMEOUT}" \
       /lidar/scan /odom /map /move_base/status \
       >>"${TRIAL_LOG}" 2>&1; then
  log "FAIL  required topics not ready. Aborting trial."
  exit 1
fi
log "All required topics ready."

# ---- 2. Runtime check ------------------------------------------------------
log "Phase 2: running check_frontier_runtime.sh (capped at 60s)"
RUNTIME_CHECK_RC=0
timeout 60 "${PKG_PATH}/scripts/check_frontier_runtime.sh" \
    >>"${TRIAL_LOG}" 2>&1 || RUNTIME_CHECK_RC=$?
if [[ "${RUNTIME_CHECK_RC}" == "124" ]]; then
  log "WARN  runtime check timed out after 60s (a TF or topic probe hung). Continuing."
elif [[ "${RUNTIME_CHECK_RC}" != "0" ]]; then
  log "WARN  runtime check reported failures rc=${RUNTIME_CHECK_RC}. Continuing."
else
  log "Phase 2 done"
fi

# ---- 3. Start bag recorder (no timeout — monitor decides) -----------------
BAG_BASE="${TRIAL_NAME}"
BAG_PATH="${PKG_PATH}/results/stage11_frontier/bags/${BAG_BASE}.bag"
mkdir -p "$(dirname "${BAG_PATH}")"

REQUIRED_TOPICS=(/map /lidar/scan /odom /tf /tf_static /clock /cmd_vel /move_base/status)
OPTIONAL_TOPICS=(
  /cmd_vel_raw /gazebo/model_states /move_base/goal /move_base_simple/goal
  /move_base/result /move_base/GlobalPlanner/plan
  /move_base/TebLocalPlannerROS/local_plan /move_base/global_costmap/costmap
  /move_base/local_costmap/costmap /explore/frontiers /simple_frontiers
)
ADVERTISED="$(rostopic list 2>/dev/null || true)"
RECORD_TOPICS=()
for t in "${REQUIRED_TOPICS[@]}";  do RECORD_TOPICS+=("${t}"); done
for t in "${OPTIONAL_TOPICS[@]}"; do
  if grep -Fxq "${t}" <<<"${ADVERTISED}"; then
    RECORD_TOPICS+=("${t}")
  fi
done

log "Starting rosbag record -> ${BAG_PATH}"
rosbag record -O "${BAG_PATH}" "${RECORD_TOPICS[@]}" \
  >>"${TRIAL_LOG}" 2>&1 &
RECORD_PID=$!

# ---- 4. Start monitor (decides stop reason) -------------------------------
log "Starting frontier_exploration_monitor"
python3 "${PKG_PATH}/scripts/frontier_exploration_monitor.py" \
  --duration "${DURATION}" \
  --min-duration "${MIN_DURATION}" \
  --growth-window "${GROWTH_WINDOW}" \
  --collision-range "${COLLISION_RANGE}" \
  --collision-duration "${COLLISION_DURATION}" \
  --stuck-dist "${STUCK_DIST}" \
  --stuck-duration "${STUCK_DURATION}" \
  --stuck-count-threshold "${STUCK_COUNT_THRESHOLD}" \
  --confirm-no-frontier "${CONFIRM_NO_FRONTIER}" \
  --min-frontier-cells "${MIN_FRONTIER_CELLS}" \
  --tick-period 2.0 \
  --state-file "${STATE_FILE}" \
  --log-file "${MONITOR_LOG}" \
  >>"${TRIAL_LOG}" 2>&1 &
MONITOR_PID=$!
log "Monitor PID=${MONITOR_PID}"
sleep 3
if ! kill -0 "${MONITOR_PID}" 2>/dev/null; then
  log "FAIL  monitor exited within 3s of start. See trial.log for traceback."
  MONITOR_PID=""
  exit 2
fi

log "Phase 4: waiting on monitor PID=${MONITOR_PID}"
set +e
wait "${MONITOR_PID}"
MONITOR_RC=$?
set -e
MONITOR_PID=""
log "Phase 4 done: monitor exited rc=${MONITOR_RC}"

# ---- 5. Stop bag recorder -------------------------------------------------
log "Phase 5: stopping bag recorder"
if [[ -n "${RECORD_PID:-}" ]] && kill -0 "${RECORD_PID}" 2>/dev/null; then
  log "Stopping rosbag record (SIGINT)"
  kill -INT "${RECORD_PID}" 2>/dev/null || true
  for _ in $(seq 1 15); do
    kill -0 "${RECORD_PID}" 2>/dev/null || break
    sleep 1
  done
  if kill -0 "${RECORD_PID}" 2>/dev/null; then
    kill -TERM "${RECORD_PID}" 2>/dev/null || true
    sleep 2
  fi
fi
RECORD_PID=""

if [[ -s "${BAG_PATH}" ]]; then
  cp -f "${BAG_PATH}" "${TRIAL_DIR}/trial.bag"
  log "Bag copied to ${TRIAL_DIR}/trial.bag"
else
  log "WARN  bag missing or empty at ${BAG_PATH}"
fi

# ---- 6. Print final report (idempotent; trap will skip if already printed)
log "Phase 6: printing final report"
print_final_report

# ---- 7. Save final map ----------------------------------------------------
log "Phase 7: saving final map"
MAP_BASENAME="${TRIAL_NAME}_map"
if "${PKG_PATH}/scripts/save_frontier_map.sh" "${MAP_BASENAME}" >>"${TRIAL_LOG}" 2>&1; then
  MAP_YAML="${PKG_PATH}/results/stage11_frontier/maps/${MAP_BASENAME}.yaml"
  MAP_PGM="${PKG_PATH}/results/stage11_frontier/maps/${MAP_BASENAME}.pgm"
  cp -f "${MAP_YAML}" "${TRIAL_DIR}/map.yaml" 2>/dev/null || true
  cp -f "${MAP_PGM}"  "${TRIAL_DIR}/map.pgm"  2>/dev/null || true
  if [[ -f "${TRIAL_DIR}/map.yaml" ]]; then
    sed -i "s|^image:.*$|image: map.pgm|" "${TRIAL_DIR}/map.yaml" || true
  fi
  log "Map saved to ${TRIAL_DIR}/map.yaml"
else
  log "WARN  save_frontier_map.sh failed; continuing without map"
fi

# ---- 8. Stop the robot definitively (kill goal sources + spam zero) -------
log "Phase 8: safe_stop_robot"
safe_stop_robot

# ---- 9. Evaluate metrics --------------------------------------------------
METRICS_CSV="${TRIAL_DIR}/metrics.csv"
log "Phase 9: computing metrics -> ${METRICS_CSV}"
EVAL_BAG_ARG=()
EVAL_MAP_ARG=()
[[ -s "${TRIAL_DIR}/trial.bag" ]] && EVAL_BAG_ARG=(--bag "${TRIAL_DIR}/trial.bag")
[[ -s "${TRIAL_DIR}/map.yaml" ]] && EVAL_MAP_ARG=(--map-yaml "${TRIAL_DIR}/map.yaml")
python3 "${PKG_PATH}/scripts/evaluate_frontier_exploration.py" \
  "${EVAL_BAG_ARG[@]}" "${EVAL_MAP_ARG[@]}" \
  --output "${METRICS_CSV}" --duration-hint "${DURATION}" \
  >>"${TRIAL_LOG}" 2>&1 || log "WARN  evaluator returned non-zero"

mkdir -p "${PKG_PATH}/results/stage11_frontier/csv"
cp -f "${METRICS_CSV}" \
   "${PKG_PATH}/results/stage11_frontier/csv/frontier_exploration_metrics.csv" \
   2>/dev/null || true

# ---- 10. Compute final verdict --------------------------------------------
log "Phase 10: computing verdict"
log "Trial outputs:"
log "  ${TRIAL_DIR}/trial.bag"
log "  ${TRIAL_DIR}/map.yaml"
log "  ${TRIAL_DIR}/metrics.csv"
log "  ${TRIAL_DIR}/monitor_state.json"
log "  ${TRIAL_DIR}/monitor.jsonl"
log "  ${TRIAL_DIR}/trial.log"

case "${STOP_REASON}" in
  DONE_duration|DONE_coverage_stable)
    log "VERDICT: TRIAL VALID (${STOP_REASON})"
    EXIT_RC=0
    ;;
  ERROR_collision_risk|ERROR_stuck)
    log "VERDICT: TRIAL FAILED (${STOP_REASON})"
    EXIT_RC=2
    ;;
  *)
    log "VERDICT: TRIAL INCONCLUSIVE (stop_reason='${STOP_REASON}', monitor_rc=${MONITOR_RC})"
    EXIT_RC=1
    ;;
esac

exit "${EXIT_RC}"
