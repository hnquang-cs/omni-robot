#!/usr/bin/env bash

# Replay a Stage 7 bag with Gmapping only. Gazebo is not launched.

set -euo pipefail

if [ "$#" -lt 1 ]; then
  echo "Usage: rosrun slam_benchmark replay_with_gmapping.sh <bag_path> [conservative|fast|baseline]"
  exit 1
fi

BAG_PATH="$1"
PROFILE="${2:-conservative}"
EXIT_AFTER_PLAY="${EXIT_AFTER_PLAY:-false}"

if [ ! -f "${BAG_PATH}" ]; then
  echo "FAIL: bag file not found: ${BAG_PATH}"
  exit 1
fi

case "${PROFILE}" in
  conservative)
    LAUNCH_FILE="gmapping_conservative.launch"
    ;;
  fast|faster_update)
    LAUNCH_FILE="gmapping_fast.launch"
    ;;
  baseline)
    LAUNCH_FILE="gmapping_sim.launch"
    ;;
  *)
    echo "FAIL: unknown profile '${PROFILE}'"
    echo "Valid profiles: conservative, fast, baseline"
    exit 1
    ;;
esac

cleanup() {
  if [ -n "${LAUNCH_PID:-}" ] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

echo "Command: roslaunch slam_benchmark ${LAUNCH_FILE} use_sim_time:=true"
roslaunch slam_benchmark "${LAUNCH_FILE}" use_sim_time:=true &
LAUNCH_PID=$!

sleep 3
rosparam set /use_sim_time true
echo "Command: rosbag play --clock ${BAG_PATH}"
rosbag play --clock "${BAG_PATH}"

echo "Replay finished."
echo "Save map with: rosrun slam_benchmark save_map_with_prefix.sh gmapping_${PROFILE}"
if [ "${EXIT_AFTER_PLAY}" = "true" ]; then
  exit 0
fi

echo "Gmapping is still running so the map can be saved. Press Ctrl-C here when done."
wait "${LAUNCH_PID}"
