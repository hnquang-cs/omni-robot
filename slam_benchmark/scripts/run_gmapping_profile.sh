#!/usr/bin/env bash

# Launch a tuned Gmapping profile without starting Gazebo.

set -euo pipefail

PROFILE="${1:-conservative}"

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
    echo "Usage: rosrun slam_benchmark run_gmapping_profile.sh [conservative|fast|baseline]"
    exit 1
    ;;
esac

echo "Command: roslaunch slam_benchmark ${LAUNCH_FILE}"
roslaunch slam_benchmark "${LAUNCH_FILE}"
