#!/usr/bin/env bash

# Save an occupancy grid map into slam_benchmark/maps with a timestamped prefix.

set -euo pipefail

PREFIX="${1:-gmapping_stage7}"
MAP_TOPIC="${2:-/map}"
PKG_PATH="$(rospack find slam_benchmark)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_PREFIX="${PKG_PATH}/maps/${PREFIX}_${STAMP}"

if ! rostopic list >/dev/null 2>&1; then
  echo "FAIL: ROS master is not reachable"
  exit 1
fi

if ! rostopic list | grep -Fxq "${MAP_TOPIC}"; then
  echo "FAIL: ${MAP_TOPIC} does not exist. Start SLAM and wait for a map before saving."
  exit 1
fi

if ! timeout 8 rostopic echo -n 1 "${MAP_TOPIC}" >/dev/null 2>&1; then
  echo "FAIL: ${MAP_TOPIC} exists but no map message was received within 8 seconds"
  exit 1
fi

mkdir -p "${PKG_PATH}/maps"
echo "Saving ${MAP_TOPIC} to ${OUTPUT_PREFIX}.pgm and ${OUTPUT_PREFIX}.yaml"
rosrun map_server map_saver map:="${MAP_TOPIC}" -f "${OUTPUT_PREFIX}"
echo "PASS: map saved"
