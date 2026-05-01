#!/usr/bin/env bash

# Save the current /map to slam_benchmark/maps as .pgm + .yaml.

set -euo pipefail

MAP_NAME="${1:-gmapping_test}"
PKG_PATH="$(rospack find slam_benchmark)"
OUTPUT_PREFIX="${PKG_PATH}/maps/${MAP_NAME}"

if ! rostopic list >/dev/null 2>&1; then
  echo "FAIL: ROS master is not reachable"
  exit 1
fi

if ! rostopic list | grep -Fxq /map; then
  echo "FAIL: /map does not exist. Start SLAM and wait for a map before saving."
  exit 1
fi

if ! timeout 5 rostopic echo -n 1 /map >/dev/null 2>&1; then
  echo "FAIL: /map exists but no map message was received within 5 seconds"
  exit 1
fi

mkdir -p "${PKG_PATH}/maps"
echo "Saving map to ${OUTPUT_PREFIX}.pgm and ${OUTPUT_PREFIX}.yaml"
rosrun map_server map_saver -f "${OUTPUT_PREFIX}"
echo "PASS: map saved"
