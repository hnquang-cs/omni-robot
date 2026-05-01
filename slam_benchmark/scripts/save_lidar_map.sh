#!/usr/bin/env bash

set -euo pipefail

PREFIX="${1:-lidar_gmapping_$(date +%Y%m%d_%H%M%S)}"
PKG_PATH="$(rospack find slam_benchmark)"
OUT_DIR="${PKG_PATH}/results/stage9/maps"
mkdir -p "${OUT_DIR}"
OUT_PREFIX="${OUT_DIR}/${PREFIX}"

if ! timeout 10 rostopic echo -n 1 /map >/tmp/stage9_map_check.txt 2>&1; then
  echo "FAIL: /map has no message. Start SLAM and drive the robot before saving."
  exit 1
fi

rosrun map_server map_saver -f "${OUT_PREFIX}"

if [[ -s "${OUT_PREFIX}.yaml" && -s "${OUT_PREFIX}.pgm" ]]; then
  echo "PASS: saved ${OUT_PREFIX}.yaml"
  echo "PASS: saved ${OUT_PREFIX}.pgm"
else
  echo "FAIL: map files were not created for prefix ${OUT_PREFIX}"
  exit 1
fi
