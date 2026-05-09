#!/usr/bin/env bash

set -euo pipefail

SLAM_PKG="$(rospack find slam_benchmark)"
NAV_PKG="$(rospack find nav_bringup)"
MAP_DIR="${SLAM_PKG}/results/stage9/maps"
NAV_MAP_DIR="${NAV_PKG}/maps"
STAMP="$(date +%Y%m%d_%H%M%S)"
PREFIX="${PREFIX:-lidar_baseline_wide_${STAMP}}"
OUT_PREFIX="${MAP_DIR}/${PREFIX}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
LINEAR="${LINEAR:-0.15}"
LATERAL="${LATERAL:-0.12}"
ANGULAR="${ANGULAR:-0.18}"
INSTALL_NAV_MAP="${INSTALL_NAV_MAP:-true}"

mkdir -p "${MAP_DIR}" "${NAV_MAP_DIR}"

publish_cmd() {
  local vx="$1"
  local vy="$2"
  local wz="$3"
  local duration="$4"
  timeout "${duration}" rostopic pub -r 10 "${CMD_TOPIC}" geometry_msgs/Twist \
    "linear:
  x: ${vx}
  y: ${vy}
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: ${wz}" >/dev/null || true
}

stop_robot() {
  rostopic pub -1 "${CMD_TOPIC}" geometry_msgs/Twist \
    "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" >/dev/null 2>&1 || true
}

trap stop_robot EXIT INT TERM

echo "Stage 10 wide LiDAR remap workflow"
echo "Expected launch in another terminal:"
echo "  roslaunch slam_benchmark slam_lidar_wide_full.launch"
echo "Gmapping wide bounds: xmin/ymin=-15.0, xmax/ymax=15.0, delta=0.05"

for topic in /lidar/scan /odom /map; do
  timeout 10 rostopic echo -n 1 "${topic}" >/tmp/stage10_wide_topic.txt 2>&1 || {
    echo "FAIL: ${topic} has no message. Start slam_lidar_wide_full.launch first."
    exit 1
  }
  echo "PASS: ${topic} message received"
done

echo "Driving pattern: rotate at spawn, scan rear wall, corridor sweep, lateral passes, final rotation."
publish_cmd 0.0 0.0 "${ANGULAR}" 22
publish_cmd "${LINEAR}" 0.0 0.0 18
publish_cmd 0.0 0.0 "${ANGULAR}" 8
publish_cmd "${LINEAR}" 0.0 0.0 18
publish_cmd 0.0 "${LATERAL}" 0.0 8
publish_cmd 0.0 "-${LATERAL}" 0.0 16
publish_cmd 0.0 "${LATERAL}" 0.0 8
publish_cmd "-${LINEAR}" 0.0 0.0 18
publish_cmd 0.0 0.0 "-${ANGULAR}" 28
stop_robot

echo "Waiting for final /map update..."
sleep 2
timeout 10 rostopic echo -n 1 /map >/tmp/stage10_wide_map_final.txt 2>&1 || {
  echo "FAIL: no final /map message"
  exit 1
}

rosrun map_server map_saver -f "${OUT_PREFIX}"

if [[ ! -s "${OUT_PREFIX}.yaml" || ! -s "${OUT_PREFIX}.pgm" ]]; then
  echo "FAIL: map_saver did not create ${OUT_PREFIX}.yaml/.pgm"
  exit 1
fi

echo "PASS: saved ${OUT_PREFIX}.yaml"
echo "PASS: saved ${OUT_PREFIX}.pgm"

if [[ "${INSTALL_NAV_MAP}" == "true" ]]; then
  cp "${OUT_PREFIX}.pgm" "${NAV_MAP_DIR}/lidar_baseline_wide.pgm"
  cp "${OUT_PREFIX}.yaml" "${NAV_MAP_DIR}/lidar_baseline_wide.yaml"
  sed -i 's#^image:.*#image: lidar_baseline_wide.pgm#' "${NAV_MAP_DIR}/lidar_baseline_wide.yaml"
  echo "PASS: installed ${NAV_MAP_DIR}/lidar_baseline_wide.yaml"
  echo "PASS: installed ${NAV_MAP_DIR}/lidar_baseline_wide.pgm"
fi
