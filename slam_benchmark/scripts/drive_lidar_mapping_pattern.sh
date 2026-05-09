#!/usr/bin/env bash

set -euo pipefail

PATTERN="${1:-rectangle}"
LINEAR="${LINEAR:-0.15}"
ANGULAR="${ANGULAR:-0.15}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"

publish_cmd() {
  local vx="$1"
  local wz="$2"
  local duration="$3"
  timeout "${duration}" rostopic pub -r 10 "${CMD_TOPIC}" geometry_msgs/Twist \
    "linear:
  x: ${vx}
  y: 0.0
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
  echo "Stop command sent"
}

trap stop_robot EXIT INT TERM

echo "Running LiDAR mapping pattern: ${PATTERN}"
case "${PATTERN}" in
  forward)
    publish_cmd "${LINEAR}" 0.0 20
    ;;
  rotate_slow)
    publish_cmd 0.0 "${ANGULAR}" 35
    ;;
  rectangle)
    for _ in 1 2 3 4; do
      echo "forward segment"
      publish_cmd "${LINEAR}" 0.0 16
      echo "corner rotation"
      publish_cmd 0.0 "${ANGULAR}" 11
    done
    ;;
  corridor)
    publish_cmd "${LINEAR}" 0.0 18
    publish_cmd 0.0 "${ANGULAR}" 10
    publish_cmd "${LINEAR}" 0.0 18
    publish_cmd 0.0 "-${ANGULAR}" 10
    publish_cmd "${LINEAR}" 0.0 18
    ;;
  narrow_turn)
    for _ in 1 2 3; do
      echo "short forward segment"
      publish_cmd "${LINEAR}" 0.0 10
      echo "slow narrow turn"
      publish_cmd 0.0 "${ANGULAR}" 14
      echo "short recovery segment"
      publish_cmd "${LINEAR}" 0.0 7
      publish_cmd 0.0 "-${ANGULAR}" 8
    done
    ;;
  *)
    echo "FAIL: unknown pattern '${PATTERN}'. Use forward, rotate_slow, rectangle, corridor, or narrow_turn."
    exit 2
    ;;
esac
