#!/usr/bin/env bash

set -euo pipefail

PATTERN="${1:-corridor}"
RATE="${RATE:-10}"

stop_robot() {
  rostopic pub /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1 >/dev/null 2>&1 || true
}

trap stop_robot EXIT INT TERM

publish_for() {
  local duration="$1"
  local x="$2"
  local y="$3"
  local z="$4"
  echo "cmd_vel ${duration}s: linear.x=${x} linear.y=${y} angular.z=${z}"
  timeout "${duration}" rostopic pub /cmd_vel geometry_msgs/Twist \
    "{linear: {x: ${x}, y: ${y}, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${z}}}" \
    -r "${RATE}" >/dev/null 2>&1 || true
  stop_robot
  sleep 1
}

if ! rostopic list >/dev/null 2>&1; then
  echo "FAIL: ROS master is not reachable"
  exit 1
fi

# Stereo-derived scans have a limited effective FOV. Slow commands keep scan
# overlap higher and make Gmapping/Hector failures easier to interpret.
case "${PATTERN}" in
  corridor)
    publish_for 24 0.12 0.00 0.00
    publish_for 8 0.00 0.00 0.14
    publish_for 22 0.12 0.00 0.00
    publish_for 8 0.00 0.00 -0.14
    publish_for 18 -0.08 0.00 0.00
    ;;
  rectangle)
    publish_for 20 0.12 0.00 0.00
    publish_for 18 0.00 0.08 0.00
    publish_for 16 -0.10 0.00 0.00
    publish_for 18 0.00 -0.08 0.00
    publish_for 10 0.00 0.00 0.16
    publish_for 12 0.10 0.00 0.00
    ;;
  rotate_scan)
    publish_for 12 0.00 0.00 0.16
    publish_for 10 0.08 0.00 0.00
    publish_for 12 0.00 0.00 -0.16
    publish_for 10 -0.08 0.00 0.00
    ;;
  narrow_turn)
    publish_for 16 0.10 0.00 0.00
    publish_for 11 0.02 0.00 0.14
    publish_for 12 0.10 0.00 0.00
    publish_for 11 0.02 0.00 -0.14
    publish_for 14 0.08 0.04 0.00
    publish_for 10 0.00 0.00 0.12
    ;;
  *)
    echo "FAIL: unknown drive pattern '${PATTERN}'"
    echo "Valid patterns: corridor rectangle rotate_scan narrow_turn"
    exit 2
    ;;
esac

stop_robot
echo "PASS: completed drive pattern ${PATTERN}"
