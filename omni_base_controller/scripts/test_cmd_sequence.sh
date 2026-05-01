#!/usr/bin/env bash

# Publish a short sequence of body-twist commands for Stage 4 testing.

set -euo pipefail

CMD_TOPIC="${1:-/cmd_vel}"
PUBLISH_RATE="${PUBLISH_RATE:-10}"

cleanup() {
  if [ -n "${PUB_PID:-}" ]; then
    kill "${PUB_PID}" >/dev/null 2>&1 || true
    wait "${PUB_PID}" 2>/dev/null || true
  fi
}

publish_phase() {
  local label="$1"
  local duration="$2"
  local vx="$3"
  local vy="$4"
  local wz="$5"

  echo "[test_cmd_sequence] ${label}"
  rostopic pub -r "${PUBLISH_RATE}" "${CMD_TOPIC}" geometry_msgs/Twist \
    "{linear: {x: ${vx}, y: ${vy}, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${wz}}}" \
    >/dev/null &
  PUB_PID=$!
  sleep "${duration}"
  cleanup
  unset PUB_PID
}

publish_stop_once() {
  rostopic pub -1 "${CMD_TOPIC}" geometry_msgs/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null
}

trap cleanup EXIT

echo "[test_cmd_sequence] Using topic: ${CMD_TOPIC}"

publish_phase "1/5 Forward motion test" 3 0.25 0.0 0.0
publish_stop_once
sleep 1

publish_phase "2/5 Left strafe test" 3 0.0 0.20 0.0
publish_stop_once
sleep 1

publish_phase "3/5 Right strafe test" 3 0.0 -0.20 0.0
publish_stop_once
sleep 1

publish_phase "4/5 In-place rotation test" 3 0.0 0.0 0.60
publish_stop_once
sleep 1

echo "[test_cmd_sequence] 5/5 Final stop"
publish_stop_once
echo "[test_cmd_sequence] Sequence complete"
