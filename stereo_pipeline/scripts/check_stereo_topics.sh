#!/usr/bin/env bash

# Check that the simulated stereo topics and processing outputs exist.

set -euo pipefail

if ! rostopic list >/dev/null 2>&1; then
  echo "FAIL: ROS master is not reachable"
  exit 1
fi

required_topics=(
  /stereo/left/image_raw
  /stereo/right/image_raw
  /stereo/left/camera_info
  /stereo/right/camera_info
  /stereo/disparity
  /stereo/points2
)

topic_list="$(rostopic list 2>/dev/null || true)"
pass_count=0
fail_count=0

for topic in "${required_topics[@]}"; do
  if grep -Fxq "${topic}" <<<"${topic_list}"; then
    echo "PASS: ${topic}"
    pass_count=$((pass_count + 1))
  else
    echo "FAIL: ${topic}"
    fail_count=$((fail_count + 1))
  fi
done

echo "Summary: PASS=${pass_count} FAIL=${fail_count}"

if [ "${fail_count}" -gt 0 ]; then
  exit 1
fi
