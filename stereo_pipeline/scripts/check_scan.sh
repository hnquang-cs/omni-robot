#!/usr/bin/env bash

# Check that /scan exists, publishes, and provides sample data.

set -euo pipefail

if ! rostopic list >/dev/null 2>&1; then
  echo "FAIL: ROS master is not reachable"
  exit 1
fi

if rostopic list 2>/dev/null | grep -Fxq /scan; then
  echo "PASS: /scan exists"
else
  echo "FAIL: /scan does not exist"
  exit 1
fi

echo "Checking /scan rate..."
if timeout 5 rostopic hz /scan; then
  echo "PASS: /scan has a measurable publish rate"
else
  echo "FAIL: Unable to measure /scan publish rate"
  exit 1
fi

echo "Checking one /scan sample..."
if timeout 5 rostopic echo -n 1 /scan; then
  echo "PASS: /scan sample received"
else
  echo "FAIL: Unable to read a /scan sample"
  exit 1
fi
