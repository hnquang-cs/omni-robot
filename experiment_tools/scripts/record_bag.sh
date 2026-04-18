#!/usr/bin/env bash

# Placeholder rosbag recording helper for repeatable thesis experiments.
# TODO:
# - select a scenario name
# - record navigation, perception, and SLAM topics
# - save bags into a timestamped experiment directory

set -euo pipefail

OUTPUT_DIR="${1:-$HOME/bags/omni_thesis}"

echo "[record_bag] Placeholder recorder"
echo "[record_bag] Output directory: ${OUTPUT_DIR}"
echo "[record_bag] TODO: add rosbag record command with the agreed topic list"
