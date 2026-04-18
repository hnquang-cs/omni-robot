#!/usr/bin/env bash

# Helper wrapper for future repeatable SLAM benchmark runs.
# TODO:
# - select backend (gmapping or hector)
# - launch rosbag playback or simulation
# - collect timing, map, and trajectory outputs

set -euo pipefail

BACKEND="${1:-gmapping}"

echo "[run_benchmark] Placeholder benchmark runner"
echo "[run_benchmark] Selected backend: ${BACKEND}"
echo "[run_benchmark] TODO: wire this script to roslaunch, rosbag, and result export"
