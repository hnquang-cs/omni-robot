#!/usr/bin/env bash

set -uo pipefail

FAIL=0

check_cmd() {
  local cmd="$1"
  if command -v "${cmd}" >/dev/null 2>&1; then
    echo "PASS: command ${cmd}"
  else
    echo "FAIL: missing command ${cmd}"
    FAIL=1
  fi
}

check_ros_pkg() {
  local pkg="$1"
  if rospack find "${pkg}" >/dev/null 2>&1; then
    echo "PASS: ROS package ${pkg}"
  else
    echo "FAIL: missing ROS package ${pkg}"
    FAIL=1
  fi
}

check_python_module() {
  local module="$1"
  if python3 - <<PY >/dev/null 2>&1
import ${module}
PY
  then
    echo "PASS: Python module ${module}"
  else
    echo "WARN: missing Python module ${module}"
  fi
}

echo "Stage 8 dependency check"
echo "Workspace package: $(rospack find slam_benchmark 2>/dev/null || pwd)"

check_cmd roscore
check_cmd roslaunch
check_cmd rostopic
check_cmd rosbag
check_cmd python3

check_ros_pkg gmapping
if rospack find hector_mapping >/dev/null 2>&1 || rospack find hector_slam >/dev/null 2>&1; then
  echo "PASS: ROS package hector_mapping/hector_slam"
else
  echo "FAIL: missing ROS package hector_mapping or hector_slam"
  FAIL=1
fi
check_ros_pkg map_server

check_python_module numpy
check_python_module matplotlib
if python3 - <<'PY' >/dev/null 2>&1
import yaml
PY
then
  echo "PASS: Python module yaml/PyYAML"
else
  echo "WARN: missing Python module yaml/PyYAML"
fi

if command -v evo_ape >/dev/null 2>&1 && command -v evo_rpe >/dev/null 2>&1; then
  echo "PASS: evo command line tools"
else
  echo "WARN: evo is not installed; ATE/RPE scripts will write N/A metrics and instructions"
fi

cat <<'EOF'

Suggested install commands if needed:
  sudo apt install ros-noetic-slam-gmapping ros-noetic-hector-slam ros-noetic-map-server
  pip3 install evo numpy matplotlib pyyaml
EOF

if [[ "${FAIL}" -eq 0 ]]; then
  echo "OVERALL: PASS"
else
  echo "OVERALL: FAIL"
fi
exit "${FAIL}"
