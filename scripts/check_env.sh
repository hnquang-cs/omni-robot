#!/usr/bin/env bash

set -u

# Keep rospack cache in a writable temp directory during checks.
export ROS_HOME="${ROS_HOME:-/tmp/ros_check_env}"
mkdir -p "$ROS_HOME"

PACKAGE_LABELS=(
  "ros-noetic-navigation"
  "ros-noetic-slam-gmapping"
  "ros-noetic-hector-slam"
  "ros-noetic-robot-localization"
  "ros-noetic-teb-local-planner"
  "ros-noetic-gazebo-ros"
  "ros-noetic-pointcloud-to-laserscan"
  "ros-noetic-stereo-image-proc"
  "ros-noetic-depth-image-proc"
  "ros-noetic-image-proc"
  "ros-noetic-image-view"
  "ros-noetic-tf"
  "ros-noetic-xacro"
  "ros-noetic-joint-state-publisher"
  "ros-noetic-robot-state-publisher"
)

PACKAGE_NAMES=(
  "move_base"
  "slam_gmapping"
  "hector_mapping"
  "robot_localization"
  "teb_local_planner"
  "gazebo_ros"
  "pointcloud_to_laserscan"
  "stereo_image_proc"
  "depth_image_proc"
  "image_proc"
  "image_view"
  "tf"
  "xacro"
  "joint_state_publisher"
  "robot_state_publisher"
)

pass_count=0
fail_count=0

pass() {
  echo "PASS: $1"
  pass_count=$((pass_count + 1))
}

fail() {
  echo "FAIL: $1"
  fail_count=$((fail_count + 1))
}

check_file() {
  local label="$1"
  local path="$2"

  if [ -f "$path" ]; then
    pass "$label ($path)"
  else
    fail "$label ($path)"
  fi
}

check_command() {
  local cmd="$1"

  if command -v "$cmd" >/dev/null 2>&1; then
    pass "command $cmd -> $(command -v "$cmd")"
  else
    fail "command $cmd"
  fi
}

check_package() {
  local label="$1"
  local pkg="$2"

  if rospack find "$pkg" >/dev/null 2>&1; then
    pass "package $label via $pkg -> $(rospack find "$pkg")"
  else
    fail "package $label via $pkg"
  fi
}

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/noetic/setup.bash
  set -u
  pass "source /opt/ros/noetic/setup.bash"
else
  fail "source /opt/ros/noetic/setup.bash"
fi

if [ -n "${ROS_DISTRO:-}" ]; then
  pass "ROS_DISTRO=$ROS_DISTRO"
else
  fail "ROS_DISTRO is empty"
fi

if [ -n "${ROS_PACKAGE_PATH:-}" ]; then
  pass "ROS_PACKAGE_PATH is set"
else
  fail "ROS_PACKAGE_PATH is empty"
fi

check_file "workspace marker" "$HOME/catkin_ws/.catkin_workspace"
check_file "workspace source" "$HOME/catkin_ws/src/CMakeLists.txt"

for cmd in roscore rviz gazebo catkin_make rospack roslaunch; do
  check_command "$cmd"
done

for i in "${!PACKAGE_NAMES[@]}"; do
  check_package "${PACKAGE_LABELS[$i]}" "${PACKAGE_NAMES[$i]}"
done

echo
echo "Summary: PASS=$pass_count FAIL=$fail_count"

if [ "$fail_count" -gt 0 ]; then
  exit 1
fi
