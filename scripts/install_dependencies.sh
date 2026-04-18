#!/usr/bin/env bash

# Install or verify the ROS Noetic packages used by this thesis repository.

set -euo pipefail

REQUIRED_PACKAGES=(
  ros-noetic-navigation
  ros-noetic-slam-gmapping
  ros-noetic-hector-slam
  ros-noetic-robot-localization
  ros-noetic-teb-local-planner
  ros-noetic-gazebo-ros
  ros-noetic-pointcloud-to-laserscan
  ros-noetic-stereo-image-proc
  ros-noetic-depth-image-proc
  ros-noetic-image-proc
  ros-noetic-image-view
  ros-noetic-tf
  ros-noetic-xacro
  ros-noetic-joint-state-publisher
  ros-noetic-robot-state-publisher
)

missing_packages=()

for pkg in "${REQUIRED_PACKAGES[@]}"; do
  if dpkg -s "$pkg" >/dev/null 2>&1; then
    echo "[install_dependencies] Installed: $pkg"
  else
    echo "[install_dependencies] Missing:   $pkg"
    missing_packages+=("$pkg")
  fi
done

if [ "${#missing_packages[@]}" -eq 0 ]; then
  echo "[install_dependencies] All required ROS packages are already installed."
  exit 0
fi

echo
echo "[install_dependencies] Installing missing packages with apt..."
sudo apt update
sudo apt install -y "${missing_packages[@]}"

echo
echo "[install_dependencies] Done."
echo "[install_dependencies] Re-run ./scripts/check_env.sh to verify the environment."
