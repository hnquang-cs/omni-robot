#!/bin/bash

set -e

echo "=== Update system ==="
sudo apt update

echo "=== Install basic tools ==="
sudo apt install -y curl gnupg2 lsb-release ca-certificates

echo "=== Add ROS repository ==="
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list'

echo "=== Add ROS key ==="
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

echo "=== Install ROS Noetic ==="
sudo apt update
sudo apt install -y ros-noetic-desktop-full

echo "=== Setup ROS environment ==="
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash

echo "=== Install rosdep ==="
sudo apt install -y python3-rosdep
sudo rosdep init || true
rosdep update

echo "=== Create catkin workspace ==="
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

echo "=== Install Gazebo ==="
sudo apt install -y gazebo11 libgazebo11-dev

echo "=== Install Navigation Stack ==="
sudo apt install -y \
  ros-noetic-navigation \
  ros-noetic-move-base \
  ros-noetic-amcl \
  ros-noetic-map-server \
  ros-noetic-global-planner \
  ros-noetic-dwa-local-planner \
  ros-noetic-teb-local-planner \
  ros-noetic-gmapping

echo "=== DONE ==="
echo "👉 Open new terminal or run: source ~/.bashrc"