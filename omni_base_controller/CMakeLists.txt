cmake_minimum_required(VERSION 3.0.2)
project(omni_base_controller)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/cmd_vel_forward_only_filter.py
  scripts/cmd_vel_limiter.py
  scripts/discrete_45_motion_controller.py
  scripts/gazebo_model_cmd_vel.py
  scripts/gazebo_truth_odom.py
  scripts/monitor_discrete_motion.py
  scripts/omni_kinematics.py
  scripts/test_discrete_45_primitives.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/test_cmd_sequence.sh
  scripts/test_discrete_45_primitives.sh
  scripts/test_forward_only_filter.sh
  scripts/debug_discrete45_odom.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
