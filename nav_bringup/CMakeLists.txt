cmake_minimum_required(VERSION 3.0.2)
project(nav_bringup)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
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

install(DIRECTORY launch config maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/aggregate_navigation_results.py
  scripts/analyze_cmd_vel_stage10.py
  scripts/check_navigation_stage10.sh
  scripts/check_cmd_vel_forward_only.sh
  scripts/check_goal_on_map.py
  scripts/debug_teb_motion_stage10.sh
  scripts/diagnose_goal_failure_stage10.sh
  scripts/evaluate_navigation_trial.py
  scripts/record_navigation_bag.sh
  scripts/run_nav_trial.sh
  scripts/send_nav_goal.py
  scripts/test_goal_1_1_stage10.sh
  scripts/test_goal_forward_only_stage10.sh
  scripts/test_smooth_motion_goal.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
