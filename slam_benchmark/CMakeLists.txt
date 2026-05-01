cmake_minimum_required(VERSION 3.0.2)
project(slam_benchmark)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/check_hector_topics.sh
  scripts/check_slam_topics.sh
  scripts/check_stage7_system.sh
  scripts/compare_map_csv.py
  scripts/aggregate_stage8_results.py
  scripts/check_stage8_dependencies.sh
  scripts/check_lidar_stage9.sh
  scripts/compute_ate_rpe.sh
  scripts/convert_trajectory_to_tum.py
  scripts/drive_mapping_pattern.sh
  scripts/drive_lidar_mapping_pattern.sh
  scripts/evaluate_map_basic.py
  scripts/extract_gazebo_ground_truth.py
  scripts/extract_slam_tf_trajectory.py
  scripts/generate_latex_tables.py
  scripts/play_bag_gmapping.sh
  scripts/plot_stage8_results.py
  scripts/record_stage7_bag.sh
  scripts/record_lidar_bag.sh
  scripts/record_slam_bag.sh
  scripts/replay_with_gmapping.sh
  scripts/replay_with_hector.sh
  scripts/run_benchmark.sh
  scripts/run_gmapping_profile.sh
  scripts/run_hector.sh
  scripts/run_single_slam_trial.sh
  scripts/run_stage8_benchmark.sh
  scripts/run_stage9_lidar_benchmark.sh
  scripts/save_lidar_map.sh
  scripts/save_map_with_prefix.sh
  scripts/save_map.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
