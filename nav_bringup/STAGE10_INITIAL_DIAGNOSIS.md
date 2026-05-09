# Stage 10 Initial Diagnosis

Generated: 2026-05-01T17:56:17+07:00

## Build
Built target robot_description_xacro_generated_to_devel_space_
Base path: /home/hnquang/catkin_ws
Source space: /home/hnquang/catkin_ws/src
Build space: /home/hnquang/catkin_ws/build
Devel space: /home/hnquang/catkin_ws/devel
Install space: /home/hnquang/catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/home/hnquang/catkin_ws/build"
####
####
#### Running command: "make -j6 -l6" in "/home/hnquang/catkin_ws/build"
####

## Rospack
$ rospack find nav_bringup
/home/hnquang/catkin_ws/src/omni-robot/nav_bringup

$ rospack find map_server
/opt/ros/noetic/share/map_server

$ rospack find amcl
/opt/ros/noetic/share/amcl

$ rospack find move_base
/opt/ros/noetic/share/move_base

$ rospack find teb_local_planner
/opt/ros/noetic/share/teb_local_planner

$ rospack find costmap_2d
/opt/ros/noetic/share/costmap_2d

$ rospack find global_planner
/opt/ros/noetic/share/global_planner

$ rospack find navfn
/opt/ros/noetic/share/navfn

## Stage 9 map candidates
2026-05-01 14:35:46.8256175480 src/omni-robot/slam_benchmark/results/stage9/maps/corridor_static_gmapping_lidar_rep1_20260501_143342.yaml
2026-05-01 14:35:46.8256175480 src/omni-robot/slam_benchmark/results/stage9/maps/corridor_static_gmapping_lidar_rep1_20260501_143342.pgm
2026-05-01 14:33:13.7844357740 src/omni-robot/slam_benchmark/results/stage9/maps/lidar_gmapping_test.yaml
2026-05-01 14:33:13.7844357740 src/omni-robot/slam_benchmark/results/stage9/maps/lidar_gmapping_test.pgm
2026-04-28 23:10:40.3995738140 src/omni-robot/slam_benchmark/maps/gmapping_stage7_test_20260428_231035.yaml
2026-04-28 23:10:40.3995738140 src/omni-robot/slam_benchmark/maps/gmapping_stage7_test_20260428_231035.pgm

## Stage 10 implementation/test addendum

- Selected newest Stage 9 map: slam_benchmark/results/stage9/maps/corridor_static_gmapping_lidar_rep1_20260501_143342.yaml
- Copied map to nav_bringup/maps/lidar_baseline.{yaml,pgm}.
- map_server standalone PASS with use_sim_time:=false; with use_sim_time:=true it needs Gazebo /clock.
- Full navigation launch PASS headless with Gazebo /clock.
- check_navigation_stage10.sh PASS for /map, /lidar/scan, /odom, AMCL, move_base, costmaps, and TF chain.
- Sent goal x=0.6 y=0.0 yaw=0.0; move_base returned status 3 SUCCEEDED via run_nav_trial.sh.
- record_navigation_bag.sh PASS: results/stage10/bags/test_goal_1_20260501_180357.bag.
- evaluate_navigation_trial.py PASS: results/stage10/csv/test_goal_1_eval.csv.
- aggregate_navigation_results.py PASS: navigation_summary.csv plus Markdown/LaTeX tables.
- GlobalPlanner.allow_unknown is enabled because the Stage 9 map has high unknown ratio; local LiDAR costmap handles near-field obstacle avoidance.
