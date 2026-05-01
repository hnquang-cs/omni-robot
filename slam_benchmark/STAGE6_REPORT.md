# STAGE6_REPORT

## Goal
Stage 6 implements a reproducible SLAM baseline in simulation.
The primary baseline is Gmapping with the existing stereo-derived `/scan` and kinematic `/odom`.

Navigation, AMCL, `move_base`, and EKF-SLAM are intentionally out of scope.

## Architecture
`Gazebo -> stereo camera -> stereo_image_proc -> pointcloud_to_laserscan -> /scan -> slam_gmapping -> /map`

Odometry path:

`/cmd_vel -> omni_kinematics -> /odom + TF odom -> base_footprint`

## Files Created Or Updated
- `slam_benchmark/launch/gmapping_sim.launch`
- `slam_benchmark/launch/hector_sim.launch`
- `slam_benchmark/launch/slam_sim_full.launch`
- `slam_benchmark/launch/gmapping.launch`
- `slam_benchmark/launch/hector.launch`
- `slam_benchmark/config/gmapping_params.yaml`
- `slam_benchmark/config/hector_params.yaml`
- `slam_benchmark/rviz/slam.rviz`
- `slam_benchmark/scripts/save_map.sh`
- `slam_benchmark/scripts/record_slam_bag.sh`
- `slam_benchmark/scripts/check_slam_topics.sh`
- `slam_benchmark/scripts/play_bag_gmapping.sh`
- `slam_benchmark/README.md`
- `slam_benchmark/STAGE6_REPORT.md`
- `slam_benchmark/package.xml`
- `slam_benchmark/CMakeLists.txt`

## Topics
- Input:
  - `/scan`
  - `/odom`
  - `/tf`
  - `/tf_static`
  - `/clock`
  - `/cmd_vel`
- Output:
  - `/map`

## Frames
- `map`
- `odom`
- `base_footprint`
- `base_link`

Gmapping defaults to `base_footprint` because `omni_base_controller` publishes `odom -> base_footprint`.
The current `/scan` message uses `base_link`; TF already provides `base_footprint -> base_link`.

## Precheck Results In This Shell
- ROS master: FAIL, no master was running during initial precheck.
- `/scan`: not available in this shell precheck.
- `/odom`: not available in this shell precheck.
- TF `odom -> base_footprint`: not available in this shell precheck.
- `/use_sim_time`: not available in this shell precheck.

User-reported Stage 5 status says `/scan` is valid with `frame_id: base_link`, range `0.25..6.0`, and mixed finite/`inf` ranges.

## Dependency Check
- `hector_mapping`: PASS, installed.
- `map_server`: PASS, installed.
- `gmapping`: PASS, installed. The executable is `slam_gmapping` inside package `gmapping`.

Install command:

```bash
sudo apt install ros-noetic-gmapping
```

## Test Status
- Build: PASS with `catkin_make`.
- Launch XML/YAML/bash syntax: PASS.
- Full launch after fix: PASS, starts Gazebo, controller, stereo pipeline, and `/slam_gmapping`.
- `/scan`: FAIL in this headless test session because the topic exists but has no messages.
- `/odom`: PASS, topic exists and publishes around 28-30 Hz.
- `/map`: FAIL in this headless test session because Gmapping has no incoming scan data.
- TF `odom -> base_footprint`: PASS.
- TF `map -> odom`: PASS after `/slam_gmapping` starts.
- `check_slam_topics.sh`: PASS=4, FAIL=2 in this environment; failures are `/scan` and `/map`.
- Map save: FAIL because `/map` exists but has no message.
- Rosbag record: command starts, but smoke test was stopped by timeout before a complete bag file was produced.

## Commands Run
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make
roslaunch slam_benchmark slam_sim_full.launch use_rviz:=false gazebo_gui:=false
rostopic list
rostopic hz /odom
rostopic hz /scan
rostopic hz /map
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo map odom
rosrun slam_benchmark check_slam_topics.sh
rosrun slam_benchmark save_map.sh stage6_smoke
timeout 6 rosrun slam_benchmark record_slam_bag.sh stage6_smoke
```

## Map Output
No map file was saved in this session because `/map` did not publish an occupancy grid.
The intended output path for a successful run is:

`/home/hnquang/catkin_ws/src/omni-robot/slam_benchmark/maps/<map_name>.pgm`

and:

`/home/hnquang/catkin_ws/src/omni-robot/slam_benchmark/maps/<map_name>.yaml`

## Known Limitations
- The current `/scan` FOV is about 60 degrees, which is narrow for Gmapping.
- Gmapping may produce noisy or incomplete maps unless the robot moves slowly with overlap.
- Kinematic odometry is open-loop and may drift.
- Stereo-derived scans depend on scene texture, point cloud quality, and height filtering.

## Stage 7 Recommendations
- Tune Gmapping parameters using recorded bags.
- Compare Hector SLAM against Gmapping on the same bags.
- Increase `/scan` FOV if map coverage is insufficient.
- Start navigation only after a saved map is repeatable.
