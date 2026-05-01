# slam_benchmark

## Purpose
This package contains the Stage 6 SLAM baseline for the omni robot thesis project on ROS1 Noetic.
The main baseline is Gmapping in Gazebo using the stereo-derived `/scan`.

Pipeline:

`Gazebo -> stereo camera -> /scan -> Gmapping -> /map -> map_saver`

## Main topics
- `/scan`: virtual scan from `stereo_pipeline`
- `/odom`: kinematic odometry from `omni_base_controller`
- `/tf`: robot and SLAM transforms
- `/map`: occupancy grid from SLAM
- `/cmd_vel`: manual mapping command input

## Main frames
- `map`: global map frame from SLAM
- `odom`: odometry frame from the controller
- `base_footprint`: current Gmapping base frame
- `base_link`: robot chassis frame and `/scan` frame

## Launch files
- `launch/gmapping_sim.launch`: Gmapping baseline only.
- `launch/hector_sim.launch`: Hector SLAM comparison launch, not used by default.
- `launch/slam_sim_full.launch`: Gazebo + controller + stereo pipeline + Gmapping + RViz.
- `launch/gmapping.launch`: compatibility wrapper for `gmapping_sim.launch`.
- `launch/hector.launch`: compatibility wrapper for `hector_sim.launch`.

## Dependencies
Required for the Gmapping baseline:

```bash
sudo apt install ros-noetic-gmapping ros-noetic-map-server
```

Optional Hector comparison:

```bash
sudo apt install ros-noetic-hector-slam
```

## Run Full Simulation
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch slam_benchmark slam_sim_full.launch
```

If testing headless, Gazebo camera rendering may require:

```bash
xvfb-run -a roslaunch slam_benchmark slam_sim_full.launch use_rviz:=false gazebo_gui:=false
```

## Run Gmapping Only
Use this when Gazebo, `/scan`, `/odom`, and TF are already running:

```bash
roslaunch slam_benchmark gmapping_sim.launch
```

## Manual Mapping Commands
Move slowly. The current `/scan` has a narrow FOV, so fast rotation reduces scan overlap.

Forward:

```bash
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10
```

Sideways:

```bash
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.10, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10
```

Slow rotation:

```bash
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.20}}' -r 10
```

Stop:

```bash
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
```

For mapping, use small loops and keep enough overlap between observations.

## Check SLAM
```bash
rosrun slam_benchmark check_slam_topics.sh
```

Manual checks:

```bash
rostopic hz /scan
rostopic hz /odom
rostopic hz /map
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo map odom
```

## Save Map
```bash
rosrun slam_benchmark save_map.sh gmapping_test
```

Direct fallback:

```bash
roscd slam_benchmark
./scripts/save_map.sh gmapping_test
```

Expected outputs:

- `maps/gmapping_test.pgm`
- `maps/gmapping_test.yaml`

## Record Bag
```bash
rosrun slam_benchmark record_slam_bag.sh
```

Default recorded topics:

- `/scan`
- `/odom`
- `/tf`
- `/tf_static`
- `/clock`
- `/cmd_vel`
- `/map`

## Replay Bag With Gmapping
Terminal 1:

```bash
roslaunch slam_benchmark gmapping_sim.launch
```

Terminal 2:

```bash
rosrun slam_benchmark play_bag_gmapping.sh /path/to/slam.bag
```

After replay:

```bash
rosrun slam_benchmark save_map.sh gmapping_replay
```

## Stage 7: Gazebo Motion, SLAM Tuning, and Benchmarking

Stage 7 keeps the project on ROS1 Noetic and focuses on repeatable SLAM experiments only. It does not launch navigation, `move_base`, AMCL, a real robot stack, or EKF-SLAM.

### Goals
- Move the Gazebo model from `/cmd_vel` so the stereo cameras and `/scan` change with the simulated world.
- Publish `/odom` and `odom -> base_footprint` from Gazebo truth instead of only integrating `/cmd_vel`.
- Tune Gmapping and add Hector SLAM as a comparison backend.
- Record/replay the same ROS bag for repeatable map benchmarks.
- Export simple map metrics to CSV.

### Why the Gazebo Model Controller Exists

The Stage 6 kinematic controller publishes wheel state, `/odom`, and TF, but it does not apply forces or joint commands to Gazebo. In that state the robot can appear to move in odom while the Gazebo model and camera sensors stay fixed.

Stage 7 adds `omni_base_controller/scripts/gazebo_model_cmd_vel.py`. It subscribes to `/cmd_vel`, integrates a holonomic body twist, and calls `/gazebo/set_model_state` for the `omni_robot` model. This is a kinematic simulation controller for SLAM baseline work.

This is different from a full `gazebo_ros_control` setup:

- kinematic controller: directly sets model pose, simple, stable, easy to debug, not physically realistic;
- `gazebo_ros_control`: commands joints through controllers and physics, more realistic, more setup and tuning.

For this thesis stage, the kinematic controller is the conservative choice because the goal is a repeatable SLAM benchmark, not final drivetrain dynamics.

### Run Full Stage 7 System

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch slam_benchmark slam_sim_full.launch
```

Headless fallback:

```bash
xvfb-run -a roslaunch slam_benchmark slam_sim_full.launch use_rviz:=false gazebo_gui:=false
```

Important Stage 7 defaults in `slam_sim_full.launch`:

- `use_gazebo_model_controller:=true`
- `use_gazebo_truth_odom:=true`
- `use_slam_scan_profile:=true`
- `gmapping_profile:=conservative`

When `use_gazebo_truth_odom` is true, `omni_kinematics.py` keeps publishing wheel/joint state but its `/odom` and TF output are disabled to avoid uncontrolled duplicate `odom -> base_footprint` publishers.

### Check Gazebo Motion

```bash
rostopic pub /cmd_vel geometry_msgs/Twist \
'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}' -r 10
```

In another terminal:

```bash
rostopic echo /gazebo/model_states
rostopic echo /odom -n 1
rosrun tf tf_echo odom base_footprint
```

The `omni_robot` yaw in `/gazebo/model_states` and `/odom` should change together.

Automated check:

```bash
rosrun slam_benchmark check_stage7_system.sh
```

### Gmapping Tuned Profiles

Run only Gmapping when `/scan`, `/odom`, and TF already exist:

```bash
roslaunch slam_benchmark gmapping_conservative.launch
roslaunch slam_benchmark gmapping_fast.launch
```

Wrapper:

```bash
rosrun slam_benchmark run_gmapping_profile.sh conservative
rosrun slam_benchmark run_gmapping_profile.sh fast
```

Profiles are documented in `config/gmapping_tuned.yaml`. The conservative profile uses more particles and smaller update thresholds for noisy stereo `/scan`; the fast profile is meant for quick rosbag sweeps.

### Hector SLAM

Hector is configured with:

- scan topic: `/scan`
- map frame: `hector_map`
- map topic: `/map_hector`
- odom frame: `odom`

Install if needed:

```bash
sudo apt install ros-noetic-hector-slam
```

Run:

```bash
rosrun slam_benchmark run_hector.sh
```

Check:

```bash
rosrun slam_benchmark check_hector_topics.sh
```

Do not run Hector and Gmapping together unless the map topics/frames are intentionally separated.

### Record Stage 7 Bag

Default bag excludes camera images to keep files small:

```bash
rosrun slam_benchmark record_stage7_bag.sh
```

Recorded default topics:

- `/scan`
- `/odom`
- `/tf`
- `/tf_static`
- `/clock`
- `/cmd_vel`
- `/gazebo/model_states`

Optional camera recording:

```bash
rosrun slam_benchmark record_stage7_bag.sh --with-camera
```

### Replay Bag

Gmapping replay:

```bash
rosrun slam_benchmark replay_with_gmapping.sh /path/to/stage7.bag conservative
```

Hector replay:

```bash
rosrun slam_benchmark replay_with_hector.sh /path/to/stage7.bag
```

The replay scripts launch only the SLAM backend, set `/use_sim_time`, play the bag with `/clock`, then keep the SLAM node running so the map can be saved. Set `EXIT_AFTER_PLAY=true` for automated smoke tests that should exit after playback.

### Save Map

Gmapping:

```bash
rosrun slam_benchmark save_map_with_prefix.sh gmapping_conservative
```

Hector:

```bash
rosrun slam_benchmark save_map_with_prefix.sh hector /map_hector
```

Maps are written to `slam_benchmark/maps/`.

### Evaluate Map

```bash
rosrun slam_benchmark evaluate_map_basic.py slam_benchmark/maps/gmapping_conservative_<timestamp>.yaml
```

The script exports a CSV in `slam_benchmark/results/` with width, height, resolution, occupied/free/unknown counts, and ratios.

Combine multiple results:

```bash
rosrun slam_benchmark compare_map_csv.py results/map_a.csv results/map_b.csv
```

The combined output is `results/stage7_map_comparison.csv`.

### RViz

Use `rviz/slam_tuning.rviz`.

- Fixed Frame `map` for Gmapping.
- Change Fixed Frame to `hector_map` and enable `Hector Map /map_hector` when running Hector separately.

### Stage 7 Limitations

- The Gazebo model controller is kinematic and does not model wheel slip, inertia, motor limits, or contact dynamics.
- Stereo-derived `/scan` has FOV/noise characteristics very different from a 2D LiDAR.
- Basic map metrics are useful for consistency checks but do not replace ground-truth map evaluation.
- If the map still does not match Gazebo, check in this order: Gazebo model motion, `/odom`/TF alignment, scan frame, scan FOV, then Gmapping parameters.

## Current Limitations
- `/scan` is generated from stereo point cloud data. The original profile has about 60 degrees FOV; the Stage 7 SLAM profile requests about 120 degrees, limited by what the stereo point cloud actually contains.
- Gmapping usually needs stable odometry and enough scan overlap; narrow FOV can make maps noisy.
- Map quality depends strongly on `pointcloud_to_laserscan` height filtering and TF consistency.
- This stage does not launch navigation, `move_base`, AMCL, or EKF-SLAM.

## TODO
- Tune Gmapping parameters after collecting bags.
- Increase `/scan` FOV if mapping coverage is insufficient.
- Compare Hector SLAM with the same bag files.
- Prepare navigation only after a repeatable map can be saved.
