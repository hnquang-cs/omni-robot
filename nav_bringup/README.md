# nav_bringup

## Purpose
This package brings up the ROS1 Noetic navigation baseline for the simulated indoor omni robot.

Stage 10 architecture:

`Gazebo + LiDAR -> /lidar/scan -> map_server -> AMCL -> move_base -> TEB -> /cmd_vel -> Gazebo robot`

Stereo-depth is not used as the default navigation source in Stage 10.

## Main Topics
- `/map`: static LiDAR map from `map_server`
- `/lidar/scan`: simulated 2D LiDAR
- `/odom`: Gazebo truth odometry
- `/amcl_pose`: AMCL pose estimate
- `/particlecloud`: AMCL particles
- `/move_base/status`: navigation action status
- `/move_base_simple/goal`: RViz/script goal input
- `/cmd_vel`: velocity command to Gazebo model controller

## Main TF
Expected chain:

`map -> odom -> base_footprint -> base_link -> lidar_link`

AMCL owns `map -> odom`. Gazebo truth odom owns `odom -> base_footprint`. Do not run Gmapping in the default navigation launch.

## Run Full Navigation
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch nav_bringup navigation_lidar_full.launch
```

Headless:

```bash
roslaunch nav_bringup navigation_lidar_full.launch use_rviz:=false gazebo_gui:=false
```

If a local Python virtualenv shadows ROS Python and `spawn_model` cannot import `rospkg`, deactivate the venv or put `/usr/bin` and `/opt/ros/noetic/bin` before the venv in `PATH`.

## Initial Pose
In RViz, use `2D Pose Estimate` near the Gazebo robot's starting pose. The default AMCL initial pose is `(0, 0, 0)` for the Stage 9 map, which matches the normal spawn pose.

## Send Goals
Use RViz `2D Nav Goal`, or send a scripted goal:

```bash
rosrun nav_bringup send_nav_goal.py --x 2.0 --y 0.5 --yaw 0.0
```

## Check Navigation
```bash
roscd nav_bringup
./scripts/check_navigation_stage10.sh
```

Manual checks:

```bash
rostopic hz /map
rostopic hz /lidar/scan
rostopic hz /odom
rostopic hz /amcl_pose
rostopic hz /move_base/status
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo base_link lidar_link
```

## Record and Evaluate
Record a bag:

```bash
rosrun nav_bringup record_navigation_bag.sh test_goal_1
```

Run a trial while navigation is already running:

```bash
rosrun nav_bringup run_nav_trial.sh test_goal_1 2.0 0.5 0.0
```

Evaluate a bag:

```bash
rosrun nav_bringup evaluate_navigation_trial.py <bag_path> 2.0 0.5 0.0 results/stage10/csv/test_goal_1_eval.csv
```

Aggregate evaluation CSVs:

```bash
rosrun nav_bringup aggregate_navigation_results.py
```

Outputs:

```text
results/stage10/bags/
results/stage10/csv/navigation_trials.csv
results/stage10/csv/navigation_summary.csv
results/stage10/latex/table_navigation_results.tex
results/stage10/markdown/table_navigation_results.md
```

## Tuning Notes
- `footprint`: currently rectangular, matching the 0.52 m x 0.40 m chassis.
- `inflation_radius`: `0.35 m`; reduce only if narrow passages are blocked, increase if obstacle clearance is too small.
- `max_vel_x/y/theta`: conservative for the kinematic Gazebo controller.
- `obstacle_range`: `6.0 m`; `raytrace_range`: `8.0 m`.
- `GlobalPlanner.allow_unknown`: enabled because the Stage 9 baseline map still has a high unknown ratio; LiDAR/local costmap remains responsible for near-field obstacle avoidance.
- AMCL uses `odom_model_type: omni`; if a local AMCL build rejects it, use `diff` as a fallback and document the holonomic odometry limitation.

## Limitations
- The Gazebo model controller is kinematic and does not simulate full drivetrain dynamics.
- Simulated LiDAR is cleaner than real LiDAR.
- AMCL needs a reasonable initial pose.
- Stereo-depth is retained for research but is not the main Stage 10 navigation sensor.
## Stage 10 LiDAR Navigation Maps

The original Stage 9 LiDAR map remains available at:

- `maps/lidar_baseline.yaml`
- `maps/lidar_baseline.pgm`

Stage 10 navigation now defaults to the wide map path:

- `maps/lidar_baseline_wide.yaml`
- `maps/lidar_baseline_wide.pgm`

The main navigation map for the final Gmapping LiDAR baseline is:

- `maps/lidar_baseline_wide_obstacles.yaml`
- `maps/lidar_baseline_wide_obstacles.pgm`

`lidar_baseline_wide_obstacles` is generated from the wider Gazebo world with
7 static box obstacles and is the preferred map for forward-only navigation.
The original and earlier wide maps are kept for comparison and reproducibility.

Use the original map when you need to reproduce the Stage 9 baseline exactly:

```bash
roslaunch nav_bringup navigation_lidar_full.launch map_file:=$(rospack find nav_bringup)/maps/lidar_baseline.yaml
```

Use the wide map for Stage 10 acceptance tests:

```bash
roslaunch nav_bringup navigation_lidar_full.launch map_file:=$(rospack find nav_bringup)/maps/lidar_baseline_wide.yaml
```

Use the final wide-obstacles map for the main navigation run:

```bash
roslaunch nav_bringup navigation_lidar_forward_only.launch \
  map_file:=$(rospack find nav_bringup)/maps/lidar_baseline_wide_obstacles.yaml
```

To regenerate the wide map from LiDAR SLAM, launch:

```bash
roslaunch slam_benchmark slam_lidar_wide_full.launch
roscd slam_benchmark
./scripts/remap_lidar_wide_area.sh
```

To regenerate the final wide-obstacles map:

```bash
roslaunch slam_benchmark slam_lidar_wide_obstacles_full.launch
roscd slam_benchmark
./scripts/create_gmapping_wide_obstacles_map.sh
```

The fixed Gazebo world adds `missing_wall_stage10` at pose `[-0.5, 0.0, 0.75]` with size `[0.12, 3.0, 1.5]`, closing the rear side of the corridor without overlapping the robot spawn at `[0.0, 0.0]`.

## Stage 10 Forward-Only Navigation

The baseline Stage 10 safety mode treats the robot as forward-only:

- allowed: `linear.x >= 0`
- allowed: `angular.z` within configured limits
- blocked: `linear.x < 0`
- blocked: any `linear.y`

Launch:

```bash
roslaunch nav_bringup navigation_lidar_forward_only.launch
```

This launch uses `move_base_teb_forward_only.launch`, which publishes planner output to `/cmd_vel_raw`. The final command sent to the Gazebo controller is `/cmd_vel`, produced by `cmd_vel_forward_only_filter.py`.

TEB is configured to avoid reverse/lateral commands with:

- `max_vel_x_backwards: 0.0`
- `max_vel_y: 0.0`
- `allow_init_with_backwards_motion: false`
- high `weight_kinematics_forward_drive`
- high `weight_kinematics_nh`

The filter remains the final guard before the controller. Any reverse command is clamped to zero forward speed, and any lateral command is clamped to `linear.y=0`.

Forward-only checks:

```bash
roscd nav_bringup
./scripts/check_cmd_vel_forward_only.sh
./scripts/test_goal_forward_only_stage10.sh
```

## Experimental Discrete-45 Navigation

The launch below is experimental and does not replace the normal
`navigation_lidar_forward_only.launch` baseline:

```bash
roslaunch nav_bringup navigation_lidar_discrete45_experimental.launch
```

In this mode, `move_base`/TEB still publishes continuous velocity commands to
`/cmd_vel_raw`, but `discrete_45_motion_controller.py` converts angular command
requests into `TURN_LEFT_45` or `TURN_RIGHT_45` primitives and publishes the
safe output to `/cmd_vel`.

This can make `move_base` harder to converge because the local planner expects
continuous velocity control, while the primitive controller only allows:

- `FORWARD`
- `TURN_LEFT_45`
- `TURN_RIGHT_45`
- `STOP`

Use this launch only for discrete-motion experiments, not as the main navigation
baseline.
