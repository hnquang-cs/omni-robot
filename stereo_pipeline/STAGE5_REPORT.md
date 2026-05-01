# STAGE5_REPORT

## Goal
Stage 5 focuses on simulation only.
The implemented architecture is:

`Gazebo stereo camera -> /stereo/left|right/image_raw + camera_info -> stereo_image_proc -> /stereo/disparity + /stereo/points2 -> pointcloud_to_laserscan -> /scan`

SLAM and navigation are intentionally not launched in this stage.

## Files created or updated
- `robot_description/urdf/omni_robot.urdf.xacro`
- `robot_description/launch/gazebo_sim.launch`
- `robot_description/worlds/test_arena.world`
- `robot_description/README.md`
- `stereo_pipeline/launch/stereo_processing.launch`
- `stereo_pipeline/launch/stereo_sim_pipeline.launch`
- `stereo_pipeline/launch/stereo_to_scan.launch`
- `stereo_pipeline/config/stereo_params.yaml`
- `stereo_pipeline/config/laserscan_params.yaml`
- `stereo_pipeline/scripts/check_stereo_topics.sh`
- `stereo_pipeline/scripts/check_scan.sh`
- `stereo_pipeline/README.md`
- `stereo_pipeline/STAGE5_REPORT.md`

## Frames
- Base frame: `base_link`
- Stereo sensor frames:
  - `camera_left`
  - `camera_right`
  - `camera_left_optical`
  - `camera_right_optical`

## Topics
- Input topics:
  - `/stereo/left/image_raw`
  - `/stereo/left/camera_info`
  - `/stereo/right/image_raw`
  - `/stereo/right/camera_info`
- Intermediate topics:
  - `/stereo/disparity`
  - `/stereo/points2`
- Output topic:
  - `/scan`

## Commands planned for validation
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch robot_description gazebo_sim.launch gui:=false
roslaunch stereo_pipeline stereo_sim_pipeline.launch
rostopic list
rostopic hz /stereo/left/image_raw
rostopic hz /stereo/right/image_raw
rostopic hz /stereo/disparity
rostopic hz /stereo/points2
rostopic hz /scan
rosrun tf tf_echo base_link camera_left
rosrun tf tf_echo base_link camera_right
```

## Commands actually run
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
xacro src/omni-robot/robot_description/urdf/omni_robot.urdf.xacro
roslaunch robot_description gazebo_sim.launch gui:=false
roslaunch stereo_pipeline stereo_sim_pipeline.launch
rosnode list
rostopic list
rostopic hz /stereo/left/image_raw
rostopic hz /scan
rosrun tf tf_echo base_link camera_left
rosrun tf tf_echo base_link camera_right
src/omni-robot/stereo_pipeline/scripts/check_stereo_topics.sh
src/omni-robot/stereo_pipeline/scripts/check_scan.sh
```

## Test results on this terminal environment
- `catkin_make`: PASS
- `xacro` parse for `omni_robot.urdf.xacro`: PASS
- `gazebo_sim.launch` starts `gzserver`, `robot_state_publisher`, and spawns the robot: PASS
- `stereo_sim_pipeline.launch` starts `stereo_image_proc` nodelets and `pointcloud_to_laserscan`: PASS
- `base_link -> camera_left` TF: PASS
- `base_link -> camera_right` TF: PASS
- `check_stereo_topics.sh` topic-name existence check: PASS
- Raw stereo image publish rate: FAIL in this shell-only environment
- Disparity/point cloud runtime data flow: FAIL in this shell-only environment
- `/scan` publish rate and sample message: FAIL in this shell-only environment

## Root cause of the runtime FAIL items
Gazebo Classic camera sensors require a render-capable display.
During verbose testing in this terminal-only session, Gazebo reported:

- `Can't open display`
- `Rendering will be disabled`

With rendering disabled, the Gazebo world and ROS graph still start, but camera images are not produced, so `stereo_image_proc` and `/scan` stay idle.

## Recommended package for headless testing
Install:

```bash
sudo apt install xvfb
```

Then launch:

```bash
xvfb-run -a roslaunch robot_description gazebo_sim.launch gui:=false
roslaunch stereo_pipeline stereo_sim_pipeline.launch
```

## Expected Stage 6 readiness
- Gazebo simulation world available
- Stereo camera topics available
- Virtual `/scan` topic prepared for later SLAM experiments
- Topic and frame conventions documented for the next stage

## Open issues to confirm during runtime
- Verify raw image message rates once Gazebo has a render-capable display.
- Verify that Gazebo camera calibration produces usable disparity in the simple world.
- Verify that the point cloud contains enough points for stable `/scan` generation.
- Tune the point cloud height filter if the ground plane appears in `/scan`.
