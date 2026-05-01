# stereo_pipeline

## Purpose
This package implements the Stage 5 simulation-only perception stack for the omni thesis robot on ROS1 Noetic.
The main goal is to keep the pipeline simple and easy to debug:

`Gazebo stereo camera -> left/right image_raw + camera_info -> stereo_image_proc -> disparity + point cloud -> pointcloud_to_laserscan -> /scan`

## Main launch files
- `launch/stereo_processing.launch`: starts `stereo_image_proc` only.
- `launch/stereo_sim_pipeline.launch`: starts the full Stage 5 pipeline and publishes `/scan`.
- `launch/stereo_to_scan.launch`: compatibility wrapper that includes `stereo_sim_pipeline.launch`.

## Main config files
- `config/stereo_params.yaml`: central reference for expected topics, frames, image resolution, and stereo baseline.
- `config/laserscan_params.yaml`: `pointcloud_to_laserscan` parameters used to generate `/scan`.

## Main topics
- Inputs:
  - `/stereo/left/image_raw`
  - `/stereo/left/camera_info`
  - `/stereo/right/image_raw`
  - `/stereo/right/camera_info`
- Intermediate outputs:
  - `/stereo/disparity`
  - `/stereo/points2`
- Final output:
  - `/scan`

## Main frames
- `base_link`
- `camera_left`
- `camera_right`
- `camera_left_optical`
- `camera_right_optical`

## How to build
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## How to run
Start Gazebo with the robot and Stage 5 test world:

```bash
roslaunch robot_description gazebo_sim.launch gui:=true
```

Start the stereo processing pipeline:

```bash
roslaunch stereo_pipeline stereo_sim_pipeline.launch
```

If you only want disparity and point cloud without `/scan`:

```bash
roslaunch stereo_pipeline stereo_processing.launch
```

If you are testing from a headless SSH session, Gazebo stereo cameras still need rendering.
Install `xvfb` on Ubuntu 20.04 if needed:

```bash
sudo apt install xvfb
```

Then launch Gazebo without the GUI through:

```bash
xvfb-run -a roslaunch robot_description gazebo_sim.launch gui:=false
```

## Step-by-step test flow
1. Check that Gazebo publishes the raw stereo camera topics:

```bash
~/catkin_ws/src/omni-robot/stereo_pipeline/scripts/check_stereo_topics.sh
```

2. Verify message rates manually:

```bash
rostopic hz /stereo/left/image_raw
rostopic hz /stereo/right/image_raw
rostopic hz /stereo/disparity
rostopic hz /stereo/points2
rostopic hz /scan
```

3. Check the virtual scan output:

```bash
~/catkin_ws/src/omni-robot/stereo_pipeline/scripts/check_scan.sh
```

4. Check camera transforms if needed:

```bash
rosrun tf tf_echo base_link camera_left
rosrun tf tf_echo base_link camera_right
```

## Notes on stereo_image_proc
`stereo_image_proc` is the standard ROS stereo-processing package used here to rectify the left/right camera streams and generate disparity plus point cloud outputs.
The launch file keeps topic naming explicit so the Stage 5 pipeline stays easy to inspect with `rostopic`, `rqt_graph`, and RViz.

## Current limitations
- The Gazebo cameras use a zero-distortion model to keep debugging simple.
- This package is only for simulation in Stage 5, not for real hardware.
- Stereo matching quality depends on visible scene texture and lighting in Gazebo.
- `/scan` is a virtual scan projected from stereo point cloud data, so it behaves differently from a physical LiDAR.
- Gazebo camera topics may appear in the ROS graph but stay silent in a headless shell if Gazebo has no render-capable display.
- No SLAM node is started in this package.
- No navigation node is started in this package.

## Fallback note
- A depth-camera-based fallback could be documented later for debugging on very slow machines.
- That fallback is not the main architecture and is intentionally not launched here.

## TODO for later stages
- Tune stereo quality, image rate, and point cloud filtering.
- Add SLAM on top of `/scan`.
- Add navigation after SLAM is stable.
- Compare stereo-processing and scan-filtering variants.
