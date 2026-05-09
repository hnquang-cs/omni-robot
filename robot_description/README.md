# robot_description

## Purpose
This package stores the robot model, frame layout, and RViz assets for the omni thesis platform during the visualization-focused stage of development.

## Robot structure
- `base_footprint`: root frame on the ground plane.
- `base_link`: chassis frame, using the ROS convention `x forward`, `y left`, `z up`.
- Wheels:
  - `front_left_wheel`
  - `front_right_wheel`
  - `rear_left_wheel`
  - `rear_right_wheel`
- Sensors:
  - `camera_left`
  - `camera_right`
  - `imu_link`
  - `lidar_link`

## Dimensions
- Chassis: `0.52 m x 0.40 m x 0.12 m`
- Wheel radius: `0.06 m`
- Wheel width: `0.03 m`
- Wheel offsets from `base_link`: `x = 0.19 m`, `y = 0.16 m`
- Stereo camera baseline: `0.10 m`
- Camera x offset: `0.19 m`
- Camera z offset: `0.14 m`
- LiDAR frame: `lidar_link`, mounted on `base_link`
- LiDAR offset from `base_link`: `x = 0.08 m`, `y = 0.00 m`, `z = 0.20 m`

## Important files
- `urdf/omni_robot.urdf.xacro`: xacro model for the chassis, wheels, and sensors.
- `launch/display.launch`: launches the model, TF publisher, and RViz.
- `launch/gazebo_sim.launch`: launches Gazebo with the robot and the Stage 5 test world.
- `launch/gazebo_lidar_sim.launch`: launches Gazebo with the same robot plus the Stage 9 simulated 2D LiDAR.
- `worlds/test_arena.world`: simple arena with obstacles at different depths for stereo testing.
- `rviz/robot.rviz`: default RViz profile with RobotModel, TF, and Grid.
- `rviz/lidar_test.rviz`: RViz profile for `/lidar/scan` validation.

## Launch
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch robot_description display.launch
```

For headless TF checks without opening RViz:

```bash
roslaunch robot_description display.launch open_rviz:=false
```

Launch the Gazebo stereo simulation:

```bash
roslaunch robot_description gazebo_sim.launch gui:=true
```

Launch the Gazebo LiDAR simulation without SLAM:

```bash
roslaunch robot_description gazebo_lidar_sim.launch gui:=true
```

Check the simulated LiDAR:

```bash
rostopic hz /lidar/scan
rostopic echo /lidar/scan -n 1
rosrun tf tf_echo base_link lidar_link
rviz -d $(rospack find robot_description)/rviz/lidar_test.rviz
```

For headless Gazebo testing:

```bash
roslaunch robot_description gazebo_sim.launch gui:=false
```

If you run Gazebo over SSH or another headless session, camera sensors still need rendering.
On a machine without an active X display, install `xvfb` and launch Gazebo through:

```bash
xvfb-run -a roslaunch robot_description gazebo_sim.launch gui:=false
```

## Expected TF chain
- `base_footprint -> base_link`
- `base_link -> front_left_wheel`
- `base_link -> front_right_wheel`
- `base_link -> rear_left_wheel`
- `base_link -> rear_right_wheel`
- `base_link -> camera_left`
- `base_link -> camera_right`
- `base_link -> imu_link`
- `base_link -> lidar_link`

## TODO for later stages
- Add measured inertial values and refined geometry.
- Add transmission or plugin elements only when simulation/control work begins.
- Tune Gazebo-specific properties once physics and control are introduced.

## Gazebo stereo camera notes
- The Gazebo model publishes:
  - `/stereo/left/image_raw`
  - `/stereo/left/camera_info`
  - `/stereo/right/image_raw`
  - `/stereo/right/camera_info`
- Camera frame IDs are published from:
  - `camera_left_optical`
  - `camera_right_optical`
- The simulated baseline matches the URDF baseline of `0.10 m`.
- The Gazebo plugins use a simple zero-distortion camera model and fixed calibration values to keep Stage 5 easy to debug.
- In headless environments, the world can still load without errors while the camera topics remain silent if Gazebo cannot create a render context.

## Stage 9 LiDAR notes
- The Gazebo model publishes a simulated planar 2D LiDAR scan on `/lidar/scan`.
- The scan frame ID is `lidar_link`.
- The horizontal FOV is 360 degrees with 720 samples at 10 Hz.
- Range limits are `0.15 m` to `10.0 m`, with small Gaussian noise for a less ideal scan.
- Stereo cameras and the existing stereo pipeline are intentionally kept. Stage 9 adds LiDAR as the main SLAM/navigation baseline sensor and leaves stereo-depth for comparison and later integration research.
## Stage 10 Navigation Fixed Arena

`worlds/test_arena_nav_fixed.world` is a non-destructive copy of `test_arena.world` for Stage 10 navigation tests.

It adds one static wall model:

- Name: `missing_wall_stage10`
- Pose: `x=-0.5, y=0.0, z=0.75, roll=0, pitch=0, yaw=0`
- Box size: `x=0.12, y=3.0, z=1.5`

The wall closes the rear side of the corridor while leaving the robot spawn at `x=0.0, y=0.0` clear.

Launch it with:

```bash
roslaunch robot_description gazebo_lidar_nav_fixed.launch
```
