# PROJECT_STRUCTURE

## Package overview
- `robot_description`: robot model, frame layout, and RViz assets.
- `omni_base_controller`: future omni-drive kinematics, odometry, and wheel command interface.
- `stereo_pipeline`: future stereo processing chain from images to virtual scan.
- `nav_bringup`: navigation launch files and planner configuration.
- `slam_benchmark`: SLAM backend comparison and benchmark helpers.
- `experiment_tools`: rosbag and result-export utility scripts.

## Package relationships
- `robot_description` defines the robot frames used by all runtime packages.
- `stereo_pipeline` is expected to publish the virtual scan consumed by `slam_benchmark` and `nav_bringup`.
- `slam_benchmark` evaluates SLAM backends that publish `/map` and TF relationships.
- `nav_bringup` consumes `/scan`, `/odom`, `/map`, and `/tf`, then publishes `/cmd_vel`.
- `omni_base_controller` consumes `/cmd_vel` and produces base motion feedback such as `/odom`.
- `experiment_tools` records and exports artifacts across all other packages.

## Planned system pipeline
`stereo camera -> depth/disparity -> pointcloud -> laserscan -> SLAM -> navigation -> cmd_vel -> omni controller`

## Planned core topics
- `/cmd_vel`: navigation command sent to the omni controller.
- `/odom`: base odometry estimate.
- `/tf`: frame transforms across the system.
- `/scan`: virtual 2D scan generated from stereo perception.
- `/map`: map output from SLAM or localization.
- `/move_base_simple/goal`: manual goal input from RViz.
- `/stereo/left/image_raw`: left camera image.
- `/stereo/right/image_raw`: right camera image.
- `/stereo/left/camera_info`: left calibration info.
- `/stereo/right/camera_info`: right calibration info.
- `/stereo/points2`: projected point cloud placeholder.

## Planned core frames
- `map`
- `odom`
- `base_footprint`
- `base_link`
- `camera_left`
- `camera_right`
- `laser_virtual`

## Design notes for later phases
- Keep frame ownership explicit to avoid TF duplication.
- Keep `/scan` as the navigation-facing abstraction even when the source is stereo.
- Push tunable planner and perception parameters into YAML instead of code.
