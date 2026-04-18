# omni-robot

ROS1 Noetic thesis workspace support files for an indoor omni-directional robot that uses stereo perception to replace LiDAR in the navigation stack.

## Fixed environment

- Ubuntu 20.04.6 LTS
- ROS1 Noetic
- Gazebo 11.15.1
- Main workspace: `~/catkin_ws`
- Build tool: `catkin_make`

## Important ROS packages

- `ros-noetic-navigation`
- `ros-noetic-slam-gmapping`
- `ros-noetic-hector-slam`
- `ros-noetic-robot-localization`
- `ros-noetic-teb-local-planner`
- `ros-noetic-gazebo-ros`
- `ros-noetic-pointcloud-to-laserscan`
- `ros-noetic-stereo-image-proc`
- `ros-noetic-depth-image-proc`
- `ros-noetic-image-proc`
- `ros-noetic-image-view`
- `ros-noetic-tf`
- `ros-noetic-xacro`
- `ros-noetic-joint-state-publisher`
- `ros-noetic-robot-state-publisher`

```bash
source ./scripts/install_dependencies.sh
```

## Source commands

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

Or use:

```bash
source ./scripts/env_setup.sh
```

## Build workspace

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Basic tools

Start ROS master:

```bash
roscore
```

Open RViz:

```bash
rviz
```

Open Gazebo:

```bash
gazebo
```

## Notes

- This thesis is fixed on ROS1 Noetic and should not be upgraded casually.
- Prefer official ROS Noetic packages from `apt` before introducing extra dependencies.
- Rebuild the workspace after structural changes.
