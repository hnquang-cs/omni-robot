# AGENTS

## Project summary
This repository hosts the ROS1 Noetic thesis skeleton for an indoor omni-directional autonomous robot that relies on stereo perception instead of LiDAR.

## Fixed environment
- Ubuntu 20.04
- ROS1 Noetic
- Catkin workspace: `~/catkin_ws`
- Build tool: `catkin_make`

## Working rules
- Do not switch to ROS2.
- Do not rename agreed topics or frames without updating project-wide documentation.
- Prefer configuration through YAML and launch arguments before hard-coded values.
- Rebuild the workspace after every meaningful structural change.
- Do not add heavy dependencies unless the current phase truly requires them.
