# robot_description

## Purpose
This package stores the placeholder URDF/Xacro model, frame layout, and RViz assets for the omni thesis robot.

## Planned inputs and outputs
- Input: `joint_states` from `joint_state_publisher` or a future hardware/simulation source.
- Output: `/tf` from `robot_state_publisher`.
- Output: `robot_description` parameter for visualization and downstream launch files.

## Important files
- `urdf/omni_robot.urdf.xacro`: minimal base, camera, and virtual laser frame layout.
- `launch/display.launch`: launches the model, TF publisher, and RViz.
- `rviz/robot.rviz`: default visualization profile for early validation.

## Next-stage work
- Replace the placeholder geometry with a measured chassis model.
- Add wheel joints, sensors, and inertial tuning for simulation.
- Align all frames with the final navigation and stereo perception stack.
