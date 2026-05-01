# omni_base_controller

## Purpose
This package provides the Stage 4 kinematic base controller for the omni thesis robot. It receives body twist commands, computes wheel angular velocities, publishes kinematic odometry, and broadcasts the `odom -> base_footprint` transform.

## Kinematic model assumption
- The node uses a symmetric 4-wheel holonomic inverse-kinematics model.
- The wheel order is:
  - `front_left`
  - `front_right`
  - `rear_left`
  - `rear_right`
- The simplified inverse mapping is:
  - `w_fl = (vx - vy - (lx + ly) * wz) / r`
  - `w_fr = (vx + vy + (lx + ly) * wz) / r`
  - `w_rl = (vx + vy - (lx + ly) * wz) / r`
  - `w_rr = (vx - vy + (lx + ly) * wz) / r`
- This is a kinematic testing model for an ideal holonomic base, not a full hardware-specific wheel-ground model.

## Input topic
- Input: `/cmd_vel`

## Output topics
- Output: `/wheel_velocities` as `sensor_msgs/JointState`
  Reason: wheel names, positions, and angular velocities stay together in one message and can be reused later by hardware or simulation layers.
- Output: `/joint_states`
  Reason: mirrors the wheel state so `robot_state_publisher` can keep wheel TFs alive without a separate joint state source.
- Output: `/odom`
- Output: `/tf` from `odom` to `base_footprint`

## Frames
- Odom frame: `odom`
- Base frame: `base_footprint`
- The rest of the robot tree comes from `robot_description` and `robot_state_publisher`.

## Important files
- `scripts/omni_kinematics.py`: main Stage 4 kinematic controller node.
- `scripts/test_cmd_sequence.sh`: quick test script for body-twist commands.
- `config/omni_params.yaml`: kinematic, frame, and topic parameters.
- `launch/controller.launch`: bringup file for standalone testing.

## Run
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch omni_base_controller controller.launch
```

## Test commands
Check topics:

```bash
rostopic list
rostopic echo /wheel_velocities
rostopic echo /odom
```

Run a command sequence:

```bash
~/catkin_ws/src/omni-robot/omni_base_controller/scripts/test_cmd_sequence.sh
```

Check TF:

```bash
rosrun tf tf_echo odom base_footprint
```

## Test note
- Do not run `robot_description/display.launch` at the same time as this controller launch unless you intentionally want a separate `joint_state_publisher`.
- A second publisher on `/joint_states` can interfere with wheel-joint visualization, even though `/wheel_velocities`, `/odom`, and `odom -> base_footprint` still come from this controller node.

## Current limitations
- Odometry is kinematic odometry integrated from the active command, not encoder-based odometry.
- The node is open-loop and does not include PID or closed-loop wheel control.
- No hardware interface or Gazebo drive plugin is included at this stage.

## TODO for later stages
- Replace kinematic odometry with real encoder-based odometry.
- Add a hardware interface layer for motor drivers.
- Add a Gazebo plugin or simulation bridge when simulation control is needed.
- Add closed-loop wheel control after the low-level interface is defined.
