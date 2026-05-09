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
## Stage 10 Forward-Only Command Filter

Stage 10 can run the robot as forward-only:

- `linear.x >= 0`
- `linear.y = 0`
- no reverse motion
- no lateral motion
- `angular.z` clamped to a safe limit

Use:

```bash
roslaunch omni_base_controller cmd_vel_forward_only_filter.launch
```

The filter subscribes to `/cmd_vel_raw` and publishes `/cmd_vel`. It is intended to sit between `move_base` and `gazebo_model_cmd_vel.py`.

Test it with:

```bash
rosrun omni_base_controller test_forward_only_filter.sh
```

The filter logs warnings when it receives reverse or lateral planner commands, then clamps them before they reach the Gazebo controller.

## Discrete 45-Degree Motion Controller

`/cmd_vel` is a velocity command, not an angle command. A normal planner can
request `angular.z=0.1`, `0.3`, or any continuous yaw rate, which does not
guarantee an exact 45 degree turn. The discrete mode therefore uses a separate
closed-loop controller:

- `scripts/discrete_45_motion_controller.py`
- `config/discrete_45_motion_controller.yaml`
- `launch/discrete_45_motion_controller.launch`

Motion primitives:

- `FORWARD`
- `TURN_LEFT_45`
- `TURN_RIGHT_45`
- `STOP`

Input topics:

- primitive mode: `/motion_primitive_cmd` as `std_msgs/String`
- cmd_vel mode: `/cmd_vel_raw` as `geometry_msgs/Twist`

Output:

- `/cmd_vel`
- `/motion_primitive_state`

In this mode, the robot never drives and turns at the same time:

- `FORWARD`: `linear.x > 0`, `linear.y = 0`, `angular.z = 0`
- `TURN_LEFT_45`: `linear.x = 0`, `linear.y = 0`, relative yaw target `+45 deg`
- `TURN_RIGHT_45`: `linear.x = 0`, `linear.y = 0`, relative yaw target `-45 deg`
- `STOP`: all velocity components zero

The controller reads `/odom` yaw and stops the turn when the relative yaw delta
is within the configured tolerance. It does not allow 10, 20, 60, or 90 degree
turns in one primitive command.

Run with an existing Gazebo/odom/controller stack:

```bash
roslaunch omni_base_controller discrete_45_motion_controller.launch
```

Manual primitive commands:

```bash
rostopic pub /motion_primitive_cmd std_msgs/String "data: 'TURN_LEFT_45'" -1
rostopic pub /motion_primitive_cmd std_msgs/String "data: 'TURN_RIGHT_45'" -1
rostopic pub /motion_primitive_cmd std_msgs/String "data: 'FORWARD'" -1
rostopic pub /motion_primitive_cmd std_msgs/String "data: 'STOP'" -1
```

Test and monitor:

```bash
rosrun omni_base_controller monitor_discrete_motion.py
rosrun omni_base_controller test_discrete_45_primitives.py
```

This mode is suitable for primitive-based mapping/benchmark experiments. It is
not necessarily a good match for continuous TEB navigation, because TEB produces
continuous velocity commands.

## Discrete 45-degree Motion Primitive Demo (end-to-end)

### Why /odom is mandatory

`/cmd_vel` is a velocity command, not an angle command. To turn exactly 45° the
controller must close the loop on yaw — it reads `/odom`, integrates the yaw
delta from the start of the turn, and stops the turn when the delta is within
`yaw_tolerance_deg`. Without `/odom`, the controller has no way to know how far
it has rotated, so it refuses to turn and reports
`/motion_primitive_state: ERROR_NO_ODOM`.

This is exactly what happens if you run only the controller-only launch:

```bash
roslaunch omni_base_controller discrete_45_motion_controller.launch   # <-- needs odom from elsewhere
```

The above ONLY starts the controller node. There is no Gazebo, no simulation,
no `/odom` — so the controller correctly enters `ERROR_NO_ODOM`. Use the demo
launch below instead.

### One-shot end-to-end demo

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch omni_base_controller discrete_45_demo.launch
```

This starts:

- Gazebo (`test_arena_wide_obstacles.world`) + `omni_robot` URDF + LiDAR.
- `gazebo_model_cmd_vel.py` (subscribes `/cmd_vel`, integrates pose,
  pushes it into Gazebo via `/gazebo/set_model_state`).
- `gazebo_truth_odom.py` (publishes `/odom` and TF `odom -> base_footprint`).
- `discrete_45_motion_controller.py` (publishes `/cmd_vel`).
- RViz with `rviz/discrete_45_demo.rviz` (Grid, TF, RobotModel, Odometry,
  LaserScan; fixed frame = `odom`).

### Health check

```bash
rosrun omni_base_controller debug_discrete45_odom.sh
```

Prints PASS/FAIL for: `odom_topic_exists`, `odom_has_message`, `odom_rate_ok`,
`tf_odom_base_ok`, `cmd_vel_has_subscriber`, `primitive_topic_ok`.

### Automated acceptance test

```bash
rosrun omni_base_controller test_discrete_45_primitives.py
```

Sends `STOP`, `TURN_LEFT_45`, `TURN_RIGHT_45`, `FORWARD`, `STOP` and asserts:

- left turn delta ≈ +45° (within 3°)
- right turn delta ≈ -45° (within 3°)
- `FORWARD` produces `linear.x > 0`, `linear.y = 0`, `angular.z = 0`
- no reverse, no lateral, no drive-and-turn at any time

A CSV is written to `omni_base_controller/results/discrete_45_test_result.csv`.

### Manual commands

```bash
rostopic pub /motion_primitive_cmd std_msgs/String "data: 'TURN_LEFT_45'"  -1
rostopic pub /motion_primitive_cmd std_msgs/String "data: 'TURN_RIGHT_45'" -1
rostopic pub /motion_primitive_cmd std_msgs/String "data: 'FORWARD'"       -1
rostopic pub /motion_primitive_cmd std_msgs/String "data: 'STOP'"          -1

rostopic echo /motion_primitive_state
rostopic echo /motion_primitive_debug
rostopic echo /cmd_vel
```

### Limitations

- Demo uses ground-truth odometry from Gazebo, not encoder-based odometry.
- Demo does not run `move_base`, Gmapping, or Hector — discrete primitives are
  intended for safe motion-primitive benchmarking, not continuous navigation.
- Maximum forward speed is clamped to 0.35 m/s; max angular speed to ~0.45 rad/s.
