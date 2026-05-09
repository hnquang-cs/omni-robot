# Stage 10 Discrete 45-Degree Motion Controller

## 1. Goal

Add a motion primitive controller that allows only:

- `FORWARD`
- `TURN_LEFT_45`
- `TURN_RIGHT_45`
- `STOP`

The robot remains forward-only: no reverse, no lateral motion, and no combined
drive-and-turn command.

## 2. Why A Separate Controller Is Needed

`/cmd_vel` is a velocity command. A command such as `angular.z=0.25` asks for a
yaw rate, not a fixed yaw angle. To guarantee a relative `+45 deg` or `-45 deg`
turn, the robot must read current yaw from `/odom`, compute a target yaw, rotate
until the target is reached, and then publish zero velocity.

## 3. Topics

Inputs:

- `/motion_primitive_cmd` (`std_msgs/String`) in primitive mode
- `/cmd_vel_raw` (`geometry_msgs/Twist`) in cmd_vel conversion mode
- `/odom` (`nav_msgs/Odometry`) for yaw feedback

Outputs:

- `/cmd_vel` (`geometry_msgs/Twist`)
- `/motion_primitive_state` (`std_msgs/String`)

## 4. State Machine

States:

- `IDLE`
- `DRIVE_STRAIGHT`
- `TURNING_LEFT_45`
- `TURNING_RIGHT_45`
- `STOPPED`
- `ERROR_NO_ODOM`
- `ERROR_TURN_TIMEOUT`

Rules:

- `DRIVE_STRAIGHT`: `linear.x > 0`, `linear.y = 0`, `angular.z = 0`
- `TURNING_LEFT_45`: `linear.x = 0`, `linear.y = 0`, `angular.z > 0`
- `TURNING_RIGHT_45`: `linear.x = 0`, `linear.y = 0`, `angular.z < 0`
- `STOPPED`: all velocity fields zero

## 5. Test Results

Run:

```bash
roslaunch slam_benchmark slam_lidar_wide_obstacles_full.launch
roslaunch omni_base_controller discrete_45_motion_controller.launch
rosrun omni_base_controller test_discrete_45_primitives.py
```

Expected checks:

- `TURN_LEFT_45`: PASS if yaw delta is `+45 deg +/- 3 deg`
- `TURN_RIGHT_45`: PASS if yaw delta is `-45 deg +/- 3 deg`
- `FORWARD`: PASS if `linear.x > 0`, `linear.y = 0`, `angular.z = 0`
- `STOP`: PASS if all velocity fields are zero
- no lateral, no reverse, no simultaneous drive-and-turn

Measured in Gazebo on the wide-obstacles stack:

| Check | Result | Value |
|---|---:|---|
| TURN_LEFT_45 | PASS | `+43.943 deg` |
| TURN_RIGHT_45 | PASS | `-43.479 deg` |
| FORWARD | PASS | `linear.x > 0`, `angular.z = 0` |
| STOP | PASS | all velocity fields zero |
| no lateral | PASS | `linear.y = 0` |
| no reverse | PASS | `linear.x >= 0` |
| no drive-and-turn simultaneous | PASS | no sample had both active |

## 6. Limitations

This primitive controller is suitable for mapping and benchmark experiments
that require discrete orientation changes. It may not be suitable for continuous
TEB navigation because TEB expects fine-grained velocity control.
