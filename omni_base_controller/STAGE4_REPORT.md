# STAGE4_REPORT

## Kinematic model used
- Symmetric 4-wheel holonomic inverse-kinematics model
- Wheel order:
  - `front_left`
  - `front_right`
  - `rear_left`
  - `rear_right`
- Inverse mapping:
  - `w_fl = (vx - vy - (lx + ly) * wz) / r`
  - `w_fr = (vx + vy + (lx + ly) * wz) / r`
  - `w_rl = (vx + vy - (lx + ly) * wz) / r`
  - `w_rr = (vx - vy + (lx + ly) * wz) / r`
- Odometry is kinematic odometry integrated from the active command velocity.

## Topics
- Input:
  - `/cmd_vel`
- Outputs:
  - `/wheel_velocities` as `sensor_msgs/JointState`
  - `/joint_states` mirrored for `robot_state_publisher`
  - `/odom`
  - `/tf`

## Frames
- `odom`
- `base_footprint`

## Files created or updated
- `omni_base_controller/scripts/omni_kinematics.py`
- `omni_base_controller/scripts/test_cmd_sequence.sh`
- `omni_base_controller/config/omni_params.yaml`
- `omni_base_controller/launch/controller.launch`
- `omni_base_controller/README.md`
- `omni_base_controller/STAGE4_REPORT.md`
- `omni_base_controller/CMakeLists.txt`
- `omni_base_controller/package.xml`

## Commands run
- `python3 -B -c "import ast, pathlib; ast.parse(...omni_kinematics.py...)"`
- `bash -n omni_base_controller/scripts/test_cmd_sequence.sh`
- `chmod +x omni_base_controller/scripts/omni_kinematics.py omni_base_controller/scripts/test_cmd_sequence.sh`
- `source /opt/ros/noetic/setup.bash && catkin_make`
- `source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && roslaunch omni_base_controller controller.launch use_rviz:=false`
- `rostopic list`
- `rosnode list`
- `rostopic echo /wheel_velocities`
- `rostopic echo /odom`
- `rostopic hz /odom`
- `rosrun tf tf_echo odom base_footprint`
- `omni_base_controller/scripts/test_cmd_sequence.sh`
- `rosnode info /joint_state_publisher`

## Test results
- `catkin_make`: PASS
- Node launch: PASS
- `/wheel_velocities` published at idle: PASS
- `/odom` published at idle: PASS
- `odom -> base_footprint` TF exists: PASS
- `/odom` rate close to `30 Hz`: PASS
- Test command sequence executes: PASS
- Forward command sample produced equal positive wheel speeds of about `4.17 rad/s`: PASS
- Odom changed during motion and ended near `x = 0.625 m`, `yaw = 1.62 rad`: PASS

## Remaining issues
- During this test session, an external `/joint_state_publisher` node was already running and also publishing `/joint_states`.
- This did not break `/wheel_velocities`, `/odom`, or `odom -> base_footprint`, but it can interfere with wheel-joint visualization through `robot_state_publisher`.
- For clean controller tests, stop other nodes that publish `/joint_states`, especially `robot_description/display.launch`.

## Suggested next steps for Stage 5
- Add a clean wheel-state or encoder interface.
- Decide the long-term odometry ownership boundary with localization.
- Add simulation-side drive integration when Gazebo motion is needed.
- Introduce closed-loop wheel control only after the hardware/simulation interface is stable.
