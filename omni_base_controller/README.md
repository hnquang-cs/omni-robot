# omni_base_controller

## Purpose
This package will host the omni-drive kinematics, odometry estimation, and the bridge between navigation commands and wheel-level actuation.

## Planned inputs and outputs
- Input: `/cmd_vel`
- Input: wheel feedback or simulated joint feedback
- Output: wheel target topics or motor commands
- Output: `/odom`
- Output: `/tf` from `odom` to `base_footprint` if owned by the controller stage

## Important files
- `scripts/omni_kinematics.py`: placeholder node for the future controller pipeline.
- `launch/controller.launch`: minimal launch entry point.

## Next-stage work
- Implement body-to-wheel kinematics.
- Define the odometry ownership boundary versus `robot_localization`.
- Add configuration for wheel radius, wheelbase, and topic names.
