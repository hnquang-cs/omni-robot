# nav_bringup

## Purpose
This package stores navigation launch files and configuration placeholders for move_base, costmaps, and the TEB local planner.

## Planned inputs and outputs
- Input: `/scan`, `/odom`, `/tf`, and later `/map`.
- Input: user goals from `/move_base_simple/goal` or navigation actions.
- Output: `/cmd_vel` for the omni base controller.

## Important files
- `launch/navigation.launch`: main bringup entry point for navigation.
- `config/costmap_common.yaml`: shared obstacle and footprint settings.
- `config/local_costmap.yaml`: local costmap layout.
- `config/global_costmap.yaml`: global costmap layout.
- `config/teb_local_planner_params.yaml`: initial TEB parameter placeholders.

## Next-stage work
- Add map server and localization ownership.
- Tune costmaps for indoor omni motion.
- Connect planner outputs to the controller package and simulation.
