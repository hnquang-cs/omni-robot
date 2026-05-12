# Stage 11 — Frontier-Based Exploration Report

## 1. Goal

Replace the Stage 9 reactive `safe_mapping_driver` motion policy with a
goal-directed exploration strategy: pick the boundary between known-free
and unknown space (a *frontier*) and ask `move_base` to drive there.
Stage 9 keeps existing as the reactive baseline.

## 2. Why frontier-based exploration

- Reactive wandering covers area only by chance; performance varies wildly
  across trials.
- Frontier exploration explicitly targets unknown regions, so coverage
  growth is monotonic and the trial can terminate when no frontiers remain.
- It composes with the existing live-SLAM pipeline (Gmapping) without
  needing AMCL or a static `map_server` map.
- It uses the same TEB-based `move_base` already validated in Stage 10.

## 3. Architecture

```
Gazebo (wide_obstacles world)
  + gazebo_truth_odom        odom -> base_footprint
  + cmd_vel_forward_only_filter  cmd_vel_raw -> cmd_vel
  + gazebo_model_controller  cmd_vel -> Gazebo

  Gmapping  /map (live)  +  map -> odom

  move_base (exploration tuning)
    global_planner: GlobalPlanner with allow_unknown=true
    local planner:  TEB (forward-only)
    output -> /cmd_vel_raw

  explore_lite
    reads /map, finds frontiers, sends ActionGoals to move_base
```

Concrete files:

| Role                          | File |
|-------------------------------|------|
| Full launch                   | `slam_benchmark/launch/frontier_exploration_full.launch` |
| explore_lite launch + params  | `slam_benchmark/launch/explore_lite.launch`, `slam_benchmark/config/explore_lite.yaml` |
| move_base for exploration     | `nav_bringup/launch/move_base_exploration.launch`, `nav_bringup/config/move_base_exploration.yaml`, `nav_bringup/config/global_costmap_exploration.yaml` |
| RViz                          | `slam_benchmark/rviz/frontier_exploration.rviz` |
| Dependency check              | `slam_benchmark/scripts/check_frontier_dependencies.sh` |
| Runtime check                 | `slam_benchmark/scripts/check_frontier_runtime.sh` |
| Bag recording                 | `slam_benchmark/scripts/record_frontier_exploration_bag.sh` |
| Map saving                    | `slam_benchmark/scripts/save_frontier_map.sh` |
| Trial orchestration           | `slam_benchmark/scripts/run_frontier_exploration_trial.sh` |
| Evaluator                     | `slam_benchmark/scripts/evaluate_frontier_exploration.py` |
| Fallback (no explore_lite)    | `slam_benchmark/scripts/simple_frontier_detector.py` |

## 4. Topics and TF

Required topics:

- `/map` (Gmapping)
- `/lidar/scan`
- `/odom`
- `/move_base/status`, `/move_base/goal`, `/move_base/result`
- `/cmd_vel`, `/cmd_vel_raw`

TF chain:

```
map -> odom        published by Gmapping
odom -> base_footprint  published by gazebo_truth_odom
base_footprint -> base_link -> lidar_link  from URDF
```

`AMCL` and `map_server` (static map) are **not** in the tree during
exploration. Both would conflict with Gmapping's authority over `/map` and
`map -> odom`.

## 5. Runtime check expectations

Run `rosrun slam_benchmark check_frontier_runtime.sh` while the full launch
is up. Expected verdict:

| Check                        | Expected |
|------------------------------|----------|
| `/map` has messages          | PASS |
| `/lidar/scan` has messages   | PASS |
| `/odom` has messages         | PASS |
| `/move_base/status`          | PASS |
| TF map -> odom               | PASS |
| TF odom -> base_*            | PASS |
| TF base_* -> lidar_link      | PASS |
| `/move_base` node running    | PASS |
| `/explore` node running      | PASS (or WARN if fallback in use) |
| AMCL not running             | PASS |
| `safe_mapping_driver` absent | PASS |
| `map_server` static absent   | PASS |
| `/explore/frontiers` topic   | PASS (warn if explore_lite missing) |
| `/cmd_vel` activity          | PASS once a frontier goal is accepted |

The current dependency check shows `explore_lite_missing` on this machine
(see `results/stage11_frontier/logs/frontier_dependency_check.log`).

## 6. Trial metrics

`run_frontier_exploration_trial.sh` writes one row to
`results/stage11_frontier/csv/frontier_exploration_metrics.csv`:

- `duration_sec`
- `final_map_global_explored_ratio`
- `final_map_global_unknown_ratio`
- `active_explored_ratio`, `active_unknown_ratio` (when the bag carries `/map`)
- `explored_area_m2`, `active_bbox_area_m2`
- `number_of_frontier_goals_sent`, `number_of_move_base_success`,
  `number_of_move_base_aborted`
- `total_distance_traveled` (from `/odom`)
- `min_lidar_range_over_trial`
- `cmd_vel_active_ratio`
- `final_status` (last `actionlib_msgs/GoalStatus` code)

Missing fields are written as `N/A:<reason>` instead of being omitted.

## 7. Limitations

- `explore_lite` is not yet installed on this workspace; until it is,
  the only way to drive frontier goals is the `simple_frontier_detector`
  fallback, which is intentionally minimal.
- Forward-only kinematics constrain the planner: many frontiers behind the
  robot cost a slow re-orientation. `orientation_scale` is set to 0 to
  reduce that bias.
- `allow_unknown=true` on the global planner makes goals reachable, but
  also lets the planner draw paths through still-unknown cells. Local TEB
  costmap obstacles are the safety net.
- Live `/map` evolves under the planner's feet. Goals can become invalid
  mid-execution; `progress_timeout` (30 s) ensures stuck frontiers get
  blacklisted.

## 8. Next steps

1. Install `ros-noetic-explore-lite` (binary) or build `m-explore` from
   source. Re-run `check_frontier_dependencies.sh` until it reports
   `explore_lite_available`.
2. Run a 300 s trial:
   `DURATION=300 rosrun slam_benchmark run_frontier_exploration_trial.sh`.
3. Compare against a Stage 9 `safe_mapping_driver` trial of equal duration:
   - explored_area_m2 (bigger expected for frontier exploration)
   - active_explored_ratio (higher and more consistent)
   - total_distance_traveled vs explored_area_m2 (efficiency)
   - failed/aborted goal counts (planner robustness)
4. Add aggregated plots under `results/stage11_frontier/plots/` once
   multi-trial data exists.
