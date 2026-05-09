# Stage 9 Report: Gmapping vs Hector SLAM Benchmark

## Safety Fix Summary

### Problem

The original Stage 9 LiDAR benchmark used a blind `/cmd_vel` drive pattern. In
`corridor_static`, the robot could publish forward velocity for too long while
`near_box` was directly ahead in the arena. This caused collision, unstable
post-contact rotation, noisy red LiDAR markers around the robot in RViz, and
failed benchmark trials.

The Stage 9 world also missed the rear wall that is present in the Stage 10
navigation-fixed arena. That made the Gazebo world and generated map appear
open on one side unless the robot explored enough geometry to compensate.

### Fix

Stage 9 now has a LiDAR-gated mapping driver:

- `scripts/safe_mapping_driver.py`
- `scripts/run_safe_mapping_driver.sh`

The driver subscribes to `/lidar/scan`, publishes only forward-only commands,
sets `linear.y = 0`, clamps yaw rate, stops on scan timeout, and rotates slowly
in place when the front sector is closer than the safety threshold. It writes
`safe_driver_log.csv` and `safe_driver_status.txt` into each trial directory.

The benchmark runner now uses the fixed-world launch and the safe driver:

- `launch/slam_lidar_stage9_fixed_full.launch`
- `scripts/run_stage9_lidar_benchmark.sh`

Each trial checks `/lidar/scan`, `/odom`, and `/gazebo/model_states` before
motion. It also samples the front LiDAR sector and fails early with
`spawn_too_close_to_obstacle` if the spawn pose is unsafe.

Command safety is reinforced with the optional forward-only filter:

- `/cmd_vel_raw` from the safe driver or planner
- `cmd_vel_forward_only_filter.py`
- `/cmd_vel` consumed by `gazebo_model_cmd_vel.py`

For Stage 9 launches, angular velocity is limited to `0.25 rad/s`. The Gazebo
model controller ignores `linear.y` and integrates only forward velocity plus
yaw, so the Stage 9 mapping motion is differential-style rather than holonomic.

### Fixed World

A new world file was added without overwriting the old arena:

- `robot_description/worlds/test_arena_stage9_fixed.world`
- `robot_description/launch/gazebo_lidar_stage9_fixed.launch`

The missing wall was added as `missing_wall_stage9_fixed` at pose
`-0.5 0.0 0.75 0 0 0` with size `0.12 3.0 1.5`. This closes the rear side of
the existing corridor geometry and matches the Stage 10 fixed-world intent
without changing `test_arena.world`.

### Wide Safe Mapping

Wide safe Gmapping assets were added:

- `config/gmapping_lidar_wide_safe.yaml`
- `launch/gmapping_lidar_wide_safe.launch`
- `launch/slam_lidar_wide_safe_full.launch`
- `scripts/create_wide_safe_map.sh`

The wide map bounds are `[-15, -15]` to `[15, 15]` at `0.05 m/cell`, with
slow-motion update thresholds for the safe driver.

## Final Gmapping LiDAR Baseline with Wide Obstacle World

### Motivation

The previous map was small and contained only a few obstacles, so it was not
representative enough for the final indoor navigation baseline. A wider world
with more static clutter was added before returning to Hector/EKF comparisons.

### World

New files:

- `robot_description/worlds/test_arena_wide_obstacles.world`
- `robot_description/launch/gazebo_lidar_wide_obstacles.launch`

The world uses a 12 m x 10 m floor, perimeter walls, a closed rear wall named
`missing_wall_stage9_fixed`, a main corridor, and an open lobby. It contains
7 static boxes named `obstacle_box_01` through `obstacle_box_07`, each with
collision and visual geometry. The robot spawn is `(0.0, 0.0, 0.05)` with
`yaw=0.0`, facing through the corridor into open space.

### Mapping

Gmapping LiDAR uses:

- `config/gmapping_lidar_wide_obstacles.yaml`
- `launch/gmapping_lidar_wide_obstacles.launch`
- `launch/slam_lidar_wide_obstacles_full.launch`
- `scripts/create_gmapping_wide_obstacles_map.sh`

The map bounds remain wide: `xmin=-15`, `ymin=-15`, `xmax=15`, `ymax=15`,
`delta=0.05`. The safe mapping driver has a `wide_obstacles_safe` pattern with
forward-only motion, LiDAR front-sector stopping, bounded rotate attempts, and
CSV logging.

Final map output:

- `results/stage9/maps/gmapping_lidar_wide_obstacles_final.yaml`
- `results/stage9/maps/gmapping_lidar_wide_obstacles_final.pgm`
- `results/stage9/maps/gmapping_lidar_wide_obstacles_final_metrics.csv`

After map creation, the map is copied to:

- `nav_bringup/maps/lidar_baseline_wide_obstacles.yaml`
- `nav_bringup/maps/lidar_baseline_wide_obstacles.pgm`

### Benchmark

The benchmark scenario `wide_obstacles` was added for Gmapping LiDAR:

```bash
./scripts/run_stage9_lidar_benchmark.sh --single wide_obstacles gmapping_lidar 1
./scripts/run_stage9_lidar_benchmark.sh --single wide_obstacles gmapping_lidar 2
./scripts/run_stage9_lidar_benchmark.sh --single wide_obstacles gmapping_lidar 3
```

Each repetition writes artifacts under
`results/stage9/raw/wide_obstacles/gmapping_lidar/rep_<N>/`, including bag,
map, ground truth, estimated trajectory, trajectory metrics, map metrics, safe
driver log, and trial log. Aggregate output includes the wide-obstacles rows in
`results/stage9/csv/slam_comparison_raw.csv` and
`results/stage9/csv/slam_comparison_mean.csv`.

Report tables and plots:

- `results/stage9/markdown/table_gmapping_baseline_wide_obstacles.md`
- `results/stage9/latex/table_gmapping_baseline_wide_obstacles.tex`
- `results/stage9/plots/gmapping_wide_obstacles_map_metrics.png`
- `results/stage9/plots/gmapping_wide_obstacles_runtime.png`

### Verification

Verifier scripts:

- `scripts/verify_wide_obstacles_world.py`
- `scripts/verify_wide_obstacles_map.py`
- `scripts/acceptance_test_gmapping_wide_obstacles.sh`

The world verifier checks that 6 or 7 `obstacle_box_` models exist, that wall
models exist, and that each obstacle has `static=true`, collision, and visual
elements. The map verifier reports width, height, resolution, occupied ratio,
free ratio, and unknown ratio, warning if the map remains too unknown or too
sparse.

### Limitations

The obstacle set is still Gazebo static boxes, and simulated LiDAR is cleaner
than a real sensor. The resulting baseline is suitable for the thesis simulator
report and navigation stack, but real-robot validation remains a later stage.

### Acceptance

Acceptance script:

```bash
roscd slam_benchmark
./scripts/acceptance_test_stage9_safe_mapping.sh
```

The script builds the workspace, launches the fixed wide safe stack, checks
LiDAR/odom/map/Gazebo topics, verifies `base_link -> lidar_link`, runs the safe
driver for 30 seconds, saves a map, evaluates map metrics, and reports:

- `no_collision`
- `map_saved`
- `wall_fixed_world_exists`
- `map_metrics_created`
- `benchmark_artifacts_created`

### Remaining Limitations

The safe driver is a deterministic mapping driver, not a navigation planner. It
does not plan to semantic goals, avoid local minima globally, or optimize map
coverage. Goal-directed navigation remains Stage 10 responsibility.

Simulated LiDAR is cleaner than real LiDAR, so real robot deployment still
requires sensor noise validation, emergency stop handling, and physical speed
limits.

## Benchmark Wrapper Debug Fix

### Problem

`run_safe_mapping_driver.sh corridor_static gmapping_lidar 1` moved the robot
when run directly, but `run_stage9_lidar_benchmark.sh --single corridor_static
gmapping_lidar 1` could leave the robot standing still.

### Root Cause

The wrapper always prepared for `/cmd_vel_raw` when the safety-filter option was
enabled. In the normal attach workflow:

```bash
roslaunch slam_benchmark slam_lidar_full.launch
./scripts/run_stage9_lidar_benchmark.sh --single corridor_static gmapping_lidar 1
```

`slam_lidar_full.launch` does not necessarily start
`cmd_vel_forward_only_filter`, so the controller listens on `/cmd_vel`, not
`/cmd_vel_raw`. The safe driver was therefore able to run while publishing to a
topic that did not drive Gazebo.

Two secondary wrapper issues were fixed during diagnosis:

- `/cmd_vel` publish checks used invalid one-line YAML, causing a false
  preflight failure.
- The first `/cmd_vel` activity checker used simulated time, which could finish
  before seeing wall-clock command traffic.

### Fix

The benchmark wrapper now defaults to attach mode when `/lidar/scan`, `/odom`,
and `/map` already exist. In attach mode it does not launch Gazebo or a second
SLAM node, preventing `/map` and TF conflicts. Use `--launch-system` only when
the script should start its own fixed Stage 9 system.

The wrapper selects the safe-driver command topic from the active graph:

- `/cmd_vel_raw` only when `/cmd_vel_forward_only_filter` is running.
- `/cmd_vel` otherwise.

It logs the exact safe-driver command, safe-driver PID, rosbag PID, SLAM PID or
attach status, topic preflight checks, command activity, map save status, and
runtime status in `trial.log`.

New debug tooling:

- `scripts/check_cmd_vel_activity.py`
- `scripts/debug_stage9_benchmark_wrapper.sh`

`check_cmd_vel_activity.py` uses wall-clock time and writes
`cmd_vel_activity.csv`.

### Test Result

Attach-mode debug wrapper:

```bash
./scripts/debug_stage9_benchmark_wrapper.sh corridor_static gmapping_lidar 1
```

Result: PASS. It selected `/cmd_vel`, detected active commands, and the safe
driver exited with `duration_complete`.

Single benchmark:

```bash
./scripts/run_stage9_lidar_benchmark.sh --single corridor_static gmapping_lidar 1
```

Result: PASS. Key trial log lines:

- `system_mode=attach`
- `Running safe driver: DURATION=45 PATTERN=corridor_safe CMD_TOPIC=/cmd_vel ...`
- `cmd_vel_activity=PASS`
- `safe_driver_exit=0`
- `rosbag_record=PASS`
- `map_save=PASS`
- `trial result success=true runtime=109s notes=complete`

## Experimental Setup

Stage 9 compares two 2D LiDAR SLAM baselines in Gazebo/ROS1 Noetic:

- `gmapping_lidar`
- `hector_lidar`

The benchmark output root is `slam_benchmark/results/stage9/` with this layout:

- `raw/`: per-trial artifacts under `raw/<scenario>/<algorithm>/rep_<N>/`
- `csv/`: raw and mean comparison tables
- `latex/`: LaTeX table
- `markdown/`: Markdown table
- `plots/`: comparison plots
- `maps/`: copied map snapshots for quick inspection

Scenarios:

- `corridor_static`: slow corridor-style forward motion with small yaw sweeps
- `open_room_obstacles`: slow rectangular loop through the arena
- `narrow_turn`: short forward segments and slow turns for turn stability stress

Each scenario/algorithm pair is configured for 3 repetitions. Each trial records:

- `/lidar/scan`
- `/odom`
- `/map`
- `/tf`
- `/tf_static`
- `/clock`
- `/cmd_vel`
- `/gazebo/model_states`

Each completed trial writes:

- `trial.bag`
- `map.yaml`
- `map.pgm`
- `groundtruth.tum`
- `estimated.tum`
- `trajectory_metrics.csv`
- `map_metrics.csv`
- `trial.log`

Hector is launched with `map_frame=hector_map` while remapping its occupancy grid to `/map`, so both algorithms have the same recorded map topic. Trajectory extraction supports `hector_map -> base_footprint` and `hector_map -> base_link` fallback.

Run command:

```bash
rosrun slam_benchmark run_stage9_lidar_benchmark.sh --yes
```

For a single trial:

```bash
rosrun slam_benchmark run_stage9_lidar_benchmark.sh --single corridor_static gmapping_lidar 1
```

## Metrics

The benchmark records these metrics per trial:

- `success`
- `ate_rmse`
- `ate_mean`
- `ate_max`
- `ate_std`
- `rpe_rmse` when `evo_rpe` is available
- `map_occupied_ratio`
- `map_free_ratio`
- `map_unknown_ratio`
- `runtime_sec`

Trajectory metrics are computed by `compute_ate_rpe.sh` from `groundtruth.tum` and `estimated.tum`. Map ratios are computed by `evaluate_map_basic.py` from `map.yaml`/`map.pgm`.

If any metric cannot be computed, the value is written as `N/A` and the reason is recorded in the `notes` column of `slam_comparison_raw.csv` and `slam_comparison_mean.csv`.

## Result Table

Generated outputs:

- `results/stage9/csv/slam_comparison_raw.csv`
- `results/stage9/csv/slam_comparison_mean.csv`
- `results/stage9/latex/table_slam_comparison.tex`
- `results/stage9/markdown/table_slam_comparison.md`
- `results/stage9/plots/bar_ate_rmse_by_algorithm.png`
- `results/stage9/plots/bar_unknown_ratio_by_algorithm.png`
- `results/stage9/plots/bar_runtime_by_algorithm.png`

Current generated table:

| Scenario | Algorithm | Success | ATE RMSE | RPE RMSE | Occupied | Free | Unknown | Runtime |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| corridor_static | gmapping_lidar | 1/3 | 0.0059 | 0.0013 | 0.0028 | 0.0242 | 0.9730 | 115.0000 |
| corridor_static | hector_lidar | 0/3 | N/A | N/A | N/A | N/A | N/A | N/A |
| open_room_obstacles | gmapping_lidar | 0/3 | N/A | N/A | N/A | N/A | N/A | N/A |
| open_room_obstacles | hector_lidar | 0/3 | N/A | N/A | N/A | N/A | N/A | N/A |
| narrow_turn | gmapping_lidar | 0/3 | N/A | N/A | N/A | N/A | N/A | N/A |
| narrow_turn | hector_lidar | 0/3 | N/A | N/A | N/A | N/A | N/A | N/A |

The table above reflects the current workspace state after generating the Stage 9 summary files. Full Gazebo benchmark trials have not been executed in this shell, so the generated CSV explicitly marks missing metrics as `N/A` with `trial_not_run` and missing-file reasons.

## Discussion: Gmapping vs Hector

Gmapping uses wheel odometry plus scan matching and is expected to be more stable in this simulated setup because `/odom` is available from Gazebo truth odometry. It should usually produce lower trajectory drift when odometry is smooth and the LiDAR map has enough structure.

Hector SLAM does not require odometry and relies heavily on scan matching. That makes it useful as an odometry-light baseline, but it can be more sensitive in sparse open areas, during fast rotations, or when scan overlap is low. The benchmark keeps Hector's TF frame as `hector_map`, and the trajectory extractor explicitly supports `hector_map -> base_footprint` and `hector_map -> base_link`.

After running all repetitions, compare:

- lower `ate_rmse` and `ate_mean` for trajectory accuracy
- lower `map_unknown_ratio` for exploration/map coverage
- lower `runtime_sec` for practical execution cost
- `success` count across the 3 repetitions for robustness

The final choice should prioritize success rate first, then ATE RMSE, then unknown map ratio and runtime.

---

## N/A Diagnosis and Fix (2026-05-04)

### Problem: Comparison Table Was Entirely N/A

After completing Stage 9 setup, the `slam_comparison_raw.csv` showed:

- Only `corridor_static/gmapping_lidar/rep_1`: `success=true` but ATE/RPE all `N/A`
- `corridor_static/gmapping_lidar/rep_2`: `fail` due to `safe_driver_failed;blocked_by_obstacle`
- All other 16 trials: `trial_not_run` with every artifact missing

### Root Causes

| Root cause | Affected trials |
|---|---|
| `evo_ape`/`evo_rpe` not installed | ATE/RPE N/A in all ran trials |
| Hector trials never ran | All 9 hector trials `trial_not_run` |
| Gmapping rep_3 and all other scenarios never ran | Missing bag/map/tum/metrics |
| `safe_driver` failed on rep_2 (blocked by obstacle) | rep_2 success=false, partial metrics |
| `algorithm_fields` returned `map` for hector TF frame | Trajectory extraction would fail for attach-mode hector |

### Fixes Applied

#### 1. Python-fallback ATE/RPE (`compute_trajectory_metrics.py`)

New script that computes ATE and RPE directly from TUM files without `evo`:
- Nearest-neighbor timestamp synchronisation (max 0.05s, retry 0.10s)
- ATE: RMSE, mean, max, std of Euclidean position errors
- RPE: delta-1 relative translation errors
- Writes `trajectory_metrics.csv` with `trajectory_metrics_source=python_fallback`
- Handles `<10` samples gracefully with N/A + reason

`compute_ate_rpe.sh` now **always** runs the Python fallback first, then overwrites with evo results if evo is available and succeeds.

#### 2. Audit script (`audit_stage9_na.py`)

New script that reads `slam_comparison_raw.csv` and the raw artifact directories to report:
- Total/success/trial_not_run counts
- Per-category missing artifact counts
- ATE N/A due to evo missing
- Writes `results/stage9/logs/audit_stage9_na_report.md`
- Writes `results/stage9/csv/trials_to_rerun.csv` with reason categories

#### 3. Safe driver obstacle recovery (`safe_mapping_driver.py`)

Improved state machine:
- `forward_speed` reduced from 0.10 to **0.06 m/s**
- `rotate_speed` reduced from 0.15 to **0.12 rad/s**
- `safety_stop_distance` increased from 0.70 to **0.80 m** (detect earlier)
- `critical_stop_distance` lowered from 0.35 to **0.30 m** (more tolerance before aborting)
- `max_rotate_segment` increased from 4.0 to **5.0 s** (longer rotation per attempt)
- `max_rotate_attempts` set to **6**
- `blocked_timeout` increased from 12 to **30 s**
- Critical proximity now requires persistent proximity (>5 s) before aborting with `collision_risk`
- Added `event` column to `safe_driver_log.csv` for debug
- Driver only fails `blocked_by_obstacle` when all rotation attempts exhausted OR blocked_timeout exceeded

#### 4. Hector TF frame fix (`run_stage9_lidar_benchmark.sh`)

`algorithm_fields` corrected:
```bash
hector_lidar) echo "Hector,hector_map,base_footprint,false" ;;
```
Previously returned `map` as the TF frame for hector, causing trajectory extraction to look for a non-existent `map->base_footprint` transform instead of `hector_map->base_footprint`.

#### 5. Model auto-detection (`extract_gazebo_ground_truth.py`)

`model_name` argument accepts `"auto"` (now the default in the benchmark script). Auto-detection:
- Reads first 30 `/gazebo/model_states` messages
- Filters by keywords: omni, robot, mobile, turtlebot, husky, pioneer
- Picks the model with largest XY position range (i.e., the moving robot)
- Logs selection to `trajectory_extract.log`

#### 6. Multi-frame fallback trajectory extraction (`extract_slam_tf_trajectory.py`)

Tries frames in algorithm-appropriate order:
- gmapping: `map`, then `odom`
- hector: `hector_map`, `map`, then `odom`

Child frames: always tries `base_link` and `base_footprint`. Logs all attempts to `trajectory_extract.log`. Notes `estimated_source=odom_not_slam_corrected` when odom fallback is used.

#### 7. Graceful map metrics (`evaluate_map_basic.py`)

`main()` no longer returns exit code 1 when map.yaml is missing. Instead writes `map_metrics.csv` with `N/A` values and `na_reason`. This prevents aggregate from falsely reporting `missing_map_metrics`.

#### 8. Aggregate improvements (`aggregate_stage9_results.py`)

- Distinguishes `trial_not_run` vs `incomplete_trial` vs `missing_artifact`
- Reads `na_reason` from `trajectory_metrics.csv` and `map_metrics.csv` for specific N/A notes
- Does not duplicate notes already in `runtime_status.csv`
- Mean CSV now includes `n_metric_valid_ate` and `n_metric_valid_map`
- Short-circuits notes building for truly missing trials

#### 9. Fix/rerun script (`fix_stage9_na_and_rerun.sh`)

```bash
./scripts/fix_stage9_na_and_rerun.sh --core-only   # corridor_static x both x rep1-3
./scripts/fix_stage9_na_and_rerun.sh --full         # all scenarios
./scripts/fix_stage9_na_and_rerun.sh --single corridor_static gmapping_lidar 1
```
Steps: audit → rerun selected trials → aggregate → post-audit.

### Expected Results After Fix

After running `./scripts/fix_stage9_na_and_rerun.sh --core-only` with Gazebo running:

| What | Expected |
|---|---|
| `corridor_static/gmapping_lidar` success | ≥ 2/3 |
| `corridor_static/hector_lidar` success | ≥ 2/3 (or specific failure reason, not `trial_not_run`) |
| ATE RMSE | Numeric values from Python fallback if evo absent |
| Map metrics | Numeric values for all trials where map.yaml exists |
| `slam_comparison_raw.csv` | No more generic `trial_not_run` for corridor_static |
| `slam_comparison_mean.csv` | `n_metric_valid_ate` ≥ 1 for corridor_static/gmapping |

### Remaining Known Limitations

- `open_room_obstacles` and `narrow_turn`: not run by `--core-only`; use `--full` if needed
- Hector in attach mode requires user to launch `hector_lidar.launch` before running the trial
- RPE may still be N/A if not enough synchronized pairs exist (`not_enough_pairs`)
- ATE via Python fallback is a simple position-only metric (no SE3 alignment); evo's aligned ATE is more accurate
