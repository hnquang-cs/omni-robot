# Stage 9 Report: LiDAR SLAM Baseline

## 1. Goal
Stage 9 adds a simulated 2D LiDAR baseline for stable indoor SLAM in ROS1 Noetic while keeping the stereo-depth pipeline available for comparison.

## 2. Rationale
The current stereo-depth-derived scan can produce maps, but it is noisy, ragged, and prone to scan matching failures. A 2D LiDAR scan is a cleaner baseline for repeatable SLAM and later navigation work. Stereo-depth remains a research branch for SOTA depth comparison and possible auxiliary obstacle integration.

## 3. URDF and Gazebo Changes
- Added `lidar_link` fixed to `base_link`.
- Mounted the LiDAR above the chassis at `x=0.08`, `y=0.0`, `z=0.20`.
- Added a Gazebo `ray` sensor with 360 degree FOV, 720 samples, 10 Hz, `0.15 m` to `10.0 m` range, and small Gaussian noise.
- Published LaserScan on `/lidar/scan` with frame `lidar_link`.
- Kept the stereo camera links and Gazebo camera plugins unchanged.

## 4. Topics and TF
- LiDAR scan: `/lidar/scan`
- LiDAR frame: `lidar_link`
- Base frames: `base_footprint -> base_link`
- Expected SLAM chain: `map -> odom -> base_footprint -> base_link -> lidar_link`
- Stereo scan remains `/scan` for comparison.

## 5. Created Launch and Scripts
- `robot_description/launch/gazebo_lidar_sim.launch`
- `robot_description/rviz/lidar_test.rviz`
- `slam_benchmark/launch/gmapping_lidar.launch`
- `slam_benchmark/launch/gmapping_stereo.launch`
- `slam_benchmark/launch/hector_lidar.launch`
- `slam_benchmark/launch/slam_lidar_full.launch`
- `slam_benchmark/rviz/lidar_slam.rviz`
- `slam_benchmark/scripts/check_lidar_stage9.sh`
- `slam_benchmark/scripts/record_lidar_bag.sh`
- `slam_benchmark/scripts/drive_lidar_mapping_pattern.sh`
- `slam_benchmark/scripts/save_lidar_map.sh`
- `slam_benchmark/scripts/run_stage9_lidar_benchmark.sh`

## 6. Test Results
Fill this table after running the Stage 9 test sequence on the target machine.

| Check | Result | Notes |
|---|---:|---|
| `/lidar/scan` | PASS | ~10 Hz, frame `lidar_link` |
| `/odom` | PASS | ~30 Hz from Gazebo truth odom |
| `/map` | PASS | Gmapping published occupancy grid |
| `base_link -> lidar_link` | PASS | Static TF from URDF |
| `map -> odom` | PASS | Published by Gmapping |
| Save map | PASS | `results/stage9/maps/lidar_gmapping_test.{pgm,yaml}` |
| Benchmark | PASS | One `corridor_static/gmapping_lidar/rep1` trial completed |

## 7. Preliminary LiDAR vs Stereo Comparison
The benchmark runner writes:
- `results/stage9/csv/stage9_summary.csv`
- `results/stage9/csv/stage9_summary_mean.csv`
- `results/stage9/latex/table_lidar_vs_stereo.tex`
- `results/stage9/markdown/table_lidar_vs_stereo.md`
- `results/stage9/plots/`

If the LiDAR map has a lower unknown ratio and fewer scan matching failures than the stereo-depth map, use that as evidence for selecting LiDAR as the main autonomous navigation sensor.

One completed LiDAR trial wrote `stage9_summary.csv` with success `true`; ATE/RPE are `N/A` because `evo` was not available in the active shell during this run.

## 8. Stage 10 Proposal
- Build navigation on the LiDAR map.
- Add AMCL, `move_base`, and TEB/local planner configuration.
- Use stereo-depth as an auxiliary obstacle layer or as a separate SOTA depth-model research path.
