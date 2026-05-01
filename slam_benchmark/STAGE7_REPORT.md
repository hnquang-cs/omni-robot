# Stage 7 Report

Date: 2026-04-28

## Initial Problem

The Stage 6 simulation produced `/scan`, `/odom`, TF, and `/map`, but the Gazebo model did not move when `/cmd_vel` was published. Initial diagnosis:

- `/scan`: available at about 4.5-5 Hz.
- `/odom`: available at about 30 Hz.
- `/map`: available after Gmapping started.
- `/gazebo/model_states`: included `omni_robot`.
- `/cmd_vel` yaw command changed neither Gazebo model pose nor yaw.

Raw initial diagnosis:

```text
slam_benchmark/results/stage7_initial_raw.txt
slam_benchmark/results/stage7_initial_diagnosis.md
```

## Changes Implemented

- Added `gazebo_model_cmd_vel.py`: kinematic Gazebo model controller using `/gazebo/set_model_state`.
- Added `gazebo_truth_odom.py`: publishes `/odom` and optional `odom -> base_footprint` TF from `/gazebo/model_states`.
- Added launch/config files for both new nodes.
- Added `publish_odom` parameter to `omni_kinematics.py`; Stage 7 full launch disables kinematic `/odom` and TF when Gazebo truth odom is active.
- Added SLAM-specific `pointcloud_to_laserscan` profile and launch.
- Added Gmapping conservative and fast tuned profiles.
- Updated Hector launch to use `hector_map` frame and `/map_hector` topic.
- Added rosbag record/replay scripts, map save script, map evaluation CSV export, CSV comparison, and Stage 7 system check.
- Added `rviz/slam_tuning.rviz`.

## Architecture

```text
/cmd_vel
  -> omni_kinematics.py
       -> /joint_states, /wheel_velocities
  -> gazebo_model_cmd_vel.py
       -> /gazebo/set_model_state
       -> Gazebo model motion

/gazebo/model_states
  -> gazebo_truth_odom.py
       -> /odom
       -> TF odom -> base_footprint

Gazebo stereo camera
  -> stereo pipeline
  -> /scan
  -> Gmapping or Hector
  -> /map or /map_hector
```

No navigation, AMCL, real robot bringup, or EKF-SLAM was added in this stage.

## Commands Tested

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
xvfb-run -a roslaunch slam_benchmark slam_sim_full.launch
rosrun slam_benchmark check_stage7_system.sh
rosrun slam_benchmark save_map_with_prefix.sh gmapping_stage7_test
rosrun slam_benchmark evaluate_map_basic.py slam_benchmark/maps/gmapping_stage7_test_20260428_231035.yaml
rosrun slam_benchmark compare_map_csv.py slam_benchmark/results/gmapping_stage7_test_20260428_231035_basic_metrics_20260428_231041.csv
timeout --signal=SIGINT 8 rosrun slam_benchmark record_stage7_bag.sh stage7_test_short
EXIT_AFTER_PLAY=true rosrun slam_benchmark replay_with_gmapping.sh slam_benchmark/bags/stage7_test_short.bag conservative
EXIT_AFTER_PLAY=true rosrun slam_benchmark replay_with_hector.sh slam_benchmark/bags/stage7_test_short.bag
```

The environment had no `DISPLAY`, so full simulation tests used `xvfb-run`.

## Test Results

- Build: PASS.
- Gazebo movement: PASS. `check_stage7_system.sh` measured `model_dyaw=0.3750`.
- `/odom`: PASS. `/odom` ran at about 30 Hz and matched the same yaw delta.
- `/scan`: PASS. `/scan` ran at about 4 Hz in the tested headless run.
- `/map`: PASS. `/map` published and was saved.
- TF: PASS for `odom -> base_footprint` and `map -> odom`.
- Save map: PASS.
- Bag record: PASS. `bags/stage7_test_short.bag`, 3.3 MB, 7270 messages.
- Basic map evaluation: PASS.
- Gmapping bag replay: PASS smoke test with `EXIT_AFTER_PLAY=true`.
- Hector bag replay: PASS smoke test with `EXIT_AFTER_PLAY=true`.

Raw Stage 7 test output:

```text
slam_benchmark/results/stage7_test_raw.txt
```

## Saved Map and CSV

```text
slam_benchmark/maps/gmapping_stage7_test_20260428_231035.pgm
slam_benchmark/maps/gmapping_stage7_test_20260428_231035.yaml
slam_benchmark/results/gmapping_stage7_test_20260428_231035_basic_metrics_20260428_231041.csv
slam_benchmark/results/stage7_map_comparison.csv
```

Basic map metrics from the short smoke test:

```text
occupied=35
free=685
unknown=146736
occupied_ratio=0.000237
free_ratio=0.004645
unknown_ratio=0.995117
```

The high unknown ratio is expected for a very short yaw-only smoke test and should not be treated as final map quality.

## Remaining Issues

- Gmapping still reports weak/failed scan matching during many early updates. This is expected with a stereo-derived, sparse/narrow effective scan and little trajectory excitation.
- Full launch shutdown under Xvfb/Gazebo Classic can print X11/Gazebo abort messages after Ctrl-C. Runtime tests completed before shutdown.
- A useful thesis map still requires a deliberate mapping trajectory, not only the short smoke-test rotation used here.

## Stage 8 Recommendations

- Record longer bags with slow forward, lateral, and rotational loops.
- Compare Gmapping conservative, Gmapping fast, and Hector on the same bag.
- Add trajectory and ground-truth map comparison if time permits.
- Only after repeatable map quality is acceptable, proceed to navigation/move_base planning.
