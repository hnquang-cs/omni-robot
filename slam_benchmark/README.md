# slam_benchmark

## Purpose
This package groups the launch files and scripts used to compare SLAM backends with the virtual scan generated from the stereo perception pipeline.

## Planned inputs and outputs
- Input: `/scan`, `/tf`, and `/odom` when required by the backend.
- Output: `/map`, trajectory estimates, and benchmark artifacts.

## Important files
- `launch/gmapping.launch`: placeholder launch for gmapping experiments.
- `launch/hector.launch`: placeholder launch for hector experiments.
- `scripts/run_benchmark.sh`: future benchmark orchestration wrapper.

## Next-stage work
- Define a repeatable benchmark protocol.
- Add bag replay or simulation automation.
- Store map quality and runtime comparison outputs.
