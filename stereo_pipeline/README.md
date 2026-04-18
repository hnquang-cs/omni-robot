# stereo_pipeline

## Purpose
This package is reserved for the stereo-camera perception chain that will eventually generate depth, point clouds, and a virtual 2D scan for SLAM and navigation.

## Planned inputs and outputs
- Input: stereo image topics and camera info topics from the left and right cameras.
- Intermediate output: disparity, depth image, and point cloud topics.
- Output: `/scan` or an equivalent virtual laser topic.

## Important files
- `launch/stereo_to_scan.launch`: placeholder launch structure for the future stereo stack.
- `config/stereo_params.yaml`: shared topic and filtering parameters.

## Next-stage work
- Integrate `stereo_image_proc`, `depth_image_proc`, and `pointcloud_to_laserscan`.
- Define camera calibration loading and synchronization rules.
- Benchmark scan quality against SLAM and navigation requirements.
