# Stage 8 Report

Date: 2026-05-01

## 1. Mục tiêu Stage 8

Stage 8 tạo quy trình benchmark SLAM tái lập được trong Gazebo cho robot omni ROS1 Noetic dùng stereo/depth-derived `LaserScan`. Giai đoạn này chỉ tập trung đo và so sánh SLAM; không triển khai navigation, `move_base`, AMCL, ROS2, hoặc EKF-SLAM đầy đủ.

## 2. Thiết kế thực nghiệm

Mỗi trial chạy một thuật toán SLAM duy nhất, ghi rosbag, điều khiển robot bằng một pattern chậm qua `/cmd_vel`, lưu map, rồi xuất metric từ bag/map. ROS state được reset bằng cách khởi động lại launch cho từng trial.

Các topic được record:

```text
/scan
/odom
/tf
/tf_static
/clock
/cmd_vel
/gazebo/model_states
/map hoặc /map_hector
```

## 3. Scenario

Ba scenario được định nghĩa trong `config/benchmark_scenarios.yaml`:

- `corridor_static`: đi thẳng chậm và quay nhẹ, kiểm tra map hành lang/tường.
- `open_room_obstacles`: đi hình chữ nhật/vòng nhỏ, kiểm tra vật cản trong không gian mở.
- `narrow_turn`: đi qua đoạn hẹp/góc cua bằng các lệnh quay chậm, kiểm tra ổn định khi scan FOV hẹp.

Hạn chế hiện tại: cả ba scenario dùng chung `robot_description/worlds/test_arena.world`; sự khác biệt chủ yếu đến từ drive pattern.

## 4. Thuật toán/Cấu hình

Ba cấu hình được định nghĩa trong `config/benchmark_algorithms.yaml`:

- `gmapping_conservative`: Gmapping ổn định, `/map`, frame `map`.
- `gmapping_fast`: Gmapping nhanh hơn cho sweep bag, `/map`, frame `map`.
- `hector`: Hector SLAM, `/map_hector`, frame `hector_map`.

Không chạy Gmapping và Hector trong cùng một trial.

## 5. Metrics

Trajectory:

- ATE RMSE, mean, max, std bằng `evo_ape` nếu có.
- RPE RMSE, mean, max, std bằng `evo_rpe` nếu có.
- Nếu thiếu `evo`, script vẫn tạo CSV với `N/A` và ghi lý do.

Map:

- width, height, resolution, area.
- occupied/free/unknown cell count và ratio.
- occupied IoU cơ bản nếu có ground-truth map cùng kích thước.

Runtime và success:

- `runtime_status.csv` ghi runtime, đường dẫn bag/map, success flag, và notes.

## 6. Quy trình chạy

Kiểm tra dependency:

```bash
rosrun slam_benchmark check_stage8_dependencies.sh
```

Chạy một trial:

```bash
rosrun slam_benchmark run_single_slam_trial.sh corridor_static gmapping_conservative 1
```

Trích xuất trajectory:

```bash
rosrun slam_benchmark extract_gazebo_ground_truth.py <bag_path> omni_robot <gt.tum>
rosrun slam_benchmark extract_slam_tf_trajectory.py <bag_path> map base_footprint <est.tum>
```

Tính ATE/RPE:

```bash
rosrun slam_benchmark compute_ate_rpe.sh <gt.tum> <est.tum> <metrics_dir>
```

Evaluate map:

```bash
rosrun slam_benchmark evaluate_map_basic.py <map.yaml> <map_metrics.csv>
```

Tổng hợp:

```bash
rosrun slam_benchmark aggregate_stage8_results.py
rosrun slam_benchmark plot_stage8_results.py
rosrun slam_benchmark generate_latex_tables.py
```

## 7. Kết quả test script

Kết quả kiểm thử trong môi trường hiện tại:

- Build `catkin_make`: PASS.
- `check_stage8_dependencies.sh`: PASS; `evo` chưa được cài nên ATE/RPE dùng fallback `N/A`.
- Single trial `corridor_static/gmapping_conservative/rep_1`: chạy được Gazebo, `/scan`, Gmapping và lưu map.
- Bag record: FAIL trong lượt test này; `rosbag` subscribe rồi đóng ngay, bag chỉ 4 KB và không có message trajectory. Script đã được chỉnh để chạy `rosbag record` với stdin tách khỏi shell và sửa kiểm tra bag file cho các lượt chạy sau.
- Extract ground truth từ trial bag: FAIL vì bag không có `/gazebo/model_states`.
- Extract SLAM TF từ trial bag: FAIL vì bag không có `/tf`.
- Map evaluation: PASS, `unknown_ratio=0.994846`.
- Aggregate: PASS, tạo `stage8_summary.csv` và `stage8_summary_mean.csv`.
- Plot: PASS cho runtime và unknown ratio; ATE/RPE bar chart bị skip vì chưa có số liệu numeric.
- LaTeX/Markdown table: PASS.

Nếu Gazebo chạy headless và camera không render, `/scan` có thể không publish; dùng `xvfb-run` như Stage 7.

## 8. Bảng kết quả

Kết quả tổng hợp được tạo tại:

```text
results/stage8/csv/stage8_summary.csv
results/stage8/csv/stage8_summary_mean.csv
results/stage8/markdown/table_slam_comparison.md
results/stage8/latex/table_slam_comparison.tex
```

Nếu chưa có trial thành công, bảng sẽ chứa `N/A` thay vì bỏ qua metric.

## 9. Nhận xét ban đầu

Kỳ vọng thực nghiệm:

- Gmapping conservative thường ổn định hơn khi `/scan` nhiễu hoặc FOV hẹp.
- Gmapping fast có thể nhanh hơn nhưng dễ mất chất lượng scan matching.
- Hector có thể phản ứng nhanh với scan nhưng nhạy với FOV và cấu hình frame; cần kiểm tra map frame `hector_map` trong RViz.
- Thuật toán có unknown ratio thấp hơn không nhất thiết tốt hơn nếu map bị méo; cần xem trajectory và map overlay.

## 10. Hạn chế

- Ground truth lấy từ Gazebo model pose, không phải motion capture.
- `/scan` được sinh từ stereo/depth, không phải LiDAR thật.
- Map metric hiện tại chỉ là occupied/free/unknown ratio và IoU cơ bản khi có ground-truth map cùng kích thước.
- FOV và chất lượng stereo point cloud ảnh hưởng mạnh tới cả Gmapping và Hector.
- Các scenario hiện dùng cùng một world, chưa đánh giá đa dạng hình học môi trường.

## 11. Việc cần làm cho Stage 9

- Chạy đủ 3 repetition cho từng scenario/algorithm và điền bảng kết quả cuối.
- Kiểm tra RViz/Gazebo cho từng map: hướng scan, TF, lệch trajectory, map méo.
- Nếu Stage 8 cho map ổn định, mới chuyển sang chuẩn bị navigation/move_base ở giai đoạn sau.
