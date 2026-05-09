# Discrete 45° Motion Primitive — Stage Report

## 1. Vấn đề ban đầu

Khi chạy:

```
roslaunch omni_base_controller discrete_45_motion_controller.launch
```

- Không có Gazebo mở.
- Không có RViz mở.
- Không có robot, không có `/odom`.
- Log:
  ```
  [WARN]  cannot start turn: no fresh odom
  [INFO]  state IDLE -> ERROR_NO_ODOM
  ```

## 2. Nguyên nhân

`discrete_45_motion_controller.launch` **chỉ chạy controller**. Vì controller
đóng vòng điều khiển yaw từ `/odom` (xem README — `/cmd_vel` không phải angle
command), nó cần `/odom` đến từ một node khác. Khi launch không bật simulation
và không bật `gazebo_truth_odom`, không có ai publish `/odom`, controller bị
lock vào `ERROR_NO_ODOM`.

Đây không phải bug của controller — controller đang làm đúng việc fail-safe.
Sai ở mức launch: thiếu các thành phần phía dưới.

## 3. Sửa

| Thay đổi | File |
|----------|------|
| Bổ sung `odom_timeout_sec`, `/motion_primitive_debug`, recovery `ERROR_NO_ODOM → IDLE`. | `scripts/discrete_45_motion_controller.py` |
| Đồng bộ YAML đúng spec (`forward_speed=0.25`, `odom_timeout_sec=1.0`, …). | `config/discrete_45_motion_controller.yaml` |
| Ghi rõ trong comment rằng launch controller-only cần `/odom` từ ngoài. | `launch/discrete_45_motion_controller.launch` |
| **Thêm launch end-to-end mở Gazebo + odom + controller + RViz.** | `launch/discrete_45_demo.launch` |
| RViz config (Grid, TF, RobotModel, Odometry, LaserScan, fixed=odom). | `rviz/discrete_45_demo.rviz` |
| Forward-only safety + warning trên drive-and-turn cho gazebo model. | `scripts/gazebo_model_cmd_vel.py`, `config/gazebo_model_controller.yaml` |
| Test script ghi CSV `results/discrete_45_test_result.csv`. | `scripts/test_discrete_45_primitives.py` |
| Debug script PASS/FAIL nhanh cho odom/TF/cmd_vel/primitive. | `scripts/debug_discrete45_odom.sh` |
| Cài đặt thêm `rviz/` và debug script. | `CMakeLists.txt` |
| Chương Demo end-to-end. | `README.md` |

## 4. Cách chạy

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash

roslaunch omni_base_controller discrete_45_demo.launch
# terminal 2:
rosrun omni_base_controller debug_discrete45_odom.sh
# terminal 3:
rosrun omni_base_controller test_discrete_45_primitives.py
```

## 5. Kết quả test (template)

Khi simulation đã chạy, kỳ vọng:

| Mục | Kết quả mong đợi |
|-----|------------------|
| Gazebo opened | PASS (gzclient + omni_robot model) |
| RViz opened | PASS (config discrete_45_demo.rviz, fixed_frame=odom) |
| /odom fresh | PASS (gazebo_truth_odom publish ~30 Hz) |
| TURN_LEFT_45 | PASS, delta_deg ≈ +45 (±3°) |
| TURN_RIGHT_45 | PASS, delta_deg ≈ -45 (±3°) |
| FORWARD | PASS, `linear.x > 0`, `linear.y = 0`, `angular.z = 0` |
| STOP | PASS, all-zero Twist |
| no_lateral | PASS, không có `linear.y ≠ 0` |
| no_reverse | PASS, không có `linear.x < 0` |
| no_drive_and_turn | PASS, không trùng nhau |

CSV chi tiết: `omni_base_controller/results/discrete_45_test_result.csv`.

## 6. Hạn chế và bước tiếp theo

- Đang dùng ground-truth odom từ Gazebo. Khi chuyển sang phần cứng cần thay
  bằng odom từ encoder.
- Chưa có giao tiếp phần cứng; demo chỉ là kinematic in Gazebo.
- Chưa tích hợp với `move_base`/TEB. Ý đồ là controller này phục vụ
  motion-primitive benchmark; muốn TEB liên tục thì cần một cầu nối khác
  (chuyển continuous Twist sang chuỗi primitive, hoặc bypass sang
  `cmd_vel_forward_only_filter`).
- Khi `ERROR_NO_ODOM` xảy ra giữa lúc đang quay, controller dừng an toàn nhưng
  không tự re-issue lệnh. Người dùng phải gửi lại primitive nếu muốn tiếp tục.
- Có thể tinh chỉnh thêm `turn_speed`/yaw_tolerance để giảm overshoot tuỳ robot.
