# Discrete 45° Motion Primitive — Diagnosis

## Triệu chứng ban đầu

Khi chạy:

```
roslaunch omni_base_controller discrete_45_motion_controller.launch
```

log thấy:

```
[INFO]  mode=primitive primitive=/motion_primitive_cmd raw=/cmd_vel_raw out=/cmd_vel odom=/odom forward=0.350 turn=0.250 angle=45.0deg tol=2.0deg
[WARN]  cannot start turn: no fresh odom
[INFO]  state IDLE -> ERROR_NO_ODOM
```

Không có Gazebo, không có RViz, không có robot, không có `/odom`.

## Khảo sát file hiện có

| File | Tồn tại | Vai trò |
|------|---------|---------|
| `omni_base_controller/scripts/discrete_45_motion_controller.py` | ✅ | Node controller chính, đã có closed-loop quay theo `/odom` yaw. |
| `omni_base_controller/config/discrete_45_motion_controller.yaml` | ✅ | Config, nhưng `forward_speed=0.35`, không có `odom_timeout_sec`, thiếu state/debug topic params. |
| `omni_base_controller/launch/discrete_45_motion_controller.launch` | ✅ | **CHỈ chạy controller**, không kèm Gazebo/odom/RViz → root cause của `ERROR_NO_ODOM`. |
| `omni_base_controller/scripts/gazebo_model_cmd_vel.py` | ✅ | Tích phân `/cmd_vel` rồi gọi `/gazebo/set_model_state`. Đã forward-only (vy=0). |
| `omni_base_controller/launch/gazebo_model_controller.launch` | ✅ | Launch riêng node trên. |
| `omni_base_controller/scripts/gazebo_truth_odom.py` | ✅ | Publish `/odom` + TF `odom→base_footprint` từ `/gazebo/model_states`. |
| `omni_base_controller/launch/gazebo_truth_odom.launch` | ✅ | Launch riêng node trên. |
| `omni_base_controller/scripts/test_discrete_45_primitives.py` | ✅ | Test script đã có sẵn, sẽ được giữ và bổ sung CSV results. |
| `omni_base_controller/scripts/monitor_discrete_motion.py` | ✅ | Monitor utility (giữ nguyên). |
| `robot_description/launch/gazebo_lidar_wide_obstacles.launch` | ✅ | Mở Gazebo + spawn robot có LiDAR (`/lidar/scan`). |
| `slam_benchmark/launch/slam_lidar_wide_obstacles_full.launch` | ✅ | SLAM full stack — không cần cho discrete demo. |

## Cấu hình hệ thống tham chiếu

- Model name trong Gazebo: `omni_robot`.
- Frame robot:
  - `odom` ← `base_footprint` ← `base_link` ← `lidar_link`.
- Topic LiDAR: `/lidar/scan`.
- Topic odom: `/odom` (publish bởi `gazebo_truth_odom`).
- Topic cmd_vel: `/cmd_vel` (controller publish, gazebo_model_cmd_vel subscribe).
- Topic primitive: `/motion_primitive_cmd` (`std_msgs/String`).
- Topic state: `/motion_primitive_state` (`std_msgs/String`).
- Sim time: `/use_sim_time = true`.

## Kết luận về nguyên nhân

1. Launch `discrete_45_motion_controller.launch` chỉ chạy node controller — không bật Gazebo, không bật `gazebo_truth_odom`, vì vậy `/odom` không tồn tại → controller chuyển sang `ERROR_NO_ODOM`.
2. Controller **đúng** khi từ chối quay nếu chưa có `/odom`, nhưng cần phục hồi tự động khi `/odom` quay lại.
3. Cần một launch tổng (`discrete_45_demo.launch`) gom: Gazebo+robot+lidar, `gazebo_truth_odom`, `gazebo_model_cmd_vel`, `discrete_45_motion_controller`, RViz.

## Hành động sửa

- Bổ sung `odom_timeout_sec`, `/motion_primitive_debug` topic, recovery `ERROR_NO_ODOM → IDLE` cho controller.
- Cập nhật YAML đúng spec (`forward_speed=0.25`, `odom_timeout_sec=1.0`, …).
- Giữ launch controller riêng nhưng ghi rõ trong comment rằng nó cần `/odom` đã có sẵn.
- Tạo `discrete_45_demo.launch` mở end-to-end + `discrete_45_demo.rviz`.
- Cập nhật `gazebo_model_cmd_vel.py` để clamp lateral về 0, log warning khi nhận drive-and-turn (an toàn cho demo discrete).
- Tạo `debug_discrete45_odom.sh` để chẩn đoán nhanh.
- Bổ sung CSV results cho `test_discrete_45_primitives.py`.
