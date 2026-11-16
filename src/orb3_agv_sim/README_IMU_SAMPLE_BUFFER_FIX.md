# ORB-SLAM3 IMU sample buffer fix

Bản này sửa lỗi `/imu` Gazebo chạy 90-100 Hz nhưng `orb3_imu_rgbd_node` vẫn báo quá ít IMU sample giữa 2 frame camera.

## Các thay đổi chính

1. `src/orb3_imu_rgbd_node_imu.cpp`
   - Dùng `MultiThreadedExecutor(4)` thay vì `rclcpp::spin(node)`.
   - Tạo callback group riêng cho IMU để IMU callback không bị nghẽn bởi `TrackRGBD()` và Viewer.
   - Tăng IMU QoS queue lên `keep_last(2000)`.
   - Tăng buffer IMU nội bộ lên `8000` sample.
   - Không `pop_front()` IMU quá sát thời gian frame hiện tại; giữ lại margin 0.5 s.

2. `launch/orb3_imu_stable_launch.py`
   - Mặc định dùng:
     - `min_imu_boot_samples:=8`
     - `min_imu_between_frames:=2`
     - `enable_wrapper_motion_gate:=false`
     - `reset_on_reject_count:=999`
   - Các tham số này giờ truyền qua `LaunchConfiguration`, nên override từ command line sẽ có tác dụng.

## Lệnh chạy kiểm tra

```bash
ros2 launch orb3_agv_sim orb3_imu_stable_launch.py \
  use_viewer:=true \
  use_sim_time:=true \
  tbc_mode:=manual_ros_optical \
  min_imu_boot_samples:=8 \
  min_imu_between_frames:=2 \
  reset_on_reject_count:=999 \
  enable_wrapper_motion_gate:=false
```

Log đúng nên không còn lặp liên tục:

```text
Too few IMU samples between frames
Waiting IMU warmup: have 3 samples, need >= 8
not IMU meas
```

Nếu vẫn thiếu sample, kiểm tra timestamp:

```bash
ros2 topic echo /imu/header --once
ros2 topic echo /camera/image_raw/header --once
ros2 topic echo /camera/depth/image_raw/header --once
```

Thử offset:

```bash
imu_time_offset_s:=0.01
imu_time_offset_s:=-0.01
imu_time_offset_s:=0.02
imu_time_offset_s:=-0.02
```
