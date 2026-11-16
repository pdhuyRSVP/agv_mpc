# Chạy ORB-SLAM3 RGB-D + IMU, không sửa core ORB-SLAM3

Package này dùng ORB-SLAM3 có sẵn tại:

```bash
~/Dev/ORB_SLAM3
```

Không cần chạy patch và không cần build lại `~/Dev/ORB_SLAM3`, miễn là đã có:

```bash
~/Dev/ORB_SLAM3/lib/libORB_SLAM3.so
~/Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt
```

World chính mặc định:

```bash
corridor_loop_2rect_inner_top_objects_long.world
```

## Build ROS2 wrapper

```bash
cd ~/agv_imuorb_ekf
source /opt/ros/humble/setup.bash

rm -rf build/orb3_agv_sim install/orb3_agv_sim log

colcon build --symlink-install --packages-select orb3_agv_sim \
  --cmake-args \
  -DBUILD_ORB_SLAM3_NODES=ON \
  -DORB_SLAM3_ROOT=$HOME/Dev/ORB_SLAM3 \
  -DGTSAM_DIR=/opt/ros/humble/lib/cmake/GTSAM

source install/setup.bash
```

## Chạy một lệnh: Gazebo + world chính + ORB3 IMU Viewer

```bash
cd ~/agv_imuorb_ekf
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch orb3_agv_sim agv_corridor_orb3_imu_viewer.launch.py
```

## Teleop

Mở terminal khác:

```bash
source /opt/ros/humble/setup.bash
source ~/agv_imuorb_ekf/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=false
```

## Kiểm tra topic

```bash
ros2 topic hz /imu
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /orbslam3/pose --once
```

Nếu log IMU còn ghi `need >= 20`, bạn đang chạy bản cũ. Hãy build lại package và `source install/setup.bash` lại. Bản này mặc định dùng `min_imu_boot_samples = 7`.
