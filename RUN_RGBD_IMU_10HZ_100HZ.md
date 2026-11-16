# RGB-D 10 Hz + HWT901B IMU 100 Hz

Workflow nay chuan hoa pipeline thuc te:

- Astra Pro RGB-D: 640x480 at 10 Hz, depth registered/aligned.
- HWT901B Modbus IMU: raw `/imu/data_raw` at 100 Hz.
- ORB-SLAM3 RGB-D + IMU: camera 10 Hz, IMU 100 Hz.
- EKF robot_localization: dung wheel velocity + raw yaw-rate `gz`, khong fuse AHRS yaw.

## Build

```bash
cd ~/agv_mpc
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select agv_sensors_cpp agv_sensors agv_control orb3_agv_sim \
  --cmake-args -DBUILD_ORB_SLAM3_NODES=ON
source install/setup.bash
```

## Chay rieng camera 10 Hz

```bash
ros2 launch orb3_agv_sim astra_rgbd_10hz.launch.py \
  color_fps:=10 \
  depth_fps:=10
```

Kiem tra topic that neu driver doi ten:

```bash
ros2 topic list | grep -E "camera|color|depth|aligned|registered"
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

## Chay rieng IMU 100 Hz

```bash
ros2 launch agv_sensors imu.launch.py \
  imu_poll_hz:=100.0 \
  imu_topic:=/imu/data_raw \
  imu_publish_orientation:=false \
  imu_read_timeout_ms:=8 \
  imu_write_timeout_ms:=5
```

Kiem tra:

```bash
ros2 topic hz /imu/data_raw
ros2 topic delay /imu/data_raw
ros2 topic echo /imu/data_raw --once
```

`orientation_covariance[0] = -1.0` la dung trong cau hinh mac dinh, nghia la AHRS orientation khong duoc EKF/consumer tin.

## Chay EKF

```bash
ros2 launch agv_control ekf.launch.py
```

EKF lay `/wheel_vel` va `/imu/data_raw`; chi fuse yaw-rate `gz` tu IMU.

## Chay ORB-SLAM3 RGB-D + IMU

Neu camera va IMU chua chay:

```bash
ros2 launch orb3_agv_sim orb3_imu_real_astra_pro.launch.py \
  start_camera:=true \
  start_imu:=true \
  use_viewer:=true \
  target_tracking_hz:=10.0 \
  imu_topic:=/imu/data_raw
```

Neu stack cam bien da chay roi:

```bash
ros2 launch orb3_agv_sim orb3_imu_real_astra_pro.launch.py \
  start_camera:=false \
  start_imu:=false \
  use_viewer:=true \
  target_tracking_hz:=10.0 \
  imu_topic:=/imu/data_raw
```

Log tot se co:

```text
IMU samples for RGB-D frame: 8..12
```

## Kiem tra timing RGB-D/IMU

```bash
cd ~/agv_mpc
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/check_rgbd_imu_timing.py --ros-args \
  -p color_topic:=/camera/color/image_raw \
  -p depth_topic:=/camera/depth/image_raw \
  -p imu_topic:=/imu/data_raw
```

Ket qua tot:

- color: khoang 10 Hz.
- depth: khoang 10 Hz.
- IMU: 95-105 Hz.
- RGB-depth dt trung binh duoi 20-30 ms.
- IMU/frame: khoang 8-12.

## TODO calibration

- Kiem tra lai `Tbc`/`IMU.T_b_c1` bang calibration camera-IMU that.
- Noise/walk IMU nen do bang datasheet/Allan variance. Gia tri hien tai chi la cau hinh dang chay, khong phai calibration cuoi.
