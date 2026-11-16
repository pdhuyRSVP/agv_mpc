# Chay ORB3 RGB-D, ORB3 RGB-D+IMU va ghi trajectory so sanh AMCL

File nay dung cho robot that tren Jetson Nano. Khuyen nghi chay tung he ORB rieng le de nhe may:

- Lan 1: ORB RGB-D + AMCL + logger.
- Lan 2: ORB RGB-D+IMU + AMCL + logger.

Neu chay ca hai ORB cung luc tren Jetson Nano thi rat de lag, mat frame RGB-D va mat tracking.

## 0. Build

```bash
cd ~/agv_mpc
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select orb3_agv_sim agv_navigation \
  --cmake-args -DBUILD_ORB_SLAM3_NODES=ON
source install/setup.bash
```

Neu vua sua EKF/localization thi build them `agv_control`:

```bash
cd ~/agv_mpc
colcon build --symlink-install --packages-select agv_control orb3_agv_sim agv_navigation \
  --cmake-args -DBUILD_ORB_SLAM3_NODES=ON
source install/setup.bash
```

## 1. Terminal localization nen de bat AMCL

Launch nay bat dung thu tu: robot model -> wheel driver -> IMU -> lidar -> EKF `/odom` -> `/scan_filtered` -> map_server/AMCL.

```bash
cd ~/agv_mpc
source install/setup.bash
ros2 launch orb3_agv_sim real_amcl_stack.launch.py \
  map:=~/agv_mpc/src/orb3_agv_sim/maps/my_loop_map.yaml \
  wheel_port:=/dev/ttyUSB1 \
  imu_port:=/dev/ttyUSB2 \
  lidar_port:=/dev/ttyUSB0
```

Kiem tra tung tang. Neu len het cac topic/TF nay thi AMCL moi chay dung:

```bash
ros2 topic hz /wheel_vel
ros2 topic hz /imu
ros2 topic hz /scan
ros2 topic hz /scan_filtered
ros2 topic hz /odom
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo odom Base_footprint
ros2 run tf2_ros tf2_echo map odom
```

Trong RViz tren laptop:

- Fixed Frame: `map`
- Add LaserScan: `/scan_filtered`
- Add Odometry: `/odom`
- Add PoseWithCovariance: `/amcl_pose`
- Bam `2D Pose Estimate` de set vi tri ban dau cho AMCL.

## 2. Terminal camera RGB-D

Neu `real_amcl_stack.launch.py` dang chay roi thi khong can mo lai wheel/IMU/lidar o day. Chi mo camera:

```bash
cd ~/agv_mpc
source install/setup.bash

ros2 launch orb3_agv_sim astra_rgbd_10hz.launch.py
```

Kiem tra camera:

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

## 3. Terminal logger trajectory

Tao thu muc log rieng cho moi lan chay:

```bash
mkdir -p ~/agv_mpc/trajectory_logs/run_rgbd
cd ~/agv_mpc
source install/setup.bash
ros2 launch orb3_agv_sim trajectory_compare_logger.launch.py \
  output_dir:=~/agv_mpc/trajectory_logs/run_rgbd
```

Logger se ghi:

- `orb_rgbd.csv`
- `orb_rgbd_imu.csv`
- `amcl.csv`
- `amcl_tf.csv`

Moi dong co `stamp_sec, x, y, z, qx, qy, qz, qw, yaw_rad`.

## 4A. Chay ORB RGB-D

```bash
cd ~/agv_mpc
source install/setup.bash
ros2 launch orb3_agv_sim orb3_rgbd_compare_real_astra_pro.launch.py
```

Realtime profile mac dinh:

- Camera config: `Camera.fps: 10`
- ORB features: `ORBextractor.nFeatures: 900`
- RViz point cloud: off, `map_points_publish_hz:=0.0`
- RViz graph/keyframes: off, `graph_publish_hz:=0.0`
- Auto localization-only: off, `auto_localization_map_points:=0`

Muon debug RViz point/graph thi chi bat tam thoi:

```bash
ros2 launch orb3_agv_sim orb3_rgbd_compare_real_astra_pro.launch.py \
  map_points_publish_hz:=0.2 \
  graph_publish_hz:=0.1
```

Topic dung de xem:

```bash
ros2 topic echo /orb_rgbd/pose --once
ros2 topic hz /orb_rgbd/pose
```

RViz:

- Fixed Frame: `orb_rgbd_map`
- Path: `/orb_rgbd/path`
- PointCloud2: `/orb_rgbd/map_points`
- MarkerArray: `/orb_rgbd/rviz/markers`
- PoseArray: `/orb_rgbd/rviz/keyframes`

## 4B. Chay ORB RGB-D+IMU

Dung terminal logger moi de khong de len log RGB-D:

```bash
mkdir -p ~/agv_mpc/trajectory_logs/run_rgbd_imu
cd ~/agv_mpc
source install/setup.bash
ros2 launch orb3_agv_sim trajectory_compare_logger.launch.py \
  output_dir:=~/agv_mpc/trajectory_logs/run_rgbd_imu
```

Chay ORB RGB-D+IMU:

```bash
cd ~/agv_mpc
source install/setup.bash
ros2 launch orb3_agv_sim orb3_imu_real_astra_pro.launch.py \
  start_camera:=false \
  start_imu:=false \
  use_viewer:=false
```

`start_imu:=false` vi IMU da duoc mo trong `real_amcl_stack.launch.py`. Neu chay rieng khong dung stack do thi doi thanh `start_imu:=true`.

Topic dung de xem:

```bash
ros2 topic echo /orb_rgbd_imu/pose --once
ros2 topic hz /orb_rgbd_imu/pose
```

RViz:

- Fixed Frame: `orb_imu_map`
- Path: `/orb_rgbd_imu/path`
- Pose: `/orb_rgbd_imu/pose`

## 5. Lai robot de quet/thu du lieu

GUI teleop:

```bash
cd ~/agv_mpc
source install/setup.bash
ros2 run agv_control agv_manual_gui
```

Neu GUI lag, dung keyboard teleop qua SSH:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Di cham de ORB khong mat tracking:

- Toc do tien: `0.05` den `0.10 m/s`.
- Toc do xoay: `0.05` den `0.10 rad/s`.
- Tranh xoay tai cho qua nhanh.
- Camera nhin vao vung co texture, tranh tuong trang/nen trong.

## 6. Kiem tra log

```bash
ls -lh ~/agv_mpc/trajectory_logs/run_rgbd
head ~/agv_mpc/trajectory_logs/run_rgbd/orb_rgbd.csv
head ~/agv_mpc/trajectory_logs/run_rgbd/amcl_tf.csv

ls -lh ~/agv_mpc/trajectory_logs/run_rgbd_imu
head ~/agv_mpc/trajectory_logs/run_rgbd_imu/orb_rgbd_imu.csv
head ~/agv_mpc/trajectory_logs/run_rgbd_imu/amcl_tf.csv
```

De so sanh nhanh, import cac file CSV vao Python/Excel/MATLAB va ve cot `x,y`.
