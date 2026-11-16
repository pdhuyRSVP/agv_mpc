# ORB-SLAM3 RGB-D + IMU stable workflow for AGV

Bản này vẫn giữ ORB-SLAM3 IMU, nhưng thêm các cơ chế chống flip hướng IMU/gravity:

- `tbc_mode` để đổi nhanh hướng Tbc mà không sửa code.
- `manual_ros_optical` là mặc định ổn định cho AGV ROS optical camera.
- Ghi cả `IMU.T_b_c1`, `IMU.T_b_c`, `Tbc` vào runtime YAML.
- IMU noise đã nới để Local-Inertial BA không kéo map quá mạnh.
- `force_planar_output` để pose publish ra ROS luôn ở mặt phẳng.
- `reset_on_reject_count` để reset ORB active map nếu wrapper gate reject liên tục.
- Có launch lưu/load ORB Atlas.

## Build

```bash
cd ~/agv_imuorb_ekf
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install --packages-select orb3_agv_sim \
  --event-handlers console_direct+ \
  --cmake-args \
  -DBUILD_ORB_SLAM3_NODES=ON \
  -DORB_SLAM3_ROOT=/home/trongtuan/Dev/ORB_SLAM3 \
  -DGTSAM_DIR=/opt/ros/humble/lib/cmake/GTSAM
source install/setup.bash
```

Kiểm tra link đúng ORB-SLAM3:

```bash
ldd ~/agv_imuorb_ekf/install/orb3_agv_sim/lib/orb3_agv_sim/orb3_imu_rgbd_node | grep ORB
```

Phải trỏ về:

```text
/home/trongtuan/Dev/ORB_SLAM3/lib/libORB_SLAM3.so
```

## Chạy sim

```bash
cd ~/agv_imuorb_ekf
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch orb3_agv_sim agv_2d_map_first_mapping.launch.py \
  world:=$HOME/agv_imuorb_ekf/src/orb3_agv_sim/worlds/corridor_loop_2rect_inner_top_objects_long.world \
  start_rviz:=true
```

## Chạy ORB-SLAM3 IMU ổn định

```bash
cd ~/agv_imuorb_ekf
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch orb3_agv_sim orb3_imu_stable_launch.py use_viewer:=true
```

Nếu vẫn bị lật hướng, test nhanh các Tbc mode:

```bash
ros2 launch orb3_agv_sim orb3_imu_stable_launch.py tbc_mode:=tf_body_camera
ros2 launch orb3_agv_sim orb3_imu_stable_launch.py tbc_mode:=tf_camera_body
ros2 launch orb3_agv_sim orb3_imu_stable_launch.py tbc_mode:=manual_inverse_ros_optical
```

Chọn mode nào qua được `start VIBA / end VIBA` mà viewer không bị lật.

## Kiểm tra runtime YAML

Sau khi ORB chạy:

```bash
grep -n "Tbc\|IMU.T_b_c1\|IMU.Frequency\|IMU.Noise\|System.LoadAtlas\|System.SaveAtlas" /tmp/orb3_rgbd_imu_runtime.yaml
```

## Lấy Tbc từ TF để so sánh

```bash
ros2 run tf2_tools view_frames
python3 ~/agv_imuorb_ekf/src/orb3_agv_sim/tools/print_tbc_candidates.py \
  --imu-frame imu_link \
  --camera-frame camera_link_optical
```

Nếu frame camera khác, thay đúng tên frame từ `frames.pdf`.

## Lưu ORB Atlas

```bash
mkdir -p ~/agv_imuorb_ekf/orb_atlas
ros2 launch orb3_agv_sim orb3_imu_save_atlas.launch.py \
  atlas_file:=$HOME/agv_imuorb_ekf/orb_atlas/corridor_loop_orb_atlas \
  use_viewer:=true
```

Khi muốn lưu, nhấn `Ctrl+C` ở terminal ORB. Không tắt viewer bằng nút X trước.

## Load ORB Atlas để định vị

```bash
ros2 launch orb3_agv_sim orb3_imu_load_atlas_localization.launch.py \
  atlas_file:=$HOME/agv_imuorb_ekf/orb_atlas/corridor_loop_orb_atlas \
  use_viewer:=true
```

Không truyền đuôi `.osa`.

## Record rosbag không ground truth

```bash
~/agv_imuorb_ekf/src/orb3_agv_sim/tools/record_eval_bag_no_gt.sh \
  ~/agv_imuorb_ekf/bags/eval_orb_ekf_amcl_01
```

## Vẽ quỹ đạo ORB/EKF/AMCL

```bash
python3 ~/agv_imuorb_ekf/src/orb3_agv_sim/tools/plot_trajectories_from_bag_no_gt.py \
  --bag ~/agv_imuorb_ekf/bags/eval_orb_ekf_amcl_01 \
  --output-dir ~/agv_imuorb_ekf/eval_plots_no_gt
```

## Nếu vẫn crash Sophus

Nếu lỗi là `R is not orthogonal` hoặc `SO3::exp omega nan`, đây là lỗi numeric/core ORB-SLAM3. Có script patch nhẹ:

```bash
cd ~/agv_imuorb_ekf/src/orb3_agv_sim
./tools/patch_orbslam3_sophus_numeric_fix.sh ~/Dev/ORB_SLAM3
cd ~/Dev/ORB_SLAM3
./build.sh
```

Sau đó build lại ROS package.
