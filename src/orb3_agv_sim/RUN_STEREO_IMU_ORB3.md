# ORB-SLAM3 Stereo + IMU workflow for AGV Gazebo

This package version switches the visual-inertial path from RGB-D-Inertial to Stereo-Inertial.
The old RGB-D topics are kept for compatibility, but the new ORB node uses only:

```text
/stereo/left/image_raw
/stereo/right/image_raw
/imu
```

## New code structure

```text
src/orb3_agv_sim/
├── src/
│   ├── orb3_stereo_imu_node.cpp              # NEW: ORB_SLAM3::System::IMU_STEREO + TrackStereo()
│   ├── orb3_imu_rgbd_node_imu.cpp            # Old RGB-D + IMU wrapper kept for fallback
│   ├── orb3_rgbd_node.cpp                    # RGB-D only wrapper kept for fallback
│   └── orb_lidar_loop_validator.cpp
├── launch/
│   ├── orb3_stereo_imu_launch.py             # NEW: generic Stereo-IMU launch
│   ├── orb3_stereo_imu_stable_launch.py      # NEW: stable defaults
│   ├── orb3_stereo_imu_save_atlas.launch.py  # NEW: save Stereo-IMU Atlas
│   └── orb3_stereo_imu_load_atlas_localization.launch.py
├── config/
│   └── orb3_stereo_imu_gz.template.yaml      # NEW: Stereo-Inertial YAML
├── urdf/
│   └── agv.urdf                              # UPDATED: stereo left/right cameras + IMU 100 Hz
└── tools/
    ├── record_eval_bag_stereo_no_gt.sh
    └── restore_orbslam3_core_from_planar_patch_backup.sh
```

## Install / replace

```bash
cd ~
mv agv_imuorb_ekf agv_imuorb_ekf_backup_$(date +%Y%m%d_%H%M%S)
unzip ~/Downloads/agv_imuorb_ekf_stereo_imu_full.zip -d ~
```

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

Check executable:

```bash
ros2 pkg executables orb3_agv_sim | grep stereo
```

Expected:

```text
orb3_agv_sim orb3_stereo_imu_node
```

## Run simulation

```bash
cd ~/agv_imuorb_ekf
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch orb3_agv_sim agv_2d_map_first_mapping.launch.py \
  world:=$HOME/agv_imuorb_ekf/src/orb3_agv_sim/worlds/corridor_loop_2rect_inner_top_objects_long.world \
  start_rviz:=true
```

Check stereo and IMU topics:

```bash
ros2 topic hz /stereo/left/image_raw
ros2 topic hz /stereo/right/image_raw
ros2 topic hz /imu
```

Expected:

```text
/stereo/left/image_raw   about 15 Hz
/stereo/right/image_raw  about 15 Hz
/imu                     about 100 Hz
```

## Run ORB-SLAM3 Stereo + IMU

```bash
cd ~/agv_imuorb_ekf
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch orb3_agv_sim orb3_stereo_imu_stable_launch.py use_viewer:=true
```

Or explicitly:

```bash
ros2 launch orb3_agv_sim orb3_stereo_imu_launch.py \
  use_viewer:=true \
  use_sim_time:=true \
  tbc_mode:=manual_ros_optical \
  min_imu_boot_samples:=8 \
  min_imu_between_frames:=2 \
  reset_on_reject_count:=30 \
  enable_wrapper_motion_gate:=true
```

Runtime config check:

```bash
grep -n "Camera1.fx\|Camera2.fx\|Stereo.T_c1_c2\|IMU.T_b_c1\|IMU.Frequency" \
  /tmp/orb3_stereo_imu_runtime.yaml
```

## Save Atlas

```bash
mkdir -p ~/agv_imuorb_ekf/orb_atlas

ros2 launch orb3_agv_sim orb3_stereo_imu_save_atlas.launch.py \
  atlas_file:=$HOME/agv_imuorb_ekf/orb_atlas/corridor_loop_stereo_imu_atlas \
  use_viewer:=true
```

Stop with Ctrl+C in the ORB terminal to trigger Atlas save.

## Load Atlas for localization

```bash
ros2 launch orb3_agv_sim orb3_stereo_imu_load_atlas_localization.launch.py \
  atlas_file:=$HOME/agv_imuorb_ekf/orb_atlas/corridor_loop_stereo_imu_atlas \
  use_viewer:=true
```

Pass atlas path without `.osa`.

## Record bag

```bash
~/agv_imuorb_ekf/src/orb3_agv_sim/tools/record_eval_bag_stereo_no_gt.sh \
  ~/agv_imuorb_ekf/bags/eval_stereo_imu_01
```

## Important note about old core patches

If the ORB-SLAM3 core still prints `PLANAR_AGV`, rollback the old experimental patch before testing stereo:

```bash
~/agv_imuorb_ekf/src/orb3_agv_sim/tools/restore_orbslam3_core_from_planar_patch_backup.sh ~/Dev/ORB_SLAM3
cd ~/Dev/ORB_SLAM3/build
make -j1
```

Then rebuild the ROS package.
