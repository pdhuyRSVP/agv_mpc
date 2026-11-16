#!/usr/bin/env bash
set -euo pipefail
OUT=${1:-$HOME/agv_imuorb_ekf/bags/eval_stereo_imu_orb_ekf_amcl_01}
mkdir -p "$(dirname "$OUT")"
ros2 bag record -o "$OUT" \
  /clock \
  /tf \
  /tf_static \
  /imu \
  /stereo/left/image_raw \
  /stereo/right/image_raw \
  /stereo/left/camera_info \
  /stereo/right/camera_info \
  /scan \
  /scan_filtered \
  /odom/wheel \
  /odometry/filtered \
  /amcl_pose \
  /orbslam3/pose \
  /orbslam3/camera_pose \
  /orbslam3/path \
  /cmd_vel
