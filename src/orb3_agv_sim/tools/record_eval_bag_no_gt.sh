#!/usr/bin/env bash
set -euo pipefail
OUT="${1:-$HOME/agv_imuorb_ekf/bags/eval_orb_ekf_amcl_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "$OUT")"
ros2 bag record -o "$OUT" \
  /clock /tf /tf_static \
  /imu /scan /scan_filtered /odom/wheel /odometry/filtered \
  /amcl_pose /orbslam3/pose /orbslam3/camera_pose /orbslam3/path /cmd_vel
