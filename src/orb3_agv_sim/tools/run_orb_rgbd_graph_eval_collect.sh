#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${AGV_ORB_ROOT:-$HOME/agv_mpc}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
RUN_DIR="${RUN_DIR:-$ROOT_DIR/eval_runs/orb_rgbd_graph_${RUN_ID}}"
MAP_YAML="${MAP_YAML:-$ROOT_DIR/install/orb3_agv_sim/share/orb3_agv_sim/maps/maze_31.yaml}"
SLAM_TOOLBOX_MAP="${SLAM_TOOLBOX_MAP:-$ROOT_DIR/maps/slam_toolbox/maze_31}"
START_CAMERA="${START_CAMERA:-true}"
START_AMCL_REFERENCE="${START_AMCL_REFERENCE:-true}"
START_SLAM_TOOLBOX_REFERENCE="${START_SLAM_TOOLBOX_REFERENCE:-true}"
START_NAV2="${START_NAV2:-false}"
PUBLISH_GRAPH_TF="${PUBLISH_GRAPH_TF:-false}"

mkdir -p "$RUN_DIR"/{orb,trajectories,bag,post}

set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

echo "RUN_DIR=$RUN_DIR"
echo "MAP_YAML=$MAP_YAML"
echo "SLAM_TOOLBOX_MAP=$SLAM_TOOLBOX_MAP"
echo "START_NAV2=$START_NAV2"
echo "PUBLISH_GRAPH_TF=$PUBLISH_GRAPH_TF"

pids=()
cleanup() {
  echo
  echo "Stopping collection..."
  for pid in "${pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  wait || true

  latest_orb_dir="$(find "$RUN_DIR/orb" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort | tail -n 1 || true)"
  if [[ -n "${latest_orb_dir:-}" ]]; then
    ros2 run orb3_agv_sim export_orb_run_summary.py "$latest_orb_dir" --output-dir "$RUN_DIR/post" || true
  else
    echo "No ORB timestamp output found under $RUN_DIR/orb"
  fi

  echo "Saved run directory: $RUN_DIR"
}
trap cleanup EXIT

ros2 launch orb3_agv_sim orb3_rgbd_real_astra_pro.launch.py \
  start_camera:="$START_CAMERA" \
  use_viewer:=true \
  publish_tf:=false \
  save_outputs:=true \
  output_dir:="$RUN_DIR/orb" \
  unique_output_dir:=true \
  save_keypoint_images:=false \
  pose_topic:=/orbslam3/pose \
  path_topic:=/orbslam3/path \
  camera_pose_topic:=/orbslam3/camera_pose \
  map_points_topic:=/orbslam3/map_points \
  tracked_points_topic:=/orbslam3/tracked_points \
  map_points_publish_hz:=0.5 \
  graph_publish_hz:=0.5 &
pids+=("$!")

sleep 3

ros2 launch orb3_agv_sim graph_factor_localization.launch.py \
  map:="$MAP_YAML" \
  start_amcl_reference:="$START_AMCL_REFERENCE" \
  start_slam_toolbox_reference:="$START_SLAM_TOOLBOX_REFERENCE" \
  slam_toolbox_map_file_name:="$SLAM_TOOLBOX_MAP" \
  start_nav2:="$START_NAV2" \
  publish_tf:="$PUBLISH_GRAPH_TF" \
  orb_pose_topic:=/orbslam3/pose \
  orb_tracking_state_topic:=/orbslam3/tracking_state \
  scan_topic:=/scan_filtered \
  ekf_odom_topic:=/odom &
pids+=("$!")

sleep 3

ros2 launch orb3_agv_sim trajectory_compare_logger.launch.py \
  output_dir:="$RUN_DIR/trajectories" \
  orb_rgbd_pose_topic:=/orbslam3/pose \
  amcl_pose_topic:=/amcl_reference/pose \
  slam_toolbox_pose_topic:=/slam_toolbox/pose \
  fused_pose_topic:=/graph/pose \
  odom_topic:=/odom \
  enable_amcl_tf:=false &
pids+=("$!")

sleep 2

ros2 bag record -o "$RUN_DIR/bag/eval" \
  /tf /tf_static \
  /odom /imu /wheel_vel /scan_filtered \
  /camera/color/image_raw /camera/color/camera_info /camera/depth/image_raw \
  /orbslam3/pose /orbslam3/camera_pose /orbslam3/path \
  /orbslam3/map_points /orbslam3/tracked_points /orbslam3/tracking_state \
  /scan_match/odom /scan_match/map_pose \
  /graph/pose /graph/odom /graph/path /graph/stats \
  /amcl_reference/pose /slam_toolbox/pose \
  /cmd_vel /cmd_vel_nav_raw &
pids+=("$!")

echo
echo "Collection is running."
echo "Drive the robot under supervision: go straight about 10 m, then return home."
echo "Press Ctrl+C in this terminal after the robot is stopped at home."

wait
