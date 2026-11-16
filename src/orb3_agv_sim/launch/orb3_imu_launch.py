#!/usr/bin/env python3
"""Launch ORB-SLAM3 IMU_RGBD node.

Node này dùng settings_template_file (không phải settings_file).
Node tự tra TF imu_link->camera_link_optical để tính Tbc,
điền vào template rồi ghi ra /tmp/orb3_rgbd_imu_runtime.yaml.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('orb3_agv_sim')
    home = os.path.expanduser('~')

    orb_root = os.path.join(home, 'Dev', 'ORB_SLAM3')

    return LaunchDescription([
        DeclareLaunchArgument('vocabulary_file',
            default_value=os.path.join(orb_root, 'Vocabulary', 'ORBvoc.txt')),

        # ── Template file (node điền Tbc từ TF vào đây) ──────────────────
        DeclareLaunchArgument('settings_template_file',
            default_value=os.path.join(pkg, 'config',
                                       'orb3_rgbd_imu_astra_pro.template.yaml')),

        # ── Runtime file (node tự tạo, KHÔNG cần tạo tay) ────────────────
        DeclareLaunchArgument('runtime_settings_file',
            default_value='/tmp/orb3_rgbd_imu_runtime.yaml'),
        DeclareLaunchArgument('atlas_load_file',
            default_value=''),
        DeclareLaunchArgument('atlas_save_file',
            default_value=''),
        DeclareLaunchArgument('localization_mode',
            default_value='false'),
        DeclareLaunchArgument('single_map_mode',
            default_value='true'),
        DeclareLaunchArgument('use_sim_time',
            default_value='false'),

        DeclareLaunchArgument('rgb_topic',
            default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic',
            default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('imu_topic',
            default_value='/imu/data_raw'),

        DeclareLaunchArgument('world_frame',
            default_value='orb_map'),
        DeclareLaunchArgument('base_frame',
            default_value='Base_footprint'),
        DeclareLaunchArgument('camera_frame',
            default_value='camera_link_optical'),
        DeclareLaunchArgument('pose_topic',
            default_value='/orbslam3/pose'),
        DeclareLaunchArgument('path_topic',
            default_value='/orbslam3/path'),
        DeclareLaunchArgument('camera_pose_topic',
            default_value='/orbslam3/camera_pose'),
        DeclareLaunchArgument('raw_camera_pose_topic',
            default_value='/orbslam3/pose_raw_camera'),
        DeclareLaunchArgument('base_pose_topic',
            default_value='/orbslam3/pose_base'),
        DeclareLaunchArgument('pose_debug_topic',
            default_value='/orbslam3/pose_debug'),
        DeclareLaunchArgument('tracking_state_topic',
            default_value='/orbslam3/tracking_state'),

        # imu_frame: để trống → node tự detect từ topic /imu header.frame_id
        DeclareLaunchArgument('imu_frame',
            default_value=''),

        # Tbc mode for ORB-SLAM3 IMU.
        # manual_ros_optical matches ROS optical camera convention for this AGV.
        # Use tf_body_camera only when a verified camera/IMU extrinsic is available in TF.
        # Options: tf_body_camera, tf_camera_body, manual_ros_optical, manual_inverse_ros_optical.
        DeclareLaunchArgument('tbc_mode',
            default_value='manual_ros_optical'),
        DeclareLaunchArgument('imu_time_offset_s',
            default_value='0.0'),

        DeclareLaunchArgument('use_viewer',
            default_value='true'),
        DeclareLaunchArgument('publish_tf',
            default_value='false'),
        DeclareLaunchArgument('force_planar_output',
            default_value='true'),

        # Real HWT901B IMU is expected at 100 Hz with RGB-D at 10 Hz.
        # A normal frame should carry about 8-12 IMU samples.
        DeclareLaunchArgument('min_imu_boot_samples',
            default_value='8'),
        DeclareLaunchArgument('min_imu_between_frames',
            default_value='2'),

        DeclareLaunchArgument('max_rgb_depth_dt',
            default_value='0.30'),
        DeclareLaunchArgument('target_tracking_hz',
            default_value='10.0'),
        DeclareLaunchArgument('sync_queue_size',
            default_value='30'),

        # Wrapper safety gate: chỉ chặn pose ORB nhảy sai trước khi publish ra ROS.
        # Không patch, không sửa, không build lại core ~/Dev/ORB_SLAM3.
        DeclareLaunchArgument('enable_wrapper_motion_gate',
            default_value='true'),
        DeclareLaunchArgument('max_pose_jump_xy_m',
            default_value='0.50'),
        DeclareLaunchArgument('max_pose_jump_z_m',
            default_value='0.25'),
        DeclareLaunchArgument('max_pose_jump_yaw_deg',
            default_value='35.0'),
        DeclareLaunchArgument('max_roll_pitch_deg',
            default_value='12.0'),
        DeclareLaunchArgument('max_orb_linear_speed_mps',
            default_value='1.50'),
        DeclareLaunchArgument('max_orb_yaw_rate_dps',
            default_value='120.0'),
        DeclareLaunchArgument('reset_on_reject_count',
            default_value='12'),
        DeclareLaunchArgument('min_acc_norm',
            default_value='7.0'),
        DeclareLaunchArgument('max_acc_norm',
            default_value='12.5'),

        Node(
            package='orb3_agv_sim',
            executable='orb3_imu_rgbd_node',
            name='orb3_imu_rgbd_node',
            output='screen',
            parameters=[{
                'vocabulary_file':        LaunchConfiguration('vocabulary_file'),
                # ── TÊN PARAM ĐÚNG theo node .cpp ──
                'settings_template_file': LaunchConfiguration('settings_template_file'),
                'runtime_settings_file':  LaunchConfiguration('runtime_settings_file'),
                'atlas_load_file':        LaunchConfiguration('atlas_load_file'),
                'atlas_save_file':        LaunchConfiguration('atlas_save_file'),
                'localization_mode':      LaunchConfiguration('localization_mode'),
                'single_map_mode':        LaunchConfiguration('single_map_mode'),
                # ─────────────────────────────────────
                'rgb_topic':              LaunchConfiguration('rgb_topic'),
                'depth_topic':            LaunchConfiguration('depth_topic'),
                'imu_topic':              LaunchConfiguration('imu_topic'),
                'world_frame':            LaunchConfiguration('world_frame'),
                'base_frame':             LaunchConfiguration('base_frame'),
                'camera_frame':           LaunchConfiguration('camera_frame'),
                'pose_topic':             LaunchConfiguration('pose_topic'),
                'path_topic':             LaunchConfiguration('path_topic'),
                'camera_pose_topic':      LaunchConfiguration('camera_pose_topic'),
                'raw_camera_pose_topic':  LaunchConfiguration('raw_camera_pose_topic'),
                'base_pose_topic':        LaunchConfiguration('base_pose_topic'),
                'pose_debug_topic':       LaunchConfiguration('pose_debug_topic'),
                'tracking_state_topic':   LaunchConfiguration('tracking_state_topic'),
                'publish_raw_camera_pose': True,
                'publish_base_pose':      True,
                'publish_pose_debug':     True,
                'imu_frame':              LaunchConfiguration('imu_frame'),
                'tbc_mode':               LaunchConfiguration('tbc_mode'),
                'imu_time_offset_s':      LaunchConfiguration('imu_time_offset_s'),
                'use_viewer':             LaunchConfiguration('use_viewer'),
                'publish_tf':             LaunchConfiguration('publish_tf'),
                'force_planar_output':    LaunchConfiguration('force_planar_output'),
                'min_imu_boot_samples':   LaunchConfiguration('min_imu_boot_samples'),
                'min_imu_between_frames': LaunchConfiguration('min_imu_between_frames'),
                'max_rgb_depth_dt':       LaunchConfiguration('max_rgb_depth_dt'),
                'target_tracking_hz':     LaunchConfiguration('target_tracking_hz'),
                'sync_queue_size':        LaunchConfiguration('sync_queue_size'),
                'enable_wrapper_motion_gate': LaunchConfiguration('enable_wrapper_motion_gate'),
                'max_pose_jump_xy_m':     LaunchConfiguration('max_pose_jump_xy_m'),
                'max_pose_jump_z_m':      LaunchConfiguration('max_pose_jump_z_m'),
                'max_pose_jump_yaw_deg':  LaunchConfiguration('max_pose_jump_yaw_deg'),
                'max_roll_pitch_deg':     LaunchConfiguration('max_roll_pitch_deg'),
                'max_orb_linear_speed_mps': LaunchConfiguration('max_orb_linear_speed_mps'),
                'max_orb_yaw_rate_dps':   LaunchConfiguration('max_orb_yaw_rate_dps'),
                'reset_on_reject_count':  LaunchConfiguration('reset_on_reject_count'),
                'min_acc_norm':           LaunchConfiguration('min_acc_norm'),
                'max_acc_norm':           LaunchConfiguration('max_acc_norm'),
                'use_sim_time':           LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
