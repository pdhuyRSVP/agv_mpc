#!/usr/bin/env python3
"""Run real Astra Pro RGB-D camera with the ORB-SLAM3 RGB-D + IMU wrapper.

Expected real topics:
  /camera/color/image_raw
  /camera/depth/image_raw
  /imu/data_raw
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    orb_pkg = get_package_share_directory('orb3_agv_sim')
    description_pkg = get_package_share_directory('agv_description')
    urdf_file = os.path.join(description_pkg, 'urdf', 'agv0509test6.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as stream:
        robot_description = stream.read()

    start_camera = LaunchConfiguration('start_camera')
    start_imu = LaunchConfiguration('start_imu')
    use_viewer = LaunchConfiguration('use_viewer')

    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='false'),
        DeclareLaunchArgument('publish_robot_model', default_value='true'),
        DeclareLaunchArgument('use_viewer', default_value='true'),
        DeclareLaunchArgument('camera_x', default_value='0.3875'),
        DeclareLaunchArgument('camera_y', default_value='-0.0030'),
        DeclareLaunchArgument('camera_z', default_value='0.1425'),
        DeclareLaunchArgument('camera_yaw', default_value='-1.57079632679'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_roll', default_value='-1.57079632679'),
        DeclareLaunchArgument('base_frame', default_value='Base_footprint'),
        DeclareLaunchArgument('camera_frame', default_value='color_optical_frame'),
        DeclareLaunchArgument('world_frame', default_value='orb_imu_map'),
        DeclareLaunchArgument('pose_topic', default_value='/orb_rgbd_imu/pose'),
        DeclareLaunchArgument('path_topic', default_value='/orb_rgbd_imu/path'),
        DeclareLaunchArgument('camera_pose_topic', default_value='/orb_rgbd_imu/camera_pose'),
        DeclareLaunchArgument(
            'raw_camera_pose_topic', default_value='/orb_rgbd_imu/pose_raw_camera'),
        DeclareLaunchArgument('base_pose_topic', default_value='/orb_rgbd_imu/pose_base'),
        DeclareLaunchArgument('pose_debug_topic', default_value='/orb_rgbd_imu/pose_debug'),
        DeclareLaunchArgument(
            'tracking_state_topic', default_value='/orb_rgbd_imu/tracking_state'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data_raw'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('atlas_load_file', default_value=''),
        DeclareLaunchArgument('atlas_save_file', default_value=''),
        DeclareLaunchArgument('localization_mode', default_value='false'),
        DeclareLaunchArgument('max_rgb_depth_dt', default_value='0.30'),
        DeclareLaunchArgument('target_tracking_hz', default_value='10.0'),
        DeclareLaunchArgument('imu_time_offset_s', default_value='0.0'),
        DeclareLaunchArgument('tbc_mode', default_value='manual_ros_optical'),
        DeclareLaunchArgument('min_imu_boot_samples', default_value='8'),
        DeclareLaunchArgument('min_imu_between_frames', default_value='2'),
        DeclareLaunchArgument('single_map_mode', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='orb_imu_robot_state_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_robot_model')),
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orb_pkg, 'launch', 'astra_rgbd_10hz.launch.py')),
            condition=IfCondition(start_camera),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_sensors'),
                    'launch',
                    'imu.launch.py',
                ])),
            condition=IfCondition(start_imu),
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='orb_imu_map_anchor_tf',
            arguments=[
                '0.0', '0.0', '0.0',
                '0.0', '0.0', '0.0',
                LaunchConfiguration('world_frame'),
                'orb_imu_map_anchor',
            ],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_astra_color_optical_tf',
            arguments=[
                LaunchConfiguration('camera_x'),
                LaunchConfiguration('camera_y'),
                LaunchConfiguration('camera_z'),
                LaunchConfiguration('camera_yaw'),
                LaunchConfiguration('camera_pitch'),
                LaunchConfiguration('camera_roll'),
                LaunchConfiguration('base_frame'),
                LaunchConfiguration('camera_frame'),
            ],
            output='screen',
        ),

        TimerAction(period=4.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(orb_pkg, 'launch', 'orb3_imu_launch.py')),
                launch_arguments={
                    'use_sim_time': 'false',
                    'use_viewer': use_viewer,
                    'publish_tf': 'false',
                    'settings_template_file': os.path.join(
                        orb_pkg, 'config', 'orb3_rgbd_imu_astra_pro.template.yaml'),
                    'atlas_load_file': LaunchConfiguration('atlas_load_file'),
                    'atlas_save_file': LaunchConfiguration('atlas_save_file'),
                    'localization_mode': LaunchConfiguration('localization_mode'),
                    'rgb_topic': LaunchConfiguration('rgb_topic'),
                    'depth_topic': LaunchConfiguration('depth_topic'),
                    'imu_topic': LaunchConfiguration('imu_topic'),
                    'world_frame': LaunchConfiguration('world_frame'),
                    'pose_topic': LaunchConfiguration('pose_topic'),
                    'path_topic': LaunchConfiguration('path_topic'),
                    'camera_pose_topic': LaunchConfiguration('camera_pose_topic'),
                    'raw_camera_pose_topic': LaunchConfiguration('raw_camera_pose_topic'),
                    'base_pose_topic': LaunchConfiguration('base_pose_topic'),
                    'pose_debug_topic': LaunchConfiguration('pose_debug_topic'),
                    'tracking_state_topic': LaunchConfiguration('tracking_state_topic'),
                    'camera_frame': LaunchConfiguration('camera_frame'),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'tbc_mode': LaunchConfiguration('tbc_mode'),
                    'single_map_mode': LaunchConfiguration('single_map_mode'),
                    'imu_time_offset_s': LaunchConfiguration('imu_time_offset_s'),
                    'force_planar_output': 'true',
                    'min_imu_boot_samples': LaunchConfiguration('min_imu_boot_samples'),
                    'min_imu_between_frames': LaunchConfiguration('min_imu_between_frames'),
                    'target_tracking_hz': LaunchConfiguration('target_tracking_hz'),
                    'max_rgb_depth_dt': LaunchConfiguration('max_rgb_depth_dt'),
                    'sync_queue_size': '50',
                    'enable_wrapper_motion_gate': 'true',
                    'max_pose_jump_xy_m': '0.50',
                    'max_pose_jump_z_m': '0.25',
                    'max_pose_jump_yaw_deg': '35.0',
                    'max_roll_pitch_deg': '12.0',
                    'max_orb_linear_speed_mps': '1.50',
                    'max_orb_yaw_rate_dps': '120.0',
                    'reset_on_reject_count': '12',
                    'min_acc_norm': '7.0',
                    'max_acc_norm': '12.5',
                }.items(),
            )
        ]),
    ])
