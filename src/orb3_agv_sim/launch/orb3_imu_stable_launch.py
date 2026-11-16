#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('orb3_agv_sim')
    return LaunchDescription([
        DeclareLaunchArgument('use_viewer', default_value='true'),
        DeclareLaunchArgument('tbc_mode', default_value='manual_ros_optical'),
        DeclareLaunchArgument('imu_time_offset_s', default_value='0.0'),
        DeclareLaunchArgument('min_imu_boot_samples', default_value='20'),
        DeclareLaunchArgument('min_imu_between_frames', default_value='6'),
        DeclareLaunchArgument('enable_wrapper_motion_gate', default_value='false'),
        DeclareLaunchArgument('reset_on_reject_count', default_value='999'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'orb3_imu_launch.py')),
            launch_arguments={
                'use_viewer': LaunchConfiguration('use_viewer'),
                'publish_tf': 'false',
                'use_sim_time': 'false',
                'tbc_mode': LaunchConfiguration('tbc_mode'),
                'imu_time_offset_s': LaunchConfiguration('imu_time_offset_s'),
                'force_planar_output': 'true',
                'enable_wrapper_motion_gate': LaunchConfiguration('enable_wrapper_motion_gate'),
                'max_pose_jump_xy_m': '0.50',
                'max_pose_jump_z_m': '0.25',
                'max_pose_jump_yaw_deg': '35.0',
                'max_roll_pitch_deg': '12.0',
                'max_orb_linear_speed_mps': '1.50',
                'max_orb_yaw_rate_dps': '120.0',
                'reset_on_reject_count': LaunchConfiguration('reset_on_reject_count'),
                'min_imu_boot_samples': LaunchConfiguration('min_imu_boot_samples'),
                'min_imu_between_frames': LaunchConfiguration('min_imu_between_frames'),
                'min_acc_norm': '7.0',
                'max_acc_norm': '12.5',
            }.items()
        )
    ])
