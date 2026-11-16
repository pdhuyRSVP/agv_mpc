#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('orb3_agv_sim')
    home = os.path.expanduser('~')
    return LaunchDescription([
        DeclareLaunchArgument('atlas_file', default_value=os.path.join(home, 'agv_imuorb_ekf', 'orb_atlas', 'corridor_loop_orb_atlas')),
        DeclareLaunchArgument('use_viewer', default_value='true'),
        DeclareLaunchArgument('tbc_mode', default_value='manual_ros_optical'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'orb3_imu_launch.py')),
            launch_arguments={
                'use_viewer': LaunchConfiguration('use_viewer'),
                'publish_tf': 'false',
                'tbc_mode': LaunchConfiguration('tbc_mode'),
                'atlas_load_file': LaunchConfiguration('atlas_file'),
                'atlas_save_file': '',
                'localization_mode': 'true',
                'force_planar_output': 'true',
                'enable_wrapper_motion_gate': 'true',
                'reset_on_reject_count': '12',
            }.items()
        )
    ])
