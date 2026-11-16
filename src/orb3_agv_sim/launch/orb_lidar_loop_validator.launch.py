#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='Base_footprint'),
        DeclareLaunchArgument('orb_pose_topic', default_value='/orbslam3/pose'),
        DeclareLaunchArgument('lidar_odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('max_xy_error_m', default_value='0.50'),
        DeclareLaunchArgument('max_yaw_error_deg', default_value='10.0'),
        DeclareLaunchArgument('auto_align_orb_to_lidar', default_value='true'),
        DeclareLaunchArgument('use_tf_lidar_pose', default_value='true'),

        Node(
            package='orb3_agv_sim',
            executable='orb_lidar_loop_validator',
            name='orb_lidar_loop_validator',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_frame': LaunchConfiguration('map_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'orb_pose_topic': LaunchConfiguration('orb_pose_topic'),
                'lidar_odom_topic': LaunchConfiguration('lidar_odom_topic'),
                'max_xy_error_m': LaunchConfiguration('max_xy_error_m'),
                'max_yaw_error_deg': LaunchConfiguration('max_yaw_error_deg'),
                'auto_align_orb_to_lidar': LaunchConfiguration('auto_align_orb_to_lidar'),
                'use_tf_lidar_pose': LaunchConfiguration('use_tf_lidar_pose'),
            }]
        )
    ])
