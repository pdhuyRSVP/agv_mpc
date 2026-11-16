#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('output_topic', default_value='/scan_match/odom'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='Base_footprint'),
        DeclareLaunchArgument('use_odom_initial_guess', default_value='true'),
        DeclareLaunchArgument('publish_on_icp_failure', default_value='false'),
        DeclareLaunchArgument('min_range', default_value='0.08'),
        DeclareLaunchArgument('max_range', default_value='8.0'),
        DeclareLaunchArgument('angle_stride', default_value='2'),
        DeclareLaunchArgument('max_points', default_value='360'),
        DeclareLaunchArgument('min_points', default_value='80'),
        DeclareLaunchArgument('min_correspondences', default_value='45'),
        DeclareLaunchArgument('max_correspondence_dist', default_value='0.35'),
        DeclareLaunchArgument('icp_iterations', default_value='12'),
        DeclareLaunchArgument('min_scan_dt', default_value='0.08'),
        DeclareLaunchArgument('max_scan_dt', default_value='1.0'),
        DeclareLaunchArgument('max_delta_xy', default_value='1.0'),
        DeclareLaunchArgument('max_delta_yaw', default_value='0.8'),

        Node(
            package='orb3_agv_sim',
            executable='relative_scan_matcher.py',
            name='relative_scan_matcher',
            output='screen',
            parameters=[{
                'scan_topic': LaunchConfiguration('scan_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'use_odom_initial_guess': LaunchConfiguration('use_odom_initial_guess'),
                'publish_on_icp_failure': LaunchConfiguration('publish_on_icp_failure'),
                'min_range': LaunchConfiguration('min_range'),
                'max_range': LaunchConfiguration('max_range'),
                'angle_stride': LaunchConfiguration('angle_stride'),
                'max_points': LaunchConfiguration('max_points'),
                'min_points': LaunchConfiguration('min_points'),
                'min_correspondences': LaunchConfiguration('min_correspondences'),
                'max_correspondence_dist': LaunchConfiguration('max_correspondence_dist'),
                'icp_iterations': LaunchConfiguration('icp_iterations'),
                'min_scan_dt': LaunchConfiguration('min_scan_dt'),
                'max_scan_dt': LaunchConfiguration('max_scan_dt'),
                'max_delta_xy': LaunchConfiguration('max_delta_xy'),
                'max_delta_yaw': LaunchConfiguration('max_delta_yaw'),
            }],
        ),
    ])
