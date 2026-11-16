#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('laser_scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('odom_topic', default_value='/rf2o/odom'),
        DeclareLaunchArgument('base_frame_id', default_value='Base_footprint'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('init_pose_from_topic', default_value=''),
        DeclareLaunchArgument('freq', default_value='20.0'),

        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': LaunchConfiguration('laser_scan_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'publish_tf': False,
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                'init_pose_from_topic': LaunchConfiguration('init_pose_from_topic'),
                'freq': LaunchConfiguration('freq'),
            }],
        ),
    ])
