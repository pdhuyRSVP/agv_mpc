#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        ('map_topic', '/map'),
        ('scan_topic', '/scan_filtered'),
        ('initial_pose_topic', '/graph/pose'),
        ('output_topic', '/scan_match/map_pose'),
        ('map_frame', 'map'),
        ('base_frame', 'Base_footprint'),
        ('min_update_period', '0.30'),
        ('search_xy', '0.40'),
        ('search_yaw', '0.261799'),
        ('max_mean_distance', '0.22'),
        ('min_valid_fraction', '0.60'),
        ('max_correction_xy', '0.55'),
        ('max_correction_yaw', '0.349066'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in arguments] +
        [Node(
            package='orb3_agv_sim',
            executable='scan_to_map_matcher.py',
            name='scan_to_map_matcher',
            output='screen',
            parameters=[{
                name: LaunchConfiguration(name) for name, _ in arguments
            }],
        )]
    )
