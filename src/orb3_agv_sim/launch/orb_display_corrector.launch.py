#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('orb_pose_topic', default_value='/orb_rgbd/pose'),
        DeclareLaunchArgument('reference_pose_topic', default_value='/graph_fusion/pose'),
        DeclareLaunchArgument('corrected_pose_topic', default_value='/orb_corrected/pose'),
        DeclareLaunchArgument('corrected_path_topic', default_value='/orb_corrected/path'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('orb_frame', default_value='orb_rgbd_map'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('continuous_correction', default_value='true'),
        DeclareLaunchArgument('correction_alpha', default_value='0.05'),
        DeclareLaunchArgument('max_path_length', default_value='2000'),

        Node(
            package='orb3_agv_sim',
            executable='orb_display_corrector.py',
            name='orb_display_corrector',
            output='screen',
            parameters=[{
                'orb_pose_topic': LaunchConfiguration('orb_pose_topic'),
                'reference_pose_topic': LaunchConfiguration('reference_pose_topic'),
                'corrected_pose_topic': LaunchConfiguration('corrected_pose_topic'),
                'corrected_path_topic': LaunchConfiguration('corrected_path_topic'),
                'map_frame': LaunchConfiguration('map_frame'),
                'orb_frame': LaunchConfiguration('orb_frame'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'continuous_correction': LaunchConfiguration('continuous_correction'),
                'correction_alpha': LaunchConfiguration('correction_alpha'),
                'max_path_length': LaunchConfiguration('max_path_length'),
            }],
        ),
    ])
