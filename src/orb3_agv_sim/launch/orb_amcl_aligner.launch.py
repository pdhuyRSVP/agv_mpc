#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('orb_pose_topic', default_value='/orb_rgbd/pose'),
        DeclareLaunchArgument('amcl_pose_topic', default_value='/amcl_pose'),
        DeclareLaunchArgument('output_pose_topic', default_value='/orb_rgbd_aligned/pose'),
        DeclareLaunchArgument(
            'output_pose_cov_topic',
            default_value='/orb_rgbd_aligned/pose_with_covariance'),
        DeclareLaunchArgument('output_path_topic', default_value='/orb_rgbd_aligned/path'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('orb_frame', default_value='orb_rgbd_map'),
        DeclareLaunchArgument('publish_map_to_orb_tf', default_value='true'),
        DeclareLaunchArgument('max_path_length', default_value='2000'),
        DeclareLaunchArgument('cov_xy', default_value='0.04'),
        DeclareLaunchArgument('cov_yaw', default_value='0.04'),

        Node(
            package='orb3_agv_sim',
            executable='orb_amcl_aligner.py',
            name='orb_amcl_aligner',
            output='screen',
            parameters=[{
                'orb_pose_topic': LaunchConfiguration('orb_pose_topic'),
                'amcl_pose_topic': LaunchConfiguration('amcl_pose_topic'),
                'output_pose_topic': LaunchConfiguration('output_pose_topic'),
                'output_pose_cov_topic': LaunchConfiguration('output_pose_cov_topic'),
                'output_path_topic': LaunchConfiguration('output_path_topic'),
                'map_frame': LaunchConfiguration('map_frame'),
                'orb_frame': LaunchConfiguration('orb_frame'),
                'publish_map_to_orb_tf': LaunchConfiguration('publish_map_to_orb_tf'),
                'max_path_length': LaunchConfiguration('max_path_length'),
                'cov_xy': LaunchConfiguration('cov_xy'),
                'cov_yaw': LaunchConfiguration('cov_yaw'),
            }],
        ),
    ])
