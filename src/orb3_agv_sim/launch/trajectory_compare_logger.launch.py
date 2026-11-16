#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('orb_rgbd_pose_topic', default_value='/orb_rgbd/pose'),
        DeclareLaunchArgument('orb_rgbd_imu_pose_topic', default_value='/orb_rgbd_imu/pose'),
        DeclareLaunchArgument('amcl_pose_topic', default_value='/amcl_pose'),
        DeclareLaunchArgument('slam_toolbox_pose_topic', default_value='/slam_toolbox/pose'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('fused_pose_topic', default_value='/graph/pose'),
        DeclareLaunchArgument('enable_amcl_tf', default_value='true'),
        DeclareLaunchArgument('amcl_map_frame', default_value='map'),
        DeclareLaunchArgument('amcl_base_frame', default_value='Base_footprint'),
        DeclareLaunchArgument('amcl_tf_hz', default_value='5.0'),

        Node(
            package='orb3_agv_sim',
            executable='trajectory_compare_logger.py',
            name='trajectory_compare_logger',
            output='screen',
            parameters=[{
                'output_dir': LaunchConfiguration('output_dir'),
                'orb_rgbd_pose_topic': LaunchConfiguration('orb_rgbd_pose_topic'),
                'orb_rgbd_imu_pose_topic': LaunchConfiguration('orb_rgbd_imu_pose_topic'),
                'amcl_pose_topic': LaunchConfiguration('amcl_pose_topic'),
                'slam_toolbox_pose_topic': LaunchConfiguration('slam_toolbox_pose_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'fused_pose_topic': LaunchConfiguration('fused_pose_topic'),
                'enable_amcl_tf': LaunchConfiguration('enable_amcl_tf'),
                'amcl_map_frame': LaunchConfiguration('amcl_map_frame'),
                'amcl_base_frame': LaunchConfiguration('amcl_base_frame'),
                'amcl_tf_hz': LaunchConfiguration('amcl_tf_hz'),
            }],
        ),
    ])
