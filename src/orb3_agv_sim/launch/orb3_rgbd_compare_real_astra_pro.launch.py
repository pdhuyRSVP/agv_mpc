#!/usr/bin/env python3
"""Run real ORB-SLAM3 RGB-D with comparison-friendly topic names."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('orb3_agv_sim')
    return LaunchDescription([
        DeclareLaunchArgument('use_viewer', default_value='false'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('world_frame', default_value='orb_rgbd_map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='Base_footprint'),
        DeclareLaunchArgument('camera_frame', default_value='color_optical_frame'),
        DeclareLaunchArgument('tf_publish_mode', default_value='world_to_base'),
        DeclareLaunchArgument('force_planar_output', default_value='true'),
        DeclareLaunchArgument('save_outputs', default_value='true'),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('unique_output_dir', default_value='true'),
        DeclareLaunchArgument('keypoint_log_hz', default_value='1.0'),
        DeclareLaunchArgument('save_keypoint_images', default_value='false'),
        DeclareLaunchArgument('max_saved_keypoints', default_value='1200'),
        DeclareLaunchArgument('map_points_publish_hz', default_value='0.2'),
        DeclareLaunchArgument('graph_publish_hz', default_value='0.2'),
        DeclareLaunchArgument('max_published_map_points', default_value='8000'),
        DeclareLaunchArgument('max_published_tracked_points', default_value='1500'),
        DeclareLaunchArgument('publish_robot_model', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'orb3_rgbd_real_astra_pro.launch.py')),
            launch_arguments={
                'world_frame': LaunchConfiguration('world_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'pose_topic': '/orb_rgbd/pose',
                'path_topic': '/orb_rgbd/path',
                'camera_pose_topic': '/orb_rgbd/camera_pose',
                'map_points_topic': '/orb_rgbd/map_points',
                'tracked_points_topic': '/orb_rgbd/tracked_points',
                'rviz_markers_topic': '/orb_rgbd/rviz/markers',
                'rviz_keyframes_topic': '/orb_rgbd/rviz/keyframes',
                'start_camera': LaunchConfiguration('start_camera'),
                'use_viewer': LaunchConfiguration('use_viewer'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'tf_publish_mode': LaunchConfiguration('tf_publish_mode'),
                'force_planar_output': LaunchConfiguration('force_planar_output'),
                'save_outputs': LaunchConfiguration('save_outputs'),
                'output_dir': LaunchConfiguration('output_dir'),
                'unique_output_dir': LaunchConfiguration('unique_output_dir'),
                'keypoint_log_hz': LaunchConfiguration('keypoint_log_hz'),
                'save_keypoint_images': LaunchConfiguration('save_keypoint_images'),
                'max_saved_keypoints': LaunchConfiguration('max_saved_keypoints'),
                'target_tracking_hz': '4.0',
                'max_rgb_depth_dt': '0.60',
                'sync_queue_size': '20',
                'map_points_publish_hz': LaunchConfiguration('map_points_publish_hz'),
                'graph_publish_hz': LaunchConfiguration('graph_publish_hz'),
                'auto_localization_map_points': '0',
                'max_published_map_points': LaunchConfiguration('max_published_map_points'),
                'max_published_tracked_points': LaunchConfiguration('max_published_tracked_points'),
                'publish_robot_model': LaunchConfiguration('publish_robot_model'),
                'max_path_length': '500',
            }.items(),
        ),
    ])
