#!/usr/bin/env python3
"""Launch ORB-SLAM3 RGB-D node.

Subscribes to:
    /camera/color/image_raw
    /camera/depth/image_raw
    /camera/color/camera_info

Publishes:
    /orbslam3/camera_pose   geometry_msgs/PoseStamped (camera pose in orb_map)
    /orbslam3/pose          geometry_msgs/PoseStamped (robot base pose)
    /orbslam3/path          nav_msgs/Path             (trajectory)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('orb3_agv_sim')

    default_vocab = os.path.expanduser('~/Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt')
    default_settings = os.path.join(pkg_share, 'config', 'orb3_rgbd_astra_pro.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('vocabulary_file', default_value=default_vocab),
        DeclareLaunchArgument('settings_file',   default_value=default_settings),

        DeclareLaunchArgument('rgb_topic',   default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),

        DeclareLaunchArgument('world_frame',  default_value='orb_map'),
        DeclareLaunchArgument('base_frame',   default_value='Base_footprint'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link_optical'),

        DeclareLaunchArgument('use_viewer',   default_value='false'),
        DeclareLaunchArgument('publish_tf',   default_value='false'),

        DeclareLaunchArgument('sync_queue_size',  default_value='30'),
        DeclareLaunchArgument('max_rgb_depth_dt', default_value='0.04'),

        Node(
            package='orb3_agv_sim',
            executable='orb3_rgbd_node',
            name='orb3_rgbd_node',
            output='screen',
            parameters=[{
                'vocabulary_file':   LaunchConfiguration('vocabulary_file'),
                'settings_file':     LaunchConfiguration('settings_file'),
                'rgb_topic':         LaunchConfiguration('rgb_topic'),
                'depth_topic':       LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'world_frame':       LaunchConfiguration('world_frame'),
                'base_frame':        LaunchConfiguration('base_frame'),
                'camera_frame':      LaunchConfiguration('camera_frame'),
                'use_viewer':        LaunchConfiguration('use_viewer'),
                'publish_tf':        LaunchConfiguration('publish_tf'),
                'sync_queue_size':   LaunchConfiguration('sync_queue_size'),
                'max_rgb_depth_dt':  LaunchConfiguration('max_rgb_depth_dt'),
            }],
        ),
    ])
