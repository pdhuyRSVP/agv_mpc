#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Astra Pro UVC RGB on this robot rejects 640x480@10 MJPEG and the
        # driver exits with code -11. Run the camera in its supported 30 Hz
        # mode; ORB nodes down-sample with target_tracking_hz.
        DeclareLaunchArgument('color_fps', default_value='30'),
        DeclareLaunchArgument('depth_fps', default_value='30'),
        DeclareLaunchArgument(
            'enable_point_cloud',
            default_value='true',
            description='Generate registered PointCloud2 for camera obstacle avoidance.'),
        DeclareLaunchArgument(
            'camera_publish_tf',
            default_value='true',
            description='Publish Astra camera frame transforms without overriding an ORB publish_tf argument.'),
        DeclareLaunchArgument(
            'camera_respawn',
            default_value='true',
            description='Restart the Astra driver if the vendor UVC process exits unexpectedly.'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('astra_camera'),
                    'launch',
                    'astra_pro.launch.xml',
                ])),
            launch_arguments={
                'camera_name': 'camera',
                'depth_registration': 'true',
                'color_depth_synchronization': 'true',
                'enable_point_cloud': LaunchConfiguration('enable_point_cloud'),
                'enable_colored_point_cloud': 'false',
                'enable_ir': 'false',
                'color_width': '640',
                'color_height': '480',
                'color_fps': LaunchConfiguration('color_fps'),
                'depth_width': '640',
                'depth_height': '480',
                'depth_fps': LaunchConfiguration('depth_fps'),
                'publish_tf': LaunchConfiguration('camera_publish_tf'),
                'respawn': LaunchConfiguration('camera_respawn'),
            }.items(),
        ),
    ])
