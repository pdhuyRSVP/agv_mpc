#!/usr/bin/env python3
"""Run ORB-SLAM3 RGB-D only against the real Astra Pro topics."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('orb3_agv_sim')
    description_pkg = get_package_share_directory('agv_description')
    home = os.path.expanduser('~')
    urdf_file = os.path.join(description_pkg, 'urdf', 'agv0509test6.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as stream:
        robot_description = stream.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'vocabulary_file',
            default_value=os.path.join(home, 'Dev', 'ORB_SLAM3', 'Vocabulary', 'ORBvoc.txt')),
        DeclareLaunchArgument(
            'settings_file',
            default_value=os.path.join(pkg, 'config', 'orb3_rgbd_astra_pro.yaml')),
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start Astra Pro RGB-D camera together with ORB. Set false if camera is already running.'),
        DeclareLaunchArgument(
            'camera_enable_point_cloud',
            default_value='true',
            description='Generate camera PointCloud2; ORB itself only needs RGB and registered depth images.'),
        DeclareLaunchArgument('load_atlas_file', default_value=''),
        DeclareLaunchArgument('single_map_mode', default_value='true'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        # The Astra driver publishes registered depth on this same topic when
        # astra_rgbd_10hz.launch.py sets depth_registration:=true.
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('world_frame', default_value='orb_map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='Base_footprint'),
        DeclareLaunchArgument('camera_frame', default_value='color_optical_frame'),
        DeclareLaunchArgument('use_viewer', default_value='false'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('tf_publish_mode', default_value='world_to_base'),
        DeclareLaunchArgument('force_planar_output', default_value='true'),
        DeclareLaunchArgument('pose_topic', default_value='/orbslam3/pose'),
        DeclareLaunchArgument('path_topic', default_value='/orbslam3/path'),
        DeclareLaunchArgument('camera_pose_topic', default_value='/orbslam3/camera_pose'),
        DeclareLaunchArgument(
            'raw_camera_pose_topic', default_value='/orbslam3/pose_raw_camera'),
        DeclareLaunchArgument('base_pose_topic', default_value='/orbslam3/pose_base'),
        DeclareLaunchArgument('pose_debug_topic', default_value='/orbslam3/pose_debug'),
        DeclareLaunchArgument(
            'tracking_state_topic', default_value='/orbslam3/tracking_state'),
        DeclareLaunchArgument(
            'tracking_status_topic', default_value='/orbslam3/tracking_status'),
        DeclareLaunchArgument('map_points_topic', default_value='/orbslam3/map_points'),
        DeclareLaunchArgument('tracked_points_topic', default_value='/orbslam3/tracked_points'),
        DeclareLaunchArgument('rviz_markers_topic', default_value='/orbslam3/rviz/markers'),
        DeclareLaunchArgument('rviz_keyframes_topic', default_value='/orbslam3/rviz/keyframes'),
        DeclareLaunchArgument('tracking_preview_topic', default_value='/agv/orb_tracking_preview'),
        DeclareLaunchArgument('save_outputs', default_value='true'),
        DeclareLaunchArgument(
            'save_atlas',
            default_value='true',
            description='Save ORB-SLAM3 binary atlas on shutdown. Keep false when only CSV/WebOrb map data is needed.'),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('unique_output_dir', default_value='true'),
        DeclareLaunchArgument('keypoint_log_hz', default_value='1.0'),
        DeclareLaunchArgument('save_keypoint_images', default_value='false'),
        DeclareLaunchArgument('max_saved_keypoints', default_value='1200'),
        DeclareLaunchArgument('max_rgb_depth_dt', default_value='0.20'),
        DeclareLaunchArgument('sync_queue_size', default_value='10'),
        DeclareLaunchArgument(
            'target_tracking_hz',
            default_value='10.0',
            description='Target ORB RGB-D tracking rate. Localization-only is forced when loading an atlas.'),
        DeclareLaunchArgument(
            'max_track_rgbd_sec',
            default_value='0.0',
            description='If >0, exit and let launch respawn when one TrackRGBD call is stuck longer than this.'),
        DeclareLaunchArgument(
            'image_callback_watchdog_sec',
            default_value='8.0',
            description='If >0, respawn ORB when RGB/depth callbacks stop while the camera topics are expected to be alive.'),
        DeclareLaunchArgument('map_points_publish_hz', default_value='0.0'),
        DeclareLaunchArgument('graph_publish_hz', default_value='0.0'),
        DeclareLaunchArgument('covisibility_min_weight', default_value='100'),
        DeclareLaunchArgument('keyframe_frustum_size', default_value='0.08'),
        DeclareLaunchArgument('auto_localization_map_points', default_value='0'),
        DeclareLaunchArgument('localization_nfeatures', default_value='2000'),
        DeclareLaunchArgument('localization_nlevels', default_value='10'),
        DeclareLaunchArgument('localization_scale_factor', default_value='1.1'),
        DeclareLaunchArgument('max_published_map_points', default_value='8000'),
        DeclareLaunchArgument('max_published_tracked_points', default_value='1500'),
        DeclareLaunchArgument('publish_tracking_preview', default_value='true'),
        DeclareLaunchArgument('tracking_preview_width', default_value='320'),
        DeclareLaunchArgument('tracking_preview_hz', default_value='0.5'),
        DeclareLaunchArgument('max_path_length', default_value='500'),
        DeclareLaunchArgument('publish_robot_model', default_value='false'),
        DeclareLaunchArgument(
            'publish_camera_base_tf',
            default_value='true',
            description='Publish the calibrated Base_footprint -> camera optical static TF.'),

        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg, 'launch', 'astra_rgbd_10hz.launch.py')),
                    condition=IfCondition(LaunchConfiguration('start_camera')),
                    launch_arguments={
                        'enable_point_cloud': LaunchConfiguration(
                            'camera_enable_point_cloud'),
                    }.items(),
                ),
            ],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='orb_robot_state_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_robot_model')),
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='orb_map_anchor_tf',
            arguments=[
                '0.0', '0.0', '0.0',
                '0.0', '0.0', '0.0',
                LaunchConfiguration('world_frame'),
                'orb_map_anchor',
            ],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_astra_color_optical_tf',
            arguments=[
                '0.3875', '-0.0030', '0.1425',
                '-1.57079632679', '0.0', '-1.57079632679',
                LaunchConfiguration('base_frame'),
                LaunchConfiguration('camera_frame'),
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_camera_base_tf')),
        ),

        Node(
            package='orb3_agv_sim',
            executable='orb3_rgbd_node',
            name='orb3_rgbd_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'vocabulary_file': LaunchConfiguration('vocabulary_file'),
                'settings_file': LaunchConfiguration('settings_file'),
                'load_atlas_file': LaunchConfiguration('load_atlas_file'),
                'single_map_mode': LaunchConfiguration('single_map_mode'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'world_frame': LaunchConfiguration('world_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'use_viewer': LaunchConfiguration('use_viewer'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'tf_publish_mode': LaunchConfiguration('tf_publish_mode'),
                'force_planar_output': LaunchConfiguration('force_planar_output'),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'path_topic': LaunchConfiguration('path_topic'),
                'camera_pose_topic': LaunchConfiguration('camera_pose_topic'),
                'raw_camera_pose_topic': LaunchConfiguration('raw_camera_pose_topic'),
                'base_pose_topic': LaunchConfiguration('base_pose_topic'),
                'pose_debug_topic': LaunchConfiguration('pose_debug_topic'),
                'tracking_state_topic': LaunchConfiguration('tracking_state_topic'),
                'tracking_status_topic': LaunchConfiguration('tracking_status_topic'),
                'publish_raw_camera_pose': True,
                'publish_base_pose': True,
                'publish_pose_debug': True,
                'map_points_topic': LaunchConfiguration('map_points_topic'),
                'tracked_points_topic': LaunchConfiguration('tracked_points_topic'),
                'rviz_markers_topic': LaunchConfiguration('rviz_markers_topic'),
                'rviz_keyframes_topic': LaunchConfiguration('rviz_keyframes_topic'),
                'tracking_preview_topic': LaunchConfiguration('tracking_preview_topic'),
                'save_outputs': LaunchConfiguration('save_outputs'),
                'save_atlas': LaunchConfiguration('save_atlas'),
                'output_dir': LaunchConfiguration('output_dir'),
                'unique_output_dir': LaunchConfiguration('unique_output_dir'),
                'keypoint_log_hz': LaunchConfiguration('keypoint_log_hz'),
                'save_keypoint_images': LaunchConfiguration('save_keypoint_images'),
                'max_saved_keypoints': LaunchConfiguration('max_saved_keypoints'),
                'max_rgb_depth_dt': LaunchConfiguration('max_rgb_depth_dt'),
                'sync_queue_size': LaunchConfiguration('sync_queue_size'),
                'target_tracking_hz': LaunchConfiguration('target_tracking_hz'),
                'max_track_rgbd_sec': LaunchConfiguration('max_track_rgbd_sec'),
                'image_callback_watchdog_sec': LaunchConfiguration('image_callback_watchdog_sec'),
                'map_points_publish_hz': LaunchConfiguration('map_points_publish_hz'),
                'graph_publish_hz': LaunchConfiguration('graph_publish_hz'),
                'covisibility_min_weight': LaunchConfiguration('covisibility_min_weight'),
                'keyframe_frustum_size': LaunchConfiguration('keyframe_frustum_size'),
                'auto_localization_map_points': LaunchConfiguration('auto_localization_map_points'),
                'localization_nfeatures': LaunchConfiguration('localization_nfeatures'),
                'localization_nlevels': LaunchConfiguration('localization_nlevels'),
                'localization_scale_factor': LaunchConfiguration('localization_scale_factor'),
                'max_published_map_points': LaunchConfiguration('max_published_map_points'),
                'max_published_tracked_points': LaunchConfiguration('max_published_tracked_points'),
                'publish_tracking_preview': LaunchConfiguration('publish_tracking_preview'),
                'tracking_preview_width': LaunchConfiguration('tracking_preview_width'),
                'tracking_preview_hz': LaunchConfiguration('tracking_preview_hz'),
                'max_path_length': LaunchConfiguration('max_path_length'),
            }],
        ),
    ])
