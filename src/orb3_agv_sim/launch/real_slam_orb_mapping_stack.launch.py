#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    orb_share = get_package_share_directory('orb3_agv_sim')
    slam_share = get_package_share_directory('agv_slam')
    session_dir = LaunchConfiguration('session_dir')

    sensors_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orb_share, 'launch', 'real_amcl_stack.launch.py')
        ),
        launch_arguments={
            'start_wheel': 'true',
            'start_imu': 'true',
            'start_lidar': 'true',
            'start_ekf': LaunchConfiguration('start_ekf'),
            'start_amcl': 'false',
            'use_initial_pose': 'false',
            'use_rviz': 'false',
        }.items(),
    )

    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_rviz': LaunchConfiguration('use_rviz'),
            'publish_robot_model': 'false',
            'start_ekf': 'false',
        }.items(),
    )

    orb_rgbd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orb_share, 'launch', 'orb3_rgbd_real_astra_pro.launch.py')
        ),
        launch_arguments={
            'start_camera': 'true',
            'use_viewer': LaunchConfiguration('use_orb_viewer'),
            'publish_tf': 'false',
            'publish_robot_model': 'false',
            'save_outputs': 'true',
            'output_dir': [session_dir, '/orb'],
            'unique_output_dir': 'false',
            'save_keypoint_images': LaunchConfiguration('save_keypoint_images'),
        }.items(),
    )

    graph_factor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orb_share, 'launch', 'gtsam_pose2_fusion.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('start_graph_factor')),
        launch_arguments={
            'enable_diagnostics': 'true',
            'diagnostics_output_dir': [session_dir, '/graph_diagnostics'],
            'ekf_odom_topic': '/odom',
            'publish_robot_model': 'false',
            'start_ekf': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'session_dir',
            default_value='/home/hcmute/agv_mpc/maps/sessions/maze_31',
            description='Common output directory for this LiDAR and ORB mapping run.',
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_orb_viewer', default_value='true'),
        DeclareLaunchArgument('start_ekf', default_value='true'),
        DeclareLaunchArgument('start_graph_factor', default_value='true'),
        DeclareLaunchArgument('save_keypoint_images', default_value='false'),

        LogInfo(msg=['Combined mapping session: ', session_dir]),
        sensors_ekf,
        TimerAction(period=4.0, actions=[slam_mapping]),
        TimerAction(period=7.0, actions=[orb_rgbd]),
        TimerAction(period=10.0, actions=[graph_factor]),
    ])
