#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orb_share = get_package_share_directory('orb3_agv_sim')
    slam_share = get_package_share_directory('agv_slam')
    default_map = '/home/hcmute/agv_mpc/maps/lidar_2d/current_20260616_185758.yaml'
    default_refine_params = os.path.join(
        slam_share, 'config', 'slam_toolbox_refine.yaml')

    sensors_and_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orb_share, 'launch', 'real_amcl_stack.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'start_wheel': LaunchConfiguration('start_wheel'),
            'start_imu': LaunchConfiguration('start_imu'),
            'start_lidar': LaunchConfiguration('start_lidar'),
            'start_ekf': LaunchConfiguration('start_ekf'),
            'start_amcl': 'false',
            'use_initial_pose': 'false',
            'use_rviz': 'false',
            'amcl_tf_broadcast': 'false',
        }.items(),
    )

    slam_refine = Node(
        package='slam_toolbox',
        executable='map_and_localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'use_sim_time': False,
                'mode': 'mapping',
                'map_file_name': LaunchConfiguration('map_file_name'),
                'map_start_at_dock': LaunchConfiguration('map_start_at_dock'),
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'Base_footprint',
                'scan_topic': LaunchConfiguration('scan_topic'),
                'transform_publish_period': LaunchConfiguration(
                    'transform_publish_period'),
                'do_loop_closing': LaunchConfiguration('do_loop_closing'),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'map_file_name',
            default_value='/home/hcmute/agv_mpc/maps/lidar_2d/graphs/current_20260616_185758',
            description='Serialized posegraph basename to continue/refine.',
        ),
        DeclareLaunchArgument('slam_params_file', default_value=default_refine_params),
        DeclareLaunchArgument('map_start_at_dock', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('transform_publish_period', default_value='0.05'),
        DeclareLaunchArgument('do_loop_closing', default_value='true'),
        DeclareLaunchArgument('start_wheel', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_ekf', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        sensors_and_ekf,
        TimerAction(period=5.0, actions=[slam_refine]),
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('use_rviz')),
                ),
            ],
        ),
    ])
