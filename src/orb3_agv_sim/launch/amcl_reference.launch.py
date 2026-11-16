#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('agv_navigation')
    default_params = os.path.join(nav_share, 'config', 'nav2_mppi_lidar.yaml')

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': False,
                'tf_broadcast': False,
                'scan_topic': LaunchConfiguration('scan_topic'),
                'global_frame_id': 'map',
                'odom_frame_id': 'odom',
                'base_frame_id': 'Base_footprint',
                'set_initial_pose': True,
                'initial_pose.x': LaunchConfiguration('initial_x'),
                'initial_pose.y': LaunchConfiguration('initial_y'),
                'initial_pose.yaw': LaunchConfiguration('initial_yaw'),
            },
        ],
        remappings=[
            ('amcl_pose', '/amcl_reference/pose'),
            ('particle_cloud', '/amcl_reference/particle_cloud'),
        ],
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl_reference',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['amcl'],
            'use_sim_time': False,
        }],
    )
    initial_pose_seed = Node(
        package='orb3_agv_sim',
        executable='initial_pose_publisher.py',
        name='amcl_reference_initial_pose',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'x': LaunchConfiguration('initial_x'),
            'y': LaunchConfiguration('initial_y'),
            'yaw': LaunchConfiguration('initial_yaw'),
            'cov_xy': 0.25,
            'cov_yaw': 0.0685,
            'publish_count': 12,
            'period_sec': 0.5,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        amcl,
        lifecycle,
        TimerAction(period=4.0, actions=[initial_pose_seed]),
    ])
