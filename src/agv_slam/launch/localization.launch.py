#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('agv_description')
    urdf_file = os.path.join(description_share, 'urdf', 'agv0509test6.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    default_params = os.path.join(
        get_package_share_directory('agv_slam'),
        'config',
        'slam_toolbox_localization.yaml',
    )
    default_rviz_config = os.path.join(
        get_package_share_directory('agv_navigation'),
        'config',
        'nav2_lidar_map.rviz',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_robot_model')),
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_control'),
                'launch',
                'ekf.launch.py',
            )
        ),
        condition=IfCondition(LaunchConfiguration('start_ekf')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_params,
            description='slam_toolbox localization parameter file.',
        ),
        DeclareLaunchArgument(
            'map_file_name',
            default_value='/home/hcmute/agv_mpc/maps/lidar_2d/graphs/current_20260703_184946',
            description='Serialized posegraph basename, without .posegraph/.data.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='Base_footprint'),
        DeclareLaunchArgument('scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('map_start_at_dock', default_value='true'),
        DeclareLaunchArgument('publish_robot_model', default_value='true'),
        DeclareLaunchArgument('start_ekf', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),

        robot_state_publisher,
        ekf_launch,

        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'map_file_name': LaunchConfiguration('map_file_name'),
                    'map_frame': LaunchConfiguration('map_frame'),
                    'odom_frame': LaunchConfiguration('odom_frame'),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'mode': 'localization',
                    'map_start_at_dock': LaunchConfiguration('map_start_at_dock'),
                    'do_loop_closing': False,
                },
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
