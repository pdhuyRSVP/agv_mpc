#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('base_map_topic', default_value='/map'),
        DeclareLaunchArgument('orb_points_topic', default_value='/orb_rgbd/map_points'),
        DeclareLaunchArgument('orb_grid_topic', default_value='/orb_2d_map'),
        DeclareLaunchArgument('combined_map_topic', default_value='/combined_map'),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument('min_z', default_value='-0.20'),
        DeclareLaunchArgument('max_z', default_value='2.00'),
        DeclareLaunchArgument('occupied_value', default_value='70'),
        DeclareLaunchArgument('unknown_orb_value', default_value='45'),
        DeclareLaunchArgument('inflation_radius', default_value='0.05'),
        DeclareLaunchArgument('publish_hz', default_value='1.0'),
        DeclareLaunchArgument('max_points', default_value='12000'),

        Node(
            package='orb3_agv_sim',
            executable='orb_map_projector.py',
            name='orb_map_projector',
            output='screen',
            parameters=[{
                'base_map_topic': LaunchConfiguration('base_map_topic'),
                'orb_points_topic': LaunchConfiguration('orb_points_topic'),
                'orb_grid_topic': LaunchConfiguration('orb_grid_topic'),
                'combined_map_topic': LaunchConfiguration('combined_map_topic'),
                'target_frame': LaunchConfiguration('target_frame'),
                'min_z': LaunchConfiguration('min_z'),
                'max_z': LaunchConfiguration('max_z'),
                'occupied_value': LaunchConfiguration('occupied_value'),
                'unknown_orb_value': LaunchConfiguration('unknown_orb_value'),
                'inflation_radius': LaunchConfiguration('inflation_radius'),
                'publish_hz': LaunchConfiguration('publish_hz'),
                'max_points': LaunchConfiguration('max_points'),
            }],
        ),
    ])
