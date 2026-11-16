#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('agv_description')
    control_share = get_package_share_directory('agv_control')
    orb_share = get_package_share_directory('orb3_agv_sim')

    urdf_file = os.path.join(description_share, 'urdf', 'agv0509test6.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    default_map = '/home/hcmute/agv_mpc/maps/lidar_2d/current_20260616_185758.yaml'
    scan_filter_params = os.path.join(control_share, 'config', 'scan_filter.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='front_scan_filter',
        output='screen',
        arguments=['--ros-args', '--log-level', 'error'],
        parameters=[scan_filter_params, {'use_sim_time': False}],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered'),
        ],
    )

    scan_filter_after_rsp = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[
                TimerAction(period=1.0, actions=[scan_filter])
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('start_wheel', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_ekf', default_value='true'),
        DeclareLaunchArgument('start_amcl', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('amcl_tf_broadcast', default_value='true'),
        DeclareLaunchArgument(
            'wheel_port',
            default_value='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'),
        DeclareLaunchArgument('wheel_baudrate', default_value='115200'),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument('imu_baud', default_value='115200'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_f005860690886840a130a940aaad5155-if00-port0'),
        DeclareLaunchArgument('use_initial_pose', default_value='true'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('initial_cov_xy', default_value='0.25'),
        DeclareLaunchArgument('initial_cov_yaw', default_value='0.0685'),

        robot_state_publisher,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('agv_sensors'), 'launch', 'wheel.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_wheel')),
            launch_arguments={
                'port': LaunchConfiguration('wheel_port'),
                'baudrate': LaunchConfiguration('wheel_baudrate'),
                'base_frame': 'Base_footprint',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('agv_sensors'), 'launch', 'imu.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_imu')),
            launch_arguments={
                'imu_port': LaunchConfiguration('imu_port'),
                'imu_baud': LaunchConfiguration('imu_baud'),
                'imu_frame_id': 'imu_link',
                'imu_orientation_topic': '/imu',
                'imu_raw_topic': '/imu/data_raw',
                'imu_orientation_rate': '50.0',
                'imu_raw_rate': '100.0',
                'imu_poll_hz': '100.0',
                'imu_publish_orientation': 'true',
                'imu_read_timeout_ms': '10',
                'imu_write_timeout_ms': '5',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('agv_sensors'), 'launch', 'lidar.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_lidar')),
            launch_arguments={
                'lidar_port': LaunchConfiguration('lidar_port'),
            }.items(),
        ),

        TimerAction(period=2.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('agv_control'), 'launch', 'ekf.launch.py')),
                condition=IfCondition(LaunchConfiguration('start_ekf')),
            ),
        ]),

        scan_filter_after_rsp,

        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('agv_navigation'), 'launch', 'navigation.launch.py')),
                condition=IfCondition(LaunchConfiguration('start_amcl')),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'amcl_tf_broadcast': LaunchConfiguration('amcl_tf_broadcast'),
                    'publish_robot_model': 'false',
                    'use_rviz': LaunchConfiguration('use_rviz'),
                }.items(),
            ),
        ]),

        TimerAction(period=8.0, actions=[
            Node(
                package='orb3_agv_sim',
                executable='initial_pose_publisher.py',
                name='initial_pose_publisher',
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_initial_pose')),
                parameters=[{
                    'frame_id': 'map',
                    'x': LaunchConfiguration('initial_x'),
                    'y': LaunchConfiguration('initial_y'),
                    'yaw': LaunchConfiguration('initial_yaw'),
                    'cov_xy': LaunchConfiguration('initial_cov_xy'),
                    'cov_yaw': LaunchConfiguration('initial_cov_yaw'),
                    'publish_count': 20,
                    'period_sec': 0.5,
                }],
            ),
        ]),
    ])
