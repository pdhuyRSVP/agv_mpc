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
    navigation_share = get_package_share_directory('agv_navigation')

    sensor_ekf_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orb_share, 'launch', 'real_amcl_stack.launch.py')
        ),
        launch_arguments={
            'start_wheel': LaunchConfiguration('start_wheel'),
            'start_imu': LaunchConfiguration('start_imu'),
            'start_lidar': LaunchConfiguration('start_lidar'),
            'start_ekf': LaunchConfiguration('start_ekf'),
            'start_amcl': 'false',
            'use_initial_pose': 'false',
            'wheel_port': LaunchConfiguration('wheel_port'),
            'imu_port': LaunchConfiguration('imu_port'),
            'lidar_port': LaunchConfiguration('lidar_port'),
        }.items(),
    )

    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'map_file_name': LaunchConfiguration('map_file_name'),
            'use_sim_time': 'false',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'Base_footprint',
            'scan_topic': '/scan_filtered',
            'map_start_at_dock': 'true',
            'publish_robot_model': 'false',
            'start_ekf': 'false',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'start_localization': 'false',
            'publish_robot_model': 'false',
            'use_sim_time': 'false',
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    initial_pose = Node(
        package='orb3_agv_sim',
        executable='initial_pose_publisher.py',
        name='initial_pose_publisher',
        output='screen',
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
    )

    graph_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orb_share, 'launch', 'gtsam_pose2_fusion.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('start_graph_monitor')),
        launch_arguments={
            'ekf_odom_topic': '/odom',
            'orb_pose_topic': LaunchConfiguration('orb_pose_topic'),
            'orb_tracking_state_topic': LaunchConfiguration('orb_tracking_state_topic'),
            'scan_odom_topic': '/disabled/scan_odom',
            'publish_tf': 'false',
            'use_global_pose_factor': 'false',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'Base_footprint',
            'publish_robot_model': 'false',
            'start_ekf': 'false',
        }.items(),
    )

    # AMCL may run in parallel for logging/comparison, but slam_toolbox remains
    # the only map->odom broadcaster. AMCL consumes slam_toolbox's /map topic.
    amcl_monitor = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_amcl_monitor')),
        parameters=[
            LaunchConfiguration('amcl_params_file'),
            {
                'tf_broadcast': False,
                'use_sim_time': False,
            },
        ],
    )

    amcl_monitor_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_amcl_monitor')),
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['amcl'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file_name',
            default_value='/home/hcmute/agv_mpc/maps/lidar_2d/graphs/current_20260616_185758',
            description='Serialized slam_toolbox posegraph basename.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                slam_share, 'config', 'slam_toolbox_localization.yaml'
            ),
        ),
        DeclareLaunchArgument('start_wheel', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_ekf', default_value='true'),
        DeclareLaunchArgument(
            'start_graph_monitor',
            default_value='false',
            description='Run GTSAM for metrics only; it does not publish map->odom.',
        ),
        DeclareLaunchArgument(
            'start_amcl_monitor',
            default_value='false',
            description=(
                'Run AMCL for /amcl_pose data only. TF is always disabled and '
                'slam_toolbox remains the localization source for Nav2.'
            ),
        ),
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=os.path.join(
                navigation_share, 'config', 'nav2_mppi_lidar.yaml'
            ),
        ),
        DeclareLaunchArgument('orb_pose_topic', default_value='/orbslam3/pose_base'),
        DeclareLaunchArgument(
            'orb_tracking_state_topic',
            default_value='/orbslam3/tracking_state',
        ),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'wheel_port',
            default_value='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0',
        ),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_f005860690886840a130a940aaad5155-if00-port0',
        ),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('initial_cov_xy', default_value='0.25'),
        DeclareLaunchArgument('initial_cov_yaw', default_value='0.0685'),

        sensor_ekf_stack,
        TimerAction(period=5.0, actions=[slam_localization]),
        TimerAction(period=8.0, actions=[navigation]),
        TimerAction(period=11.0, actions=[initial_pose]),
        TimerAction(period=12.0, actions=[graph_monitor]),
        TimerAction(
            period=9.0,
            actions=[amcl_monitor, amcl_monitor_lifecycle],
        ),
    ])
