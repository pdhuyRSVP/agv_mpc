#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================
    # PACKAGE PATHS
    # ============================
    pkg_sensors = get_package_share_directory('agv_sensors')
    pkg_slam    = get_package_share_directory('agv_slam')
    pkg_robot   = get_package_share_directory('agv_description')

    # ============================
    # LIDAR
    # ============================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensors, 'launch', 'lidar.launch.py')
        ),
        launch_arguments={
            'lidar_port': LaunchConfiguration('lidar_port')
        }.items()
    )

    # ============================
    # IMU
    # ============================
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensors, 'launch', 'imu.launch.py')
        ),
        launch_arguments={
            'imu_port': LaunchConfiguration('imu_port'),
            'imu_baud': LaunchConfiguration('imu_baud'),
            'imu_frame_id': LaunchConfiguration('imu_frame_id')
        }.items()
    )

    # ============================
    # WHEEL DRIVER: /cmd_vel -> USB-RS485 Modbus -> ELD2-RS, publish /wheel_vel
    # ============================
    wheel_vel_node = Node(
        package='agv_sensors_cpp',
        executable='wheel_vel_node_direct',
        name='wheel_vel_node_direct',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('wheel_port'),
            'baud': LaunchConfiguration('wheel_baudrate'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'gear_ratio': LaunchConfiguration('wheel_gear_ratio'),
            'base_frame': 'Base_footprint',
            'modbus_timeout_ms': 30.0,
            'loop_hz': 20.0,
            'invert_right_fb': True,
            'invert_right_cmd': False,
            'cmd_rpm_clamp': 3000,
            'init_path0': True,
        }]
    )

    # ============================
    # ROBOT STATE + EKF + DISPLAY
    # ============================
    urdf_file = os.path.join(pkg_robot, 'urdf', 'agv0509test6.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )

    # ============================
    # SLAM TOOLBOX (ĐÃ CHỈNH SỬA)
    # ============================
    slam_params = os.path.join(
        pkg_slam,
        'config',
        'slam_toolbox_params.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {
                'odom_frame': 'odom',
                'base_frame': 'Base_footprint',   # chuẩn TF ROS2
                'map_frame': 'map',
                'scan_topic': '/scan_filtered',
                'use_sim_time': False
            }
        ]
    )

    # ============================
    # DECLARE LAUNCH ARGUMENTS
    # ============================
    declare_lidar = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_f005860690886840a130a940aaad5155-if00-port0'
    )
    declare_imu_port = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
    )
    declare_imu_baud = DeclareLaunchArgument(
        'imu_baud', default_value='9600'
    )
    declare_imu_frame = DeclareLaunchArgument(
        'imu_frame_id', default_value='imu_link'
    )
    declare_wheel_port = DeclareLaunchArgument(
        'wheel_port',
        default_value='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
    )
    declare_wheel_baud = DeclareLaunchArgument(
        'wheel_baudrate', default_value='115200'
    )
    declare_wheel_radius = DeclareLaunchArgument(
        'wheel_radius', default_value='0.1'
    )
    declare_wheel_sep = DeclareLaunchArgument(
        'wheel_separation', default_value='0.636'
    )
    declare_wheel_gear = DeclareLaunchArgument(
        'wheel_gear_ratio', default_value='10.0'
    )

    return LaunchDescription([
        declare_lidar,
        declare_imu_port,
        declare_imu_baud,
        declare_imu_frame,
        declare_wheel_port,
        declare_wheel_baud,
        declare_wheel_radius,
        declare_wheel_sep,
        declare_wheel_gear,

        lidar_launch,
        imu_launch,
        wheel_vel_node,
        #cmd_vel_uart_node,
        robot_state_pub,

        # SLAM toolbox node (đưa vào launch)
        slam_node
    ])
