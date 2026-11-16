from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def generate_launch_description():

    # ===== Tham số IMU =====
    declare_imu_port = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        description='Serial port of IMU'
    )

    declare_imu_baud = DeclareLaunchArgument(
        'imu_baud',
        default_value='115200',
        description='Baudrate of IMU'
    )

    declare_imu_frame = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_link',
        description='Frame ID of IMU'
    )

    declare_imu_topic = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data_raw',
        description='Backward-compatible raw IMU topic alias'
    )

    declare_imu_orientation_topic = DeclareLaunchArgument(
        'imu_orientation_topic',
        default_value='/imu',
        description='Orientation IMU topic for EKF'
    )

    declare_imu_raw_topic = DeclareLaunchArgument(
        'imu_raw_topic',
        default_value='/imu/data_raw',
        description='Raw gyro+accel IMU topic for ORB-SLAM3'
    )

    declare_imu_orientation_rate = DeclareLaunchArgument(
        'imu_orientation_rate',
        default_value='50.0',
        description='EKF IMU orientation topic rate'
    )

    declare_imu_raw_rate = DeclareLaunchArgument(
        'imu_raw_rate',
        default_value='100.0',
        description='ORB raw IMU topic rate'
    )

    declare_imu_poll_hz = DeclareLaunchArgument(
        'imu_poll_hz',
        default_value='100.0',
        description='IMU polling rate in Hz'
    )

    # ===== Tham số Lidar =====
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_f005860690886840a130a940aaad5155-if00-port0',
        description='Serial port of RPLidar'
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_sensors'),
                'launch',
                'lidar.launch.py'
            )
        ),
        launch_arguments={
            'lidar_port': LaunchConfiguration('lidar_port'),
        }.items()
    )

    scan_filter_params = os.path.join(
        get_package_share_directory('agv_sensors'),
        'config',
        'scan_filter.yaml'
    )

    scan_filter_node = Node(
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

    # ===== Include imu.launch.py =====
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_sensors'),
                'launch',
                'imu.launch.py'
            )
        ),
        launch_arguments={
            'imu_port': LaunchConfiguration('imu_port'),
            'imu_baud': LaunchConfiguration('imu_baud'),
            'imu_frame_id': LaunchConfiguration('imu_frame_id'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'imu_orientation_topic': LaunchConfiguration('imu_orientation_topic'),
            'imu_raw_topic': LaunchConfiguration('imu_raw_topic'),
            'imu_orientation_rate': LaunchConfiguration('imu_orientation_rate'),
            'imu_raw_rate': LaunchConfiguration('imu_raw_rate'),
            'imu_poll_hz': LaunchConfiguration('imu_poll_hz'),
            'imu_publish_orientation': 'true',
            'imu_read_timeout_ms': '10',
            'imu_write_timeout_ms': '5',
        }.items()
    )

    # ===== Include wheel.launch.py =====
    declare_wheel_port = DeclareLaunchArgument(
        'wheel_port',
        default_value='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0',
        description='USB-RS485 serial port connected directly to ELD2-RS drivers'
    )

    declare_wheel_baud = DeclareLaunchArgument(
        'wheel_baudrate',
        default_value='115200',
        description='Modbus RTU baudrate for ELD2-RS drivers'
    )

    declare_wheel_gear = DeclareLaunchArgument(
        'wheel_gear_ratio',
        default_value='10.0',
        description='Gear ratio motor:wheel'
    )

    declare_wheel_radius = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.1',
        description='Wheel radius (m)'
    )

    declare_wheel_sep = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.636',
        description='Wheel separation (m)'
    )

    wheel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_sensors'),
                'launch',
                'wheel.launch.py'
            )
        ),
        launch_arguments={
            'port': LaunchConfiguration('wheel_port'),
            'baudrate': LaunchConfiguration('wheel_baudrate'),
            'gear_ratio': LaunchConfiguration('wheel_gear_ratio'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'base_frame': 'Base_footprint',
            'modbus_timeout_ms': '30.0',
            'loop_hz': '20.0',
            'invert_right_fb': 'true',
            'invert_right_cmd': 'false',
            'cmd_rpm_clamp': '3000',
        }.items()
    )

    sensor_health_node = Node(
        package='agv_sensors_cpp',
        executable='sensor_health_node',
        name='sensor_health_node',
        output='screen',
    )

    return LaunchDescription([
        declare_imu_port,
        declare_imu_baud,
        declare_imu_frame,
        declare_imu_topic,
        declare_imu_orientation_topic,
        declare_imu_raw_topic,
        declare_imu_orientation_rate,
        declare_imu_raw_rate,
        declare_imu_poll_hz,
        declare_lidar_port,
        declare_wheel_port,
        declare_wheel_baud,
        declare_wheel_gear,
        declare_wheel_radius,
        declare_wheel_sep,
        imu_launch,
        wheel_launch,
        lidar_launch,
        scan_filter_node,
        sensor_health_node,
    ])
