#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_port = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        description='Serial/RS485 port of the IMU'
    )

    declare_baud = DeclareLaunchArgument(
        'imu_baud',
        default_value='115200',
        description='IMU serial baudrate'
    )

    declare_frame = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_link',
        description='Frame ID used in raw IMU header'
    )

    declare_topic = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data_raw',
        description='Backward-compatible raw IMU topic alias'
    )

    declare_orientation_topic = DeclareLaunchArgument(
        'imu_orientation_topic',
        default_value='/imu',
        description='Orientation IMU topic for EKF, decimated from the same serial reader'
    )

    declare_raw_topic = DeclareLaunchArgument(
        'imu_raw_topic',
        default_value='/imu/data_raw',
        description='Raw gyro+accel IMU topic for ORB-SLAM3'
    )

    declare_orientation_rate = DeclareLaunchArgument(
        'imu_orientation_rate',
        default_value='50.0',
        description='Publish rate for EKF orientation IMU topic'
    )

    declare_raw_rate = DeclareLaunchArgument(
        'imu_raw_rate',
        default_value='100.0',
        description='Publish/poll rate for raw ORB-SLAM3 IMU topic'
    )

    declare_slave = DeclareLaunchArgument(
        'imu_slave_id',
        default_value='81',
        description='HWT901B Modbus slave ID. 81 decimal is 0x51.'
    )

    declare_poll_hz = DeclareLaunchArgument(
        'imu_poll_hz',
        default_value='100.0',
        description='IMU polling rate in Hz'
    )

    declare_time_offset = DeclareLaunchArgument(
        'imu_time_offset_sec',
        default_value='0.0',
        description='Timestamp correction added to midpoint request-response time'
    )

    declare_publish_orientation = DeclareLaunchArgument(
        'imu_publish_orientation',
        default_value='true',
        description='Publish the HWT901B AHRS quaternion on the orientation-only /imu topic.'
    )

    declare_read_timeout = DeclareLaunchArgument(
        'imu_read_timeout_ms',
        default_value='10',
        description='Serial read timeout for 100 Hz Modbus polling'
    )

    declare_write_timeout = DeclareLaunchArgument(
        'imu_write_timeout_ms',
        default_value='5',
        description='Serial write timeout for 100 Hz Modbus polling'
    )

    imu_node = Node(
        package='agv_sensors_cpp',
        executable='hwt901b_modbus_node',
        name='hwt901b_modbus',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baud': LaunchConfiguration('imu_baud'),
            'frame_id': LaunchConfiguration('imu_frame_id'),
            'slave_id': LaunchConfiguration('imu_slave_id'),
            'poll_hz': LaunchConfiguration('imu_poll_hz'),
            'orientation_topic': LaunchConfiguration('imu_orientation_topic'),
            'raw_topic': LaunchConfiguration('imu_raw_topic'),
            'orientation_rate': LaunchConfiguration('imu_orientation_rate'),
            'raw_rate': LaunchConfiguration('imu_raw_rate'),
            'time_offset_sec': LaunchConfiguration('imu_time_offset_sec'),
            'publish_orientation': LaunchConfiguration('imu_publish_orientation'),
            'read_timeout_ms': LaunchConfiguration('imu_read_timeout_ms'),
            'write_timeout_ms': LaunchConfiguration('imu_write_timeout_ms'),
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        declare_port,
        declare_baud,
        declare_frame,
        declare_topic,
        declare_orientation_topic,
        declare_raw_topic,
        declare_orientation_rate,
        declare_raw_rate,
        declare_slave,
        declare_poll_hz,
        declare_time_offset,
        declare_publish_orientation,
        declare_read_timeout,
        declare_write_timeout,
        imu_node,
    ])
