from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # File cấu hình joystick
    joy_params = os.path.join(
        get_package_share_directory('agv_slam'),
        'config',
        'joystick.yaml'
    )

    # Node đọc tay cầm
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            joy_params,
            {"dev": "/dev/input/js0"},
        ],
        output='screen'
    )

    # Node chuyển joystick -> /cmd_vel
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ],
        output='screen'
    )

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

    wheel_node = Node(
            package='agv_sensors_cpp',
            executable='wheel_vel_node_direct',
            name='wheel_vel_node_direct',
            output='screen',
            parameters=[
                {
                    'port': LaunchConfiguration('wheel_port'),
                    'baud': LaunchConfiguration('wheel_baudrate'),
                    'wheel_radius': 0.10,
                    'wheel_separation': 0.636,
                    'gear_ratio': 10.0,
                    'base_frame': 'Base_footprint',
                    'modbus_timeout_ms': 30.0,
                    'loop_hz': 20.0,
                    'invert_right_fb': True,
                    'invert_right_cmd': False,
                    'cmd_rpm_clamp': 3000,
                    'init_path0': True,
                }
            ]
    )


    return LaunchDescription([
        declare_wheel_port,
        declare_wheel_baud,
        joy_node,
        teleop_node,
        wheel_node,
    ])
