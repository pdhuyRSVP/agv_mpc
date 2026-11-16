from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0',
        description='USB-RS485 serial port connected directly to ELD2-RS drivers'
    )

    declare_baud = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Modbus RTU baudrate for ELD2-RS drivers'
    )

    declare_gear = DeclareLaunchArgument(
        'gear_ratio',
        default_value='10.0',
        description='Gear ratio motor : wheel'
    )

    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='Base_footprint',
        description='Frame ID used in /wheel_vel header'
    )

    declare_modbus_timeout = DeclareLaunchArgument(
        'modbus_timeout_ms',
        default_value='30.0',
        description='Timeout for each Modbus transaction in milliseconds'
    )

    declare_loop_hz = DeclareLaunchArgument(
        'loop_hz',
        default_value='20.0',
        description='Direct RS485 write/read loop frequency'
    )

    declare_command_timeout = DeclareLaunchArgument(
        'command_timeout_s',
        default_value='0.5',
        description='Seconds to keep the last cmd_vel command before stopping'
    )

    declare_invert_right_fb = DeclareLaunchArgument(
        'invert_right_fb',
        default_value='true',
        description='Invert right wheel feedback RPM'
    )

    declare_invert_right_cmd = DeclareLaunchArgument(
        'invert_right_cmd',
        default_value='false',
        description='Invert right wheel command RPM'
    )

    declare_cmd_rpm_clamp = DeclareLaunchArgument(
        'cmd_rpm_clamp',
        default_value='3000',
        description='Maximum absolute motor RPM command'
    )

    declare_init_path0 = DeclareLaunchArgument(
        'init_path0',
        default_value='true',
        description='Initialize ELD2-RS PATH0 registers on startup'
    )

    declare_left_slave = DeclareLaunchArgument(
        'left_slave_id',
        default_value='1',
        description='Modbus slave ID of the left wheel driver'
    )

    declare_right_slave = DeclareLaunchArgument(
        'right_slave_id',
        default_value='2',
        description='Modbus slave ID of the right wheel driver'
    )

    declare_radius = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.1',
        description='Wheel radius (m)'
    )

    declare_sep = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.636',
        description='Distance between left and right wheel centers (m)'
    )

    wheel_node = Node(
        package='agv_sensors_cpp',
        executable='wheel_vel_node_direct',
        name='wheel_vel_node_direct',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baudrate'),
            'gear_ratio': LaunchConfiguration('gear_ratio'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'base_frame': LaunchConfiguration('base_frame'),
            'modbus_timeout_ms': ParameterValue(
                LaunchConfiguration('modbus_timeout_ms'),
                value_type=float
            ),
            'command_timeout_s': LaunchConfiguration('command_timeout_s'),
            'loop_hz': LaunchConfiguration('loop_hz'),
            'invert_right_fb': LaunchConfiguration('invert_right_fb'),
            'invert_right_cmd': LaunchConfiguration('invert_right_cmd'),
            'cmd_rpm_clamp': LaunchConfiguration('cmd_rpm_clamp'),
            'init_path0': LaunchConfiguration('init_path0'),
            'left_slave_id': LaunchConfiguration('left_slave_id'),
            'right_slave_id': LaunchConfiguration('right_slave_id'),
        }]
    )

    return LaunchDescription([
        declare_port,
        declare_baud,
        declare_gear,
        declare_base_frame,
        declare_modbus_timeout,
        declare_loop_hz,
        declare_command_timeout,
        declare_invert_right_fb,
        declare_invert_right_cmd,
        declare_cmd_rpm_clamp,
        declare_init_path0,
        declare_left_slave,
        declare_right_slave,
        declare_radius,
        declare_sep,
        wheel_node
    ])
