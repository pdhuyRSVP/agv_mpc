from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    sensors_share = Path(get_package_share_directory('agv_sensors'))
    workspace_install = sensors_share.parents[2]
    rplidar_node_exe = workspace_install / 'rplidar_ros' / 'lib' / 'rplidar_ros' / 'rplidar_node'

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_f005860690886840a130a940aaad5155-if00-port0',
        description='Serial port of RPLidar'
    )

    lidar_node = ExecuteProcess(
        cmd=[
            str(rplidar_node_exe),
            '--ros-args',
            '-r', '__node:=rplidar_node',
            '-p', 'channel_type:=serial',
            '-p', ['serial_port:=', LaunchConfiguration('lidar_port')],
            '-p', 'serial_baudrate:=256000',
            '-p', 'frame_id:=Lidar',
            '-p', 'inverted:=false',
            '-p', 'angle_compensate:=true',
            '-p', 'scan_mode:=Sensitivity',
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        declare_lidar_port,
        lidar_node,
    ])
