import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sensors_share = get_package_share_directory("agv_sensors")
    description_share = get_package_share_directory("agv_description")
    lidar_port = LaunchConfiguration("lidar_port")
    urdf_path = os.path.join(description_share, "urdf", "agv0509test6.urdf")
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="lidar_robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False,
            }
        ],
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_share, "launch", "lidar.launch.py")
        ),
        launch_arguments={"lidar_port": lidar_port}.items(),
    )

    scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="front_scan_filter",
        output="screen",
        arguments=["--ros-args", "--log-level", "error"],
        parameters=[
            os.path.join(sensors_share, "config", "scan_filter.yaml"),
            {"use_sim_time": False},
        ],
        remappings=[
            ("scan", "/scan"),
            ("scan_filtered", "/scan_filtered"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_f005860690886840a130a940aaad5155-if00-port0",
        ),
        robot_state_publisher,
        lidar,
        scan_filter,
    ])
