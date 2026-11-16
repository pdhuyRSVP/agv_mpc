from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # ==== Các tham số chung ====
    declare_use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Run slam_toolbox (mapping mode)'
    )

    declare_use_nav = DeclareLaunchArgument(
        'use_nav',
        default_value='true',
        description='Run Navigation2 stack'
    )

    declare_start_sensors = DeclareLaunchArgument(
        'start_sensors',
        default_value='false',
        description='Start IMU, wheel and LiDAR from bringup. Set false when sensors.launch.py is already running.'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock from simulation'
    )

    declare_publish_robot_model = DeclareLaunchArgument(
        'publish_robot_model',
        default_value='true',
        description='Publish robot_description and fixed TF from agv_description'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz with the 2D map fixed frame'
    )

    # ==== Load URDF từ agv_description ====
    description_share = get_package_share_directory('agv_description')
    urdf_file = os.path.join(description_share, 'urdf', 'agv0509test6.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ==== Include sensors (IMU + wheel + lidar) từ agv_sensors ====
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_sensors'),
                'launch',
                'sensors.launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration('start_sensors')),
        # Các tham số port/baud… đang dùng default; nếu muốn truyền từ đây
        # thì thêm launch_arguments={...}.items()
    )

    # ==== Include EKF từ agv_control ====
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_control'),
                'launch',
                'ekf.launch.py'
            )
        )
    )

    # ==== SLAM (slam_toolbox) – chỉ chạy khi use_slam:=true ====
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_slam'),
                'launch',
                'slam.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'publish_robot_model': 'false',
            'start_ekf': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam'))
    )

    # ==== Navigation2 – chỉ chạy khi use_nav:=true ====
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_navigation'),
                'launch',
                'navigation.launch.py'
            )
        ),
        launch_arguments={
            # bringup already owns robot_state_publisher; keep a single source
            # for /robot_description and /tf_static.
            'publish_robot_model': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav'))
    )

    return LaunchDescription([
        declare_use_slam,
        declare_use_nav,
        declare_start_sensors,
        declare_use_sim_time,
        declare_publish_robot_model,
        declare_use_rviz,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_robot_model')),
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        sensors_launch,
        ekf_launch,
        slam_launch,
        nav_launch,
    ])
