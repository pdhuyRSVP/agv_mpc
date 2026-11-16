from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    description_share = get_package_share_directory('agv_description')
    urdf_file = os.path.join(description_share, 'urdf', 'agv0509test6.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    default_params_file = os.path.join(
        get_package_share_directory('agv_slam'),
        'config',
        'slam_toolbox_mapping.yaml'
    )
    default_rviz_config = os.path.join(
        get_package_share_directory('agv_navigation'),
        'config',
        'nav2_lidar_map.rviz'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_robot_model')),
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_control'),
                'launch',
                'ekf.launch.py',
            )
        ),
        condition=IfCondition(LaunchConfiguration('start_ekf')),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('publish_robot_model', default_value='true'),
        DeclareLaunchArgument('start_ekf', default_value='true'),
        DeclareLaunchArgument('slam_start_delay', default_value='8.0'),
        DeclareLaunchArgument('slam_params_file', default_value=default_params_file),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        robot_state_publisher,
        ekf_launch,
        TimerAction(period=LaunchConfiguration('slam_start_delay'), actions=[slam_toolbox_launch]),
        rviz_node,
    ])
