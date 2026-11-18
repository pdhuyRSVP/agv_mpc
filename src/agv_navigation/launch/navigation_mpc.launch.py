"""Nav2 planning + standalone MPC for the physical differential-drive AGV.

Run only after sensor drivers and EKF are up.
Do not run alongside the existing MPPI navigation launch.
Default MPC commands are intentionally isolated from motor commands.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_share = get_package_share_directory('agv_navigation')
    description_share = get_package_share_directory('agv_description')

    default_params = os.path.join(nav_share, 'config', 'nav2_params_mpc.yaml')
    default_rviz = os.path.join(nav_share, 'config', 'nav2_lidar_map.rviz')
    default_map = '/home/hcmute/agv_mpc/maps/lidar_2d/map_c205_20261029.yaml'
    urdf_path = os.path.join(description_share, 'urdf', 'agv0509test6.urdf')
    ekf_config = os.path.join(get_package_share_directory('agv_control'),'config','ekf.yaml')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')

    # Dung mot bo Nav2 rieng cho MPC; khong khoi chay MPPI song song.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {
            'yaml_filename': map_file,
            'use_sim_time': False,
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {
            'tf_broadcast': ParameterValue(
                LaunchConfiguration('amcl_tf_broadcast'), value_type=bool
            ),
            'use_sim_time': False,
        }],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': False}],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': False}],
    )

    # Lifecycle chi quan ly cac node Nav2 co lifecycle.
    lifecycle_manager = TimerAction(
        period=2.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mpc_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'smoother_server',
                ],
            }],
        )],
    )

    # Goal Manager yeu cau NavFn -> Simple Smoother qua Nav2 actions.
    goal_manager = Node(
        package='navigation_mpc',
        executable='goal_manager',
        name='goal_manager',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # Mac dinh chi xuat len topic thu nghiem, KHONG noi voi /cmd_vel.
    mpc_controller = Node(
        package='navigation_mpc',
        executable='mpc_controller',
        name='mpc_controller',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_base_frame': 'Base_footprint',
            'cmd_vel_topic': '/cmd_vel',
        }],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_robot_model')),
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', 'odom')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('map', default_value=default_map),
        # Neu node khac publish map->odom, phai chuyen tham so nay thanh false.
        DeclareLaunchArgument('amcl_tf_broadcast', default_value='true'),
        DeclareLaunchArgument('publish_robot_model', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        robot_state_publisher,
        map_server,
        amcl,
        planner_server,
        smoother_server,
        lifecycle_manager,
        goal_manager,
        mpc_controller,
        rviz,
        ekf_node,
    ])