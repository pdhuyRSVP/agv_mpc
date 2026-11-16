from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    description_share = get_package_share_directory('agv_description')
    urdf_file = os.path.join(description_share, 'urdf', 'agv0509test6.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    default_nav2_params = os.path.join(
        get_package_share_directory('agv_navigation'),
        'config',
        'nav2_mppi_lidar.yaml'
    )
    default_rviz_config = os.path.join(
        get_package_share_directory('agv_navigation'),
        'config',
        'nav2_lidar_map.rviz'
    )
    default_nav_to_pose_bt = os.path.join(
        get_package_share_directory('agv_navigation'),
        'behavior_trees',
        'navigate_to_pose_no_reverse.xml'
    )
    default_map = '/home/hcmute/agv_mpc/maps/lidar_2d/map_c205_20261029.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the occupancy grid map yaml for AMCL.'
        ),
        DeclareLaunchArgument(
            'amcl_tf_broadcast',
            default_value='true',
            description='Whether AMCL publishes map->odom. Set false when slam_toolbox or GTSAM owns map->odom.'
        ),
        DeclareLaunchArgument(
            'publish_robot_model',
            default_value='true',
            description='Start robot_state_publisher so RViz can display the robot model.'
        ),
        DeclareLaunchArgument(
            'start_localization',
            default_value='true',
            description='Start map_server and AMCL. Set false when localization is already running.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Full path to the Nav2 params file.'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz with fixed frame map.'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Full path to RViz config.'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_robot_model')),
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),

        # Map server (nếu localization mode)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_localization')),
            parameters=[LaunchConfiguration('params_file'), {
                'yaml_filename': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),

        # AMCL (nếu localization mode)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_localization')),
            parameters=[LaunchConfiguration('params_file'), {
                'tf_broadcast': LaunchConfiguration('amcl_tf_broadcast'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),

        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('cmd_vel', '/cmd_vel_nav_raw'),
            ],
        ),

        # Behavior Tree Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'default_nav_to_pose_bt_xml': default_nav_to_pose_bt,
            }]
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Recoveries
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('cmd_vel', '/cmd_vel_nav_raw'),
            ],
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('cmd_vel', '/cmd_vel_nav_raw'),
                ('cmd_vel_smoothed', '/cmd_vel_nav_smooth'),
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),

        Node(
            package='agv_navigation',
            executable='turn_in_place_cmd_filter.py',
            name='turn_in_place_cmd_filter',
            output='screen',
            parameters=[{
                'input_topic': '/cmd_vel_nav_smooth',
                'output_topic': '/cmd_vel',
                'angular_enter_threshold': 0.18,
                'angular_exit_threshold': 0.06,
                'hard_stop_angular_threshold': 0.38,
                'min_forward_to_keep_turning': 0.04,
                'zero_y_when_turning': True,
                'allow_reverse': True,
                'reverse_max_abs': 0.08,
                'reverse_block_angular_z': 0.20,
                'max_forward_velocity': 0.40,
                'max_angular_velocity': 0.40,
                'corner_slowdown_start': 0.10,
                'corner_full_slowdown_angular': 0.30,
                'corner_max_forward_velocity': 0.20,
            }],
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('start_localization')),
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'autostart': True,
                        'node_names': [
                            'map_server',
                            'amcl',
                            'planner_server',
                            'controller_server',
                            'smoother_server',
                            'behavior_server',
                            'bt_navigator',
                            'waypoint_follower',
                            'velocity_smoother',
                        ],
                    }],
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    condition=UnlessCondition(LaunchConfiguration('start_localization')),
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'autostart': True,
                        'node_names': [
                            'planner_server',
                            'controller_server',
                            'smoother_server',
                            'behavior_server',
                            'bt_navigator',
                            'waypoint_follower',
                            'velocity_smoother',
                        ],
                    }],
                ),
            ],
        ),
    ])
