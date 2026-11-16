#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('orb3_agv_sim')
    slam_share = get_package_share_directory('agv_slam')
    nav_share = get_package_share_directory('agv_navigation')
    description_share = get_package_share_directory('agv_description')
    urdf_file = os.path.join(description_share, 'urdf', 'agv0509test6.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    default_map = '/home/hcmute/agv_mpc/maps/lidar_2d/current_20260616_185758.yaml'
    default_slam_params = os.path.join(
        slam_share, 'config', 'slam_toolbox_localization.yaml')
    default_nav_params = os.path.join(nav_share, 'config', 'nav2_mppi_lidar.yaml')

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

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='graph_map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'topic_name': '/map',
            'frame_id': 'map',
            'use_sim_time': False,
        }],
    )
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_graph_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['graph_map_server'],
            'use_sim_time': False,
        }],
    )

    rf2o_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'rf2o_laser_odometry.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_scan_to_scan')),
        launch_arguments={
            'laser_scan_topic': LaunchConfiguration('rf2o_scan_topic'),
            'odom_topic': LaunchConfiguration('rf2o_odom_topic'),
            'odom_frame_id': 'odom',
            'base_frame_id': 'Base_footprint',
            'freq': LaunchConfiguration('rf2o_frequency'),
        }.items(),
    )
    graph = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'gtsam_pose2_fusion.launch.py')),
        launch_arguments={
            'ekf_odom_topic': LaunchConfiguration('ekf_odom_topic'),
            'orb_pose_topic': LaunchConfiguration('orb_pose_topic'),
            'orb_tracking_state_topic': LaunchConfiguration('orb_tracking_state_topic'),
            'scan_odom_topic': LaunchConfiguration('graph_scan_odom_topic'),
            'turn_rate_topic': LaunchConfiguration('turn_rate_topic'),
            'turn_rate_timeout_sec': LaunchConfiguration('turn_rate_timeout_sec'),
            'global_pose_topic': LaunchConfiguration('global_pose_topic'),
            'use_global_pose_factor': 'true',
            'global_pose_is_sensor_frame': LaunchConfiguration('global_pose_is_sensor_frame'),
            'global_pose_sensor_frame': LaunchConfiguration('global_pose_sensor_frame'),
            'global_pose_yaw_offset': LaunchConfiguration('global_pose_yaw_offset'),
            'global_pose_auto_yaw_offset': LaunchConfiguration('global_pose_auto_yaw_offset'),
            'map_yaml_file': LaunchConfiguration('map'),
            'enforce_scan_map_bounds': LaunchConfiguration('enforce_scan_map_bounds'),
            'scan_map_bounds_margin': LaunchConfiguration('scan_map_bounds_margin'),
            'scan_map_occupancy_gate_radius': LaunchConfiguration(
                'scan_map_occupancy_gate_radius'),
            'init_from_scan_map': LaunchConfiguration('init_from_scan_map'),
            'scan_map_init_stable_count': LaunchConfiguration('scan_map_init_stable_count'),
            'scan_map_init_max_xy_spread': LaunchConfiguration('scan_map_init_max_xy_spread'),
            'scan_map_init_max_yaw_spread': LaunchConfiguration('scan_map_init_max_yaw_spread'),
            'scan_map_init_min_duration_sec': LaunchConfiguration('scan_map_init_min_duration_sec'),
            'scan_map_init_timeout_sec': LaunchConfiguration('scan_map_init_timeout_sec'),
            'init_prior_use_scan_map_yaw': LaunchConfiguration('init_prior_use_scan_map_yaw'),
            'scan_map_init_ignore_yaw_stability': LaunchConfiguration('scan_map_init_ignore_yaw_stability'),
            'init_prior_sigma_x': LaunchConfiguration('init_prior_sigma_x'),
            'init_prior_sigma_y': LaunchConfiguration('init_prior_sigma_y'),
            'init_prior_sigma_yaw': LaunchConfiguration('init_prior_sigma_yaw'),
            'ignore_scan_map_after_init_sec': LaunchConfiguration('ignore_scan_map_after_init_sec'),
            'fallback_to_initial_pose_on_scan_map_timeout': LaunchConfiguration('fallback_to_initial_pose_on_scan_map_timeout'),
            'orb_pose_is_base': 'true',
            'publish_tf': LaunchConfiguration('publish_tf'),
            'tf_future_tolerance_sec': LaunchConfiguration('tf_future_tolerance_sec'),
            'use_initial_pose': LaunchConfiguration('use_initial_pose'),
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
            'wait_for_global_pose_before_init': LaunchConfiguration('wait_for_global_pose_before_init'),
            'seed_from_global_pose': LaunchConfiguration('seed_from_global_pose'),
            # In long rectangular corridors RGB-D can relocalize to a visually
            # similar opposite direction. Keep ORB as a yaw-only weak cue and
            # never let its translation pull the 2D graph.
            'use_orb_translation': 'false',
            'use_orb_yaw': 'true',
            'require_orb_tracking_state': 'true',
            'reject_orb_when_turning': 'true',
            'orb_turn_reject_wz': '0.25',
            'orb_turn_block_sec': '1.5',
            'ekf_sigma_x': LaunchConfiguration('ekf_sigma_x'),
            'ekf_sigma_y': LaunchConfiguration('ekf_sigma_y'),
            'ekf_sigma_yaw': LaunchConfiguration('ekf_sigma_yaw'),
            'orb_sigma_x': LaunchConfiguration('orb_sigma_x'),
            'orb_sigma_y': LaunchConfiguration('orb_sigma_y'),
            'orb_sigma_yaw': LaunchConfiguration('orb_sigma_yaw'),
            'scan_odom_sigma_x': LaunchConfiguration('scan_odom_sigma_x'),
            'scan_odom_sigma_y': LaunchConfiguration('scan_odom_sigma_y'),
            'scan_odom_sigma_yaw': LaunchConfiguration('scan_odom_sigma_yaw'),
            'max_scan_ekf_disagreement_xy': LaunchConfiguration('max_scan_ekf_disagreement_xy'),
            'max_scan_ekf_disagreement_yaw': LaunchConfiguration('max_scan_ekf_disagreement_yaw'),
            'scan_odom_max_yaw_rate': LaunchConfiguration('scan_odom_max_yaw_rate'),
            'reject_scan_odom_when_not_forward': LaunchConfiguration(
                'reject_scan_odom_when_not_forward'),
            'scan_odom_min_forward_vx': LaunchConfiguration('scan_odom_min_forward_vx'),
            'scan_map_sigma_x': LaunchConfiguration('scan_map_sigma_x'),
            'scan_map_sigma_y': LaunchConfiguration('scan_map_sigma_y'),
            'scan_map_sigma_yaw': LaunchConfiguration('scan_map_sigma_yaw'),
            'scan_map_untrusted_yaw_sigma': LaunchConfiguration('scan_map_untrusted_yaw_sigma'),
            'trust_scan_map_yaw': LaunchConfiguration('trust_scan_map_yaw'),
            'enable_home_scan_map_correction': LaunchConfiguration(
                'enable_home_scan_map_correction'),
            'scan_map_only_near_home': LaunchConfiguration('scan_map_only_near_home'),
            'home_map_x': LaunchConfiguration('home_map_x'),
            'home_map_y': LaunchConfiguration('home_map_y'),
            'home_scan_map_radius': LaunchConfiguration('home_scan_map_radius'),
            'home_scan_map_sigma_xy': LaunchConfiguration('home_scan_map_sigma_xy'),
            'home_scan_map_max_innovation_xy': LaunchConfiguration(
                'home_scan_map_max_innovation_xy'),
            'robust_huber_k': LaunchConfiguration('robust_huber_k'),
            'reject_scan_map_when_turning': LaunchConfiguration('reject_scan_map_when_turning'),
            'reject_scan_map_when_not_forward': LaunchConfiguration('reject_scan_map_when_not_forward'),
            'scan_map_min_forward_vx': LaunchConfiguration('scan_map_min_forward_vx'),
            'scan_map_turn_reject_wz': LaunchConfiguration('scan_map_turn_reject_wz'),
            'scan_map_turn_block_sec': LaunchConfiguration('scan_map_turn_block_sec'),
            'scan_map_selective_updates': LaunchConfiguration('scan_map_selective_updates'),
            'scan_map_update_min_interval_sec': LaunchConfiguration(
                'scan_map_update_min_interval_sec'),
            'scan_map_update_stable_count': LaunchConfiguration(
                'scan_map_update_stable_count'),
            'scan_map_update_max_innovation_spread_xy': LaunchConfiguration(
                'scan_map_update_max_innovation_spread_xy'),
            'scan_map_update_max_innovation_spread_yaw': LaunchConfiguration(
                'scan_map_update_max_innovation_spread_yaw'),
            'scan_map_update_min_duration_sec': LaunchConfiguration(
                'scan_map_update_min_duration_sec'),
            'scan_map_relocalize_trigger_topic': LaunchConfiguration(
                'global_initializer_trigger_topic'),
            'enable_scan_map_candidate_gate': LaunchConfiguration(
                'enable_scan_map_candidate_gate'),
            'scan_map_candidate_required_count': LaunchConfiguration(
                'scan_map_candidate_required_count'),
            'scan_map_candidate_window_sec': LaunchConfiguration(
                'scan_map_candidate_window_sec'),
            'scan_map_candidate_max_spread_xy': LaunchConfiguration(
                'scan_map_candidate_max_spread_xy'),
            'scan_map_candidate_max_spread_yaw': LaunchConfiguration(
                'scan_map_candidate_max_spread_yaw'),
            'scan_map_max_single_correction_xy': LaunchConfiguration(
                'scan_map_max_single_correction_xy'),
            'scan_map_max_single_correction_yaw': LaunchConfiguration(
                'scan_map_max_single_correction_yaw'),
            'scan_map_reject_correction_xy': LaunchConfiguration(
                'scan_map_reject_correction_xy'),
            'scan_map_reject_correction_yaw': LaunchConfiguration(
                'scan_map_reject_correction_yaw'),
            'scan_map_use_xy_only_when_yaw_bad': LaunchConfiguration(
                'scan_map_use_xy_only_when_yaw_bad'),
            'scan_map_xy_only_yaw_reject_threshold': LaunchConfiguration(
                'scan_map_xy_only_yaw_reject_threshold'),
            'scan_map_suspect_reject_count_threshold': LaunchConfiguration(
                'scan_map_suspect_reject_count_threshold'),
            'scan_map_suspect_cooldown_sec': LaunchConfiguration(
                'scan_map_suspect_cooldown_sec'),
            'scan_map_relocalize_cooldown_sec': LaunchConfiguration(
                'scan_map_relocalize_cooldown_sec'),
            'enable_scan_map_auto_relocalization': LaunchConfiguration(
                'enable_scan_map_auto_relocalization'),
            'reject_scan_map_backward_when_forward': LaunchConfiguration(
                'reject_scan_map_backward_when_forward'),
            'scan_map_backward_reject_min_vx': LaunchConfiguration(
                'scan_map_backward_reject_min_vx'),
            'scan_map_backward_reject_distance': LaunchConfiguration(
                'scan_map_backward_reject_distance'),
            'enable_scan_map_bounded_correction': LaunchConfiguration(
                'enable_scan_map_bounded_correction'),
            'use_amcl_covariance': 'false',
            'max_amcl_jump_xy': LaunchConfiguration('max_amcl_jump_xy'),
            'max_amcl_jump_yaw': LaunchConfiguration('max_amcl_jump_yaw'),
            'max_global_innovation_xy': LaunchConfiguration('max_global_innovation_xy'),
            'max_global_innovation_yaw': LaunchConfiguration('max_global_innovation_yaw'),
            'max_association_dt': LaunchConfiguration('max_association_dt'),
            'min_node_translation': LaunchConfiguration('min_node_translation'),
            'min_node_rotation': LaunchConfiguration('min_node_rotation'),
            'max_node_dt': LaunchConfiguration('max_node_dt'),
            'create_node_on_time_gap': LaunchConfiguration('create_node_on_time_gap'),
            'use_turn_rate_yaw_when_ekf_stalled': LaunchConfiguration(
                'use_turn_rate_yaw_when_ekf_stalled'),
            'turn_rate_yaw_min_wz': LaunchConfiguration('turn_rate_yaw_min_wz'),
            'turn_rate_yaw_max_pose_delta': LaunchConfiguration(
                'turn_rate_yaw_max_pose_delta'),
            'enable_diagnostics': LaunchConfiguration('enable_diagnostics'),
            'diagnostics_output_dir': LaunchConfiguration('diagnostics_output_dir'),
            'enable_run_logging': LaunchConfiguration('enable_run_logging'),
            'run_log_root': LaunchConfiguration('run_log_root'),
            'run_log_tag': LaunchConfiguration('run_log_tag'),
            'run_pose_log_min_interval_sec': LaunchConfiguration(
                'run_pose_log_min_interval_sec'),
            'run_rejected_factor_log_min_interval_sec': LaunchConfiguration(
                'run_rejected_factor_log_min_interval_sec'),
            'run_scan_map_debug_min_interval_sec': LaunchConfiguration(
                'run_scan_map_debug_min_interval_sec'),
            'nav2_status_topic': LaunchConfiguration('nav2_status_topic'),
            'nav2_feedback_topic': LaunchConfiguration('nav2_feedback_topic'),
            'goal_pose_topic': LaunchConfiguration('goal_pose_topic'),
        }.items(),
    )
    slam_toolbox_reference = Node(
        package='slam_toolbox',
        executable=LaunchConfiguration('slam_toolbox_executable'),
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_slam_toolbox_reference')),
        parameters=[
            LaunchConfiguration('slam_toolbox_params_file'),
            {
                'use_sim_time': False,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': LaunchConfiguration('slam_toolbox_base_frame'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'mode': LaunchConfiguration('slam_toolbox_mode'),
                'map_file_name': LaunchConfiguration('slam_toolbox_map_file_name'),
                'map_start_at_dock': LaunchConfiguration('slam_toolbox_map_start_at_dock'),
                'use_scan_matching': True,
                'do_loop_closing': LaunchConfiguration('slam_toolbox_do_loop_closing'),
                'transform_publish_period': LaunchConfiguration('slam_toolbox_transform_publish_period'),
            },
        ],
        remappings=[
            ('map', '/slam_toolbox/map'),
            ('/tf', '/slam_toolbox/tf'),
        ],
    )
    slam_toolbox_pose_bridge = Node(
        package='orb3_agv_sim',
        executable='slam_toolbox_tf_pose_bridge.py',
        name='slam_toolbox_tf_pose_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_slam_toolbox_reference')),
        parameters=[{
            'tf_map_frame': 'map',
            'output_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': LaunchConfiguration('slam_toolbox_base_frame'),
            'pose_topic': LaunchConfiguration('global_pose_topic'),
            'public_tf_topic': '/tf',
            'private_tf_topic': '/slam_toolbox/tf',
            'relay_public_tf': True,
            'allowed_public_tf_pairs': [
                'odom:Base_footprint',
                'Base_footprint:Base_Link',
                'Base_Link:Lidar',
                'Base_footprint:Lidar',
            ],
            'wait_for_posegraph_loaded': LaunchConfiguration('slam_toolbox_wait_for_posegraph_loaded'),
            'posegraph_loaded_topic': LaunchConfiguration('slam_toolbox_posegraph_loaded_topic'),
            'publish_hz': LaunchConfiguration('slam_toolbox_pose_bridge_hz'),
            'enable_graph_pose_gate': LaunchConfiguration('slam_toolbox_enable_graph_pose_gate'),
            'graph_pose_topic': '/graph/pose',
            'max_graph_pose_age_sec': LaunchConfiguration('slam_toolbox_graph_pose_gate_max_age_sec'),
            'max_graph_distance_xy': LaunchConfiguration('slam_toolbox_graph_pose_gate_xy'),
            'max_graph_distance_yaw': LaunchConfiguration('slam_toolbox_graph_pose_gate_yaw'),
            'max_tf_time_delta_sec': LaunchConfiguration('slam_toolbox_max_tf_time_delta_sec'),
            'max_pose_age_sec': LaunchConfiguration('slam_toolbox_max_pose_age_sec'),
            'private_tf_future_tolerance_sec': LaunchConfiguration('slam_toolbox_private_tf_future_tolerance_sec'),
            'tf_lookup_timeout_sec': LaunchConfiguration('slam_toolbox_tf_lookup_timeout_sec'),
            'allow_latest_tf_fallback': True,
            'drop_on_tf_time_mismatch': True,
            'cov_x': LaunchConfiguration('scan_map_sigma_x'),
            'cov_y': LaunchConfiguration('scan_map_sigma_y'),
            'cov_yaw': LaunchConfiguration('scan_map_sigma_yaw'),
        }],
    )
    slam_toolbox_posegraph_loader = Node(
        package='orb3_agv_sim',
        executable='slam_toolbox_global_posegraph_loader.py',
        name='slam_toolbox_global_posegraph_loader',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_slam_toolbox_posegraph_loader')),
        parameters=[{
            'pose_topic': LaunchConfiguration('global_initializer_pose_topic'),
            'map_file_name': LaunchConfiguration('slam_toolbox_map_file_name'),
            'deserialize_service': '/slam_toolbox/deserialize_map',
            'loaded_topic': LaunchConfiguration('slam_toolbox_posegraph_loaded_topic'),
            'slam_pose_topic': LaunchConfiguration('global_pose_topic'),
            'graph_pose_topic': '/graph/pose',
            'odom_topic': LaunchConfiguration('ekf_odom_topic'),
            'map_topic': '/map',
            'trigger_topic': LaunchConfiguration('global_initializer_trigger_topic'),
            'relocalize_service': '/relocalize_slam_toolbox',
            'load_on_start': LaunchConfiguration('slam_toolbox_posegraph_load_on_start'),
            'initial_x': LaunchConfiguration('slam_toolbox_posegraph_initial_x'),
            'initial_y': LaunchConfiguration('slam_toolbox_posegraph_initial_y'),
            'initial_yaw': LaunchConfiguration('slam_toolbox_posegraph_initial_yaw'),
            'enable_relocalization': LaunchConfiguration('enable_relocalization'),
            'relocalize_distance_threshold': LaunchConfiguration(
                'relocalize_distance_threshold'),
            'relocalize_yaw_threshold': LaunchConfiguration(
                'relocalize_yaw_threshold'),
            'relocalize_reject_count_threshold': LaunchConfiguration(
                'relocalize_reject_count_threshold'),
            'relocalize_cooldown_sec': LaunchConfiguration(
                'relocalize_cooldown_sec'),
        }],
    )
    amcl_reference = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl_reference',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_amcl_reference')),
        parameters=[
            LaunchConfiguration('amcl_params_file'),
            {
                'tf_broadcast': False,
                'use_sim_time': False,
                'scan_topic': LaunchConfiguration('scan_topic'),
                'global_frame_id': 'map',
                'odom_frame_id': 'odom',
                'base_frame_id': 'Base_footprint',
            },
        ],
        remappings=[
            ('amcl_pose', '/amcl_reference/pose'),
        ],
    )
    amcl_reference_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl_reference',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_amcl_reference')),
        parameters=[{
            'autostart': True,
            'node_names': ['amcl_reference'],
            'use_sim_time': False,
        }],
    )
    amcl_reference_home_seed = Node(
        package='orb3_agv_sim',
        executable='initial_pose_publisher.py',
        name='amcl_reference_home_initial_pose',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_amcl_reference')),
        parameters=[{
            'frame_id': 'map',
            'x': LaunchConfiguration('home_map_x'),
            'y': LaunchConfiguration('home_map_y'),
            'yaw': LaunchConfiguration('home_map_yaw'),
            'cov_xy': 0.25,
            'cov_yaw': 0.0685,
            'publish_count': 12,
            'period_sec': 0.5,
        }],
    )
    global_scan_initializer = Node(
        package='orb3_agv_sim',
        executable='global_scan_initializer.py',
        name='global_scan_initializer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_global_scan_initializer')),
        parameters=[{
            'map_topic': '/map',
            'scan_topic': LaunchConfiguration('scan_topic'),
            'initialpose_topic': LaunchConfiguration('global_initializer_initialpose_topic'),
            'output_pose_topic': LaunchConfiguration('global_initializer_pose_topic'),
            'status_topic': LaunchConfiguration('global_initializer_status_topic'),
            'trigger_topic': LaunchConfiguration('global_initializer_trigger_topic'),
            'map_frame': 'map',
            'base_frame': 'Base_footprint',
            'global_xy_step': LaunchConfiguration('global_initializer_xy_step'),
            'global_yaw_step': LaunchConfiguration('global_initializer_yaw_step'),
            'fine_xy_radius': LaunchConfiguration('global_initializer_fine_xy_radius'),
            'fine_xy_step': LaunchConfiguration('global_initializer_fine_xy_step'),
            'fine_yaw_radius': LaunchConfiguration('global_initializer_fine_yaw_radius'),
            'fine_yaw_step': LaunchConfiguration('global_initializer_fine_yaw_step'),
            'max_best_score': LaunchConfiguration('global_initializer_max_score'),
            'min_valid_fraction': LaunchConfiguration('global_initializer_min_valid_fraction'),
            'min_score_margin': LaunchConfiguration('global_initializer_min_score_margin'),
            'required_successes': LaunchConfiguration('global_initializer_required_successes'),
            'ambiguous_required_successes': LaunchConfiguration(
                'global_initializer_ambiguous_required_successes'),
            'stable_xy_tolerance': LaunchConfiguration(
                'global_initializer_stable_xy_tolerance'),
            'stable_yaw_tolerance': LaunchConfiguration(
                'global_initializer_stable_yaw_tolerance'),
            'enable_home_seed': LaunchConfiguration('global_initializer_enable_home_seed'),
            'home_x': LaunchConfiguration('home_map_x'),
            'home_y': LaunchConfiguration('home_map_y'),
            'home_seed_radius': LaunchConfiguration('global_initializer_home_seed_radius'),
            'home_required_successes': LaunchConfiguration(
                'global_initializer_home_required_successes'),
            'home_yaw_tolerance': LaunchConfiguration(
                'global_initializer_home_yaw_tolerance'),
            'max_scan_attempts': LaunchConfiguration('global_initializer_max_scan_attempts'),
        }],
    )
    scan_to_map_matcher = Node(
        package='orb3_agv_sim',
        executable='scan_to_map_matcher.py',
        name='scan_to_map_matcher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_scan_to_map_matcher')),
        parameters=[{
            'map_topic': '/map',
            'scan_topic': LaunchConfiguration('scan_topic'),
            'initial_pose_topic': '/graph/pose',
            'initialpose_topic': '/initialpose',
            'output_topic': LaunchConfiguration('global_pose_topic'),
            'map_frame': 'map',
            'base_frame': 'Base_footprint',
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
            'max_mean_distance': LaunchConfiguration(
                'scan_matcher_max_mean_distance'),
            'max_correction_xy': LaunchConfiguration(
                'scan_matcher_max_correction_xy'),
        }],
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'start_localization': 'false',
            'publish_robot_model': 'false',
            'use_sim_time': 'false',
            'use_rviz': LaunchConfiguration('use_rviz'),
            'params_file': LaunchConfiguration('nav2_params_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_nav2')),
    )
    nav2_lifecycle_recovery = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            (
                # Graph Factor owns map->odom in this stack. If Nav2 starts
                # before that TF is stable, lifecycle_manager can leave BT or
                # velocity_smoother inactive; inactive BT rejects goals.
                # The local costmap only needs odom->base, so activate the
                # controller branch as a recovery path even while map->odom is
                # still waiting for scan-map initialization.
                'ros2 lifecycle set /controller_server activate >/tmp/weborb_controller_activate.log 2>&1 || true; '
                'ros2 lifecycle set /local_costmap/local_costmap activate >/tmp/weborb_local_costmap_activate.log 2>&1 || true; '
                'ros2 lifecycle set /bt_navigator activate >/tmp/weborb_bt_activate.log 2>&1 || true; '
                'ros2 lifecycle set /velocity_smoother activate >/tmp/weborb_velocity_smoother_activate.log 2>&1 || true'
            ),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_nav2')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'start_slam_toolbox_reference',
            default_value='true',
            description=(
                'Run slam_toolbox localization as the outer-loop scan-to-map pose source.'
            ),
        ),
        DeclareLaunchArgument(
            'start_global_scan_initializer',
            default_value='true',
            description='Use /scan_filtered + /map to estimate the initial map pose for slam_toolbox localization.',
        ),
        DeclareLaunchArgument(
            'start_slam_toolbox_posegraph_loader',
            default_value='true',
            description='Call /slam_toolbox/deserialize_map at the global initializer pose when relocalizing.',
        ),
        DeclareLaunchArgument(
            'global_initializer_initialpose_topic',
            default_value='/initialpose',
            description='Legacy/debug PoseWithCovariance topic used only when start_global_scan_initializer is true.',
        ),
        DeclareLaunchArgument(
            'global_initializer_pose_topic',
            default_value='/global_scan_initializer/pose',
            description='Debug output pose from the global scan initializer.',
        ),
        DeclareLaunchArgument(
            'global_initializer_status_topic',
            default_value='/global_scan_initializer/status',
            description='Debug status string from the global scan initializer.',
        ),
        DeclareLaunchArgument(
            'global_initializer_trigger_topic',
            default_value='/global_scan_initializer/trigger',
            description='Bool trigger used to restart global scan matching.',
        ),
        DeclareLaunchArgument('global_initializer_xy_step', default_value='0.25'),
        DeclareLaunchArgument('global_initializer_yaw_step', default_value='0.261799'),
        DeclareLaunchArgument('global_initializer_fine_xy_radius', default_value='0.30'),
        DeclareLaunchArgument('global_initializer_fine_xy_step', default_value='0.05'),
        DeclareLaunchArgument('global_initializer_fine_yaw_radius', default_value='0.261799'),
        DeclareLaunchArgument('global_initializer_fine_yaw_step', default_value='0.052360'),
        DeclareLaunchArgument('global_initializer_max_score', default_value='0.22'),
        DeclareLaunchArgument('global_initializer_min_valid_fraction', default_value='0.70'),
        DeclareLaunchArgument('global_initializer_min_score_margin', default_value='0.020'),
        DeclareLaunchArgument('global_initializer_required_successes', default_value='2'),
        DeclareLaunchArgument(
            'global_initializer_ambiguous_required_successes',
            default_value='3',
            description='Stable repeated matches required when corridor candidates have a low score margin.',
        ),
        DeclareLaunchArgument(
            'global_initializer_stable_xy_tolerance',
            default_value='0.35',
            description='Maximum XY spread in metres for repeated global initialization candidates.',
        ),
        DeclareLaunchArgument(
            'global_initializer_stable_yaw_tolerance',
            default_value='0.209440',
            description='Maximum yaw spread in radians for repeated global initialization candidates.',
        ),
        DeclareLaunchArgument('global_initializer_enable_home_seed', default_value='true'),
        DeclareLaunchArgument('global_initializer_home_seed_radius', default_value='0.80'),
        DeclareLaunchArgument('global_initializer_home_required_successes', default_value='3'),
        DeclareLaunchArgument('global_initializer_home_yaw_tolerance', default_value='0.349066'),
        DeclareLaunchArgument('global_initializer_max_scan_attempts', default_value='60'),
        DeclareLaunchArgument(
            'global_pose_topic',
            default_value='/slam_toolbox/pose',
            description='Absolute scan-to-map pose topic consumed by the GTSAM graph.',
        ),
        DeclareLaunchArgument(
            'global_pose_is_sensor_frame',
            default_value='false',
            description='Use false when slam_toolbox publishes Base_footprint pose directly.',
        ),
        DeclareLaunchArgument(
            'global_pose_sensor_frame',
            default_value='Lidar',
            description='Sensor frame used when global_pose_is_sensor_frame is true.',
        ),
        DeclareLaunchArgument(
            'global_pose_yaw_offset',
            default_value='0.0',
            description='Yaw offset added to scan-to-map/global pose before adding graph factor.',
        ),
        DeclareLaunchArgument(
            'global_pose_auto_yaw_offset',
            default_value='false',
            description='Calibrate scan-to-map/global yaw offset from first pose to initial_yaw.',
        ),
        DeclareLaunchArgument('init_from_scan_map', default_value='true'),
        DeclareLaunchArgument('scan_map_init_stable_count', default_value='8'),
        DeclareLaunchArgument('scan_map_init_max_xy_spread', default_value='0.12'),
        DeclareLaunchArgument('scan_map_init_max_yaw_spread', default_value='0.15'),
        DeclareLaunchArgument('scan_map_init_min_duration_sec', default_value='0.5'),
        DeclareLaunchArgument('scan_map_init_timeout_sec', default_value='15.0'),
        DeclareLaunchArgument('init_prior_use_scan_map_yaw', default_value='true'),
        DeclareLaunchArgument('scan_map_init_ignore_yaw_stability', default_value='false'),
        DeclareLaunchArgument('init_prior_sigma_x', default_value='0.10'),
        DeclareLaunchArgument('init_prior_sigma_y', default_value='0.10'),
        DeclareLaunchArgument('init_prior_sigma_yaw', default_value='0.50'),
        DeclareLaunchArgument('ignore_scan_map_after_init_sec', default_value='0.5'),
        DeclareLaunchArgument('fallback_to_initial_pose_on_scan_map_timeout', default_value='false'),
        DeclareLaunchArgument(
            'start_scan_to_map_matcher',
            default_value='false',
            description='Run occupancy-grid scan-to-map matcher and publish the graph global pose factor.',
        ),
        DeclareLaunchArgument(
            'scan_matcher_max_mean_distance',
            default_value='0.35',
            description='Maximum mean LiDAR endpoint distance for accepting an occupancy-map match.',
        ),
        DeclareLaunchArgument(
            'scan_matcher_max_correction_xy',
            default_value='0.70',
            description='Maximum local XY correction allowed from the graph/dock search seed.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_params_file',
            default_value=default_slam_params,
        ),
        DeclareLaunchArgument(
            'slam_toolbox_executable',
            default_value='localization_slam_toolbox_node',
            description='Use localization_slam_toolbox_node for mission navigation.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_mode',
            default_value='localization',
            description='slam_toolbox mode override. Mission navigation uses localization only.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_do_loop_closing',
            default_value='false',
            description='Enable loop closing only for a deliberate refine pass, not the normal navigation mission.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_map_file_name',
            default_value='/home/hcmute/agv_mpc/maps/lidar_2d/graphs/current_20260616_185758',
            description='Serialized slam_toolbox posegraph basename used by slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_map_start_at_dock',
            default_value='false',
            description='Start slam_toolbox localization from the dock/start node instead of global scan initialization.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_base_frame',
            default_value='Base_footprint',
            description=(
                'Frame whose pose slam_toolbox publishes. Use Base_footprint '
                'for graph fusion so scan-to-map pose is already robot pose.'
            ),
        ),
        DeclareLaunchArgument(
            'slam_toolbox_internal_map_frame',
            default_value='map',
            description=(
                'Deprecated compatibility argument. Ignored: slam_toolbox '
                'localization is forced to map_frame=map so /slam_toolbox/pose '
                'is not a relabeled private-frame pose.'
            ),
        ),
        DeclareLaunchArgument(
            'slam_toolbox_transform_publish_period',
            default_value='0.05',
            description='Private map->odom TF period used only by slam_toolbox and the pose bridge.',
        ),
        DeclareLaunchArgument('slam_toolbox_pose_bridge_hz', default_value='10.0'),
        DeclareLaunchArgument(
            'slam_toolbox_enable_graph_pose_gate',
            default_value='false',
            description='Debug-only graph pose gate. Keep false so graph fused pose is not treated as scan-map truth.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_graph_pose_gate_max_age_sec',
            default_value='1.0',
            description='Maximum graph pose age for validating slam_toolbox scan-map output.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_graph_pose_gate_xy',
            default_value='1.0',
            description='Maximum allowed XY difference between slam_toolbox pose and graph pose.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_graph_pose_gate_yaw',
            default_value='3.2',
            description='Maximum allowed yaw difference between slam_toolbox pose and graph pose.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_max_tf_time_delta_sec',
            default_value='0.45',
            description='Maximum private/public TF timestamp delta used by the pose bridge.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_max_pose_age_sec',
            default_value='0.70',
            description='Maximum age of private slam_toolbox map->odom TF.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_private_tf_future_tolerance_sec',
            default_value='0.75',
            description='Allowed future dating of private slam_toolbox map->odom TF.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_tf_lookup_timeout_sec',
            default_value='0.03',
            description='Short timeout for odom->base lookup at the private TF timestamp.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_posegraph_loaded_topic',
            default_value='/slam_toolbox/posegraph_loaded',
            description='Bool gate published after slam_toolbox posegraph is loaded at the matched initial pose.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_wait_for_posegraph_loaded',
            default_value='true',
            description='Prevent /slam_toolbox/pose publication until posegraph_loaded is true.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_posegraph_load_on_start',
            default_value='false',
            description='Deserialize slam_toolbox posegraph immediately at the configured initial map pose.',
        ),
        DeclareLaunchArgument('slam_toolbox_posegraph_initial_x', default_value='0.0'),
        DeclareLaunchArgument('slam_toolbox_posegraph_initial_y', default_value='0.0'),
        DeclareLaunchArgument('slam_toolbox_posegraph_initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('enable_relocalization', default_value='false'),
        DeclareLaunchArgument('relocalize_distance_threshold', default_value='1.0'),
        DeclareLaunchArgument('relocalize_yaw_threshold', default_value='0.8'),
        DeclareLaunchArgument('relocalize_reject_count_threshold', default_value='30'),
        DeclareLaunchArgument('relocalize_cooldown_sec', default_value='5.0'),
        DeclareLaunchArgument(
            'start_amcl_reference',
            default_value='false',
            description='Run AMCL as /amcl_reference/pose only. TF is always disabled.',
        ),
        DeclareLaunchArgument('amcl_params_file', default_value=default_nav_params),
        DeclareLaunchArgument(
            'start_nav2',
            default_value='false',
            description='Run Nav2 using Graph Factor map->odom TF and EKF odom->Base_footprint.',
        ),
        DeclareLaunchArgument(
            'nav2_lifecycle_recovery_delay',
            default_value='28.0',
            description='Delay before retrying BT navigator and velocity smoother activation.',
        ),
        DeclareLaunchArgument(
            'nav2_start_delay',
            default_value='8.0',
            description='Wait for Graph Factor map->odom before starting Nav2.',
        ),
        DeclareLaunchArgument(
            'tf_future_tolerance_sec',
            default_value='0.40',
            description='Future-date map->odom TF from graph to avoid Nav2 extrapolation under CPU load.',
        ),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav_params),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('enable_diagnostics', default_value='true'),
        DeclareLaunchArgument(
            'diagnostics_output_dir',
            default_value='/home/hcmute/agv_mpc/diagnostics',
            description='Directory written once when the graph node shuts down cleanly.',
        ),
        DeclareLaunchArgument(
            'enable_run_logging',
            default_value='true',
            description='Write per-run graph/Nav2/pose CSV logs for later evaluation.',
        ),
        DeclareLaunchArgument(
            'run_log_root',
            default_value='/home/hcmute/agv_mpc/logs/graph_runs',
            description='Root directory for per-run graph evaluation logs.',
        ),
        DeclareLaunchArgument(
            'run_log_tag',
            default_value='graph_localization',
            description='Prefix for each timestamped graph log folder.',
        ),
        DeclareLaunchArgument(
            'run_pose_log_min_interval_sec',
            default_value='0.20',
            description='Minimum interval between pose CSV rows for each source.',
        ),
        DeclareLaunchArgument(
            'run_rejected_factor_log_min_interval_sec',
            default_value='1.0',
            description='Minimum interval between repeated rejected-factor CSV rows.',
        ),
        DeclareLaunchArgument(
            'run_scan_map_debug_min_interval_sec',
            default_value='0.50',
            description='Minimum interval between rejected scan-map debug CSV rows.',
        ),
        DeclareLaunchArgument(
            'nav2_status_topic',
            default_value='/navigate_to_pose/_action/status',
            description='Nav2 NavigateToPose action status topic to record.',
        ),
        DeclareLaunchArgument(
            'nav2_feedback_topic',
            default_value='/navigate_to_pose/_action/feedback',
            description='Nav2 NavigateToPose action feedback topic to record.',
        ),
        DeclareLaunchArgument(
            'goal_pose_topic',
            default_value='/goal_pose',
            description='PoseStamped goal topic to record when using RViz/Web simple goal.',
        ),
        DeclareLaunchArgument('scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('ekf_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('publish_robot_model', default_value='true'),
        DeclareLaunchArgument('start_ekf', default_value='true'),
        DeclareLaunchArgument('orb_pose_topic', default_value='/orbslam3/pose_base'),
        DeclareLaunchArgument(
            'orb_tracking_state_topic', default_value='/orbslam3/tracking_state'),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='GTSAM is the only map->odom TF publisher in graph-primary mode.',
        ),
        DeclareLaunchArgument('rf2o_odom_topic', default_value='/rf2o/odom'),
        DeclareLaunchArgument('rf2o_scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('rf2o_frequency', default_value='20.0'),
        DeclareLaunchArgument('start_scan_to_scan', default_value='false'),
        DeclareLaunchArgument('graph_scan_odom_topic', default_value='/rf2o/odom'),
        DeclareLaunchArgument(
            'enforce_scan_map_bounds',
            default_value='true',
            description='Reject scan-to-map poses outside/unknown/occupied on the active 2D map YAML/PGM.',
        ),
        DeclareLaunchArgument(
            'scan_map_bounds_margin',
            default_value='0.30',
            description='Meters allowed outside map bounds before rejecting scan-to-map pose.',
        ),
        DeclareLaunchArgument(
            'scan_map_occupancy_gate_radius',
            default_value='0.20',
            description='Radius around scan-to-map pose that must remain free on the occupancy map.',
        ),
        DeclareLaunchArgument(
            'turn_rate_topic',
            default_value='/wheel_vel',
            description='Odometry topic used only for turn gating; EKF yaw remains IMU-driven.',
        ),
        DeclareLaunchArgument('turn_rate_timeout_sec', default_value='0.50'),
        DeclareLaunchArgument('use_initial_pose', default_value='false'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'wait_for_global_pose_before_init',
            default_value='true',
            description='Wait for slam_toolbox/scan-to-map pose before creating the first graph prior.',
        ),
        DeclareLaunchArgument(
            'seed_from_global_pose',
            default_value='true',
            description='Use the first scan-to-map pose as the graph initial map pose when no manual initial pose is provided.',
        ),
        DeclareLaunchArgument('max_association_dt', default_value='0.35'),
        DeclareLaunchArgument('min_node_translation', default_value='0.06'),
        DeclareLaunchArgument('min_node_rotation', default_value='0.05'),
        DeclareLaunchArgument('max_node_dt', default_value='0.35'),
        DeclareLaunchArgument('create_node_on_time_gap', default_value='false'),
        DeclareLaunchArgument('use_turn_rate_yaw_when_ekf_stalled', default_value='true'),
        DeclareLaunchArgument('turn_rate_yaw_min_wz', default_value='0.25'),
        DeclareLaunchArgument('turn_rate_yaw_max_pose_delta', default_value='0.01'),
        DeclareLaunchArgument('ekf_sigma_x', default_value='0.07'),
        DeclareLaunchArgument('ekf_sigma_y', default_value='0.07'),
        DeclareLaunchArgument('ekf_sigma_yaw', default_value='0.12'),
        DeclareLaunchArgument('orb_sigma_x', default_value='10.0'),
        DeclareLaunchArgument('orb_sigma_y', default_value='10.0'),
        DeclareLaunchArgument('orb_sigma_yaw', default_value='0.20'),
        DeclareLaunchArgument('scan_odom_sigma_x', default_value='0.70'),
        DeclareLaunchArgument('scan_odom_sigma_y', default_value='0.70'),
        DeclareLaunchArgument('scan_odom_sigma_yaw', default_value='0.40'),
        DeclareLaunchArgument('max_scan_ekf_disagreement_xy', default_value='0.35'),
        DeclareLaunchArgument('max_scan_ekf_disagreement_yaw', default_value='0.30'),
        DeclareLaunchArgument('scan_odom_max_yaw_rate', default_value='1.80'),
        DeclareLaunchArgument('reject_scan_odom_when_not_forward', default_value='true'),
        DeclareLaunchArgument('scan_odom_min_forward_vx', default_value='0.03'),
        DeclareLaunchArgument('scan_map_sigma_x', default_value='0.60'),
        DeclareLaunchArgument('scan_map_sigma_y', default_value='0.60'),
        DeclareLaunchArgument('scan_map_sigma_yaw', default_value='0.80'),
        DeclareLaunchArgument('scan_map_untrusted_yaw_sigma', default_value='10.0'),
        DeclareLaunchArgument('trust_scan_map_yaw', default_value='true'),
        DeclareLaunchArgument('enable_home_scan_map_correction', default_value='true'),
        DeclareLaunchArgument('scan_map_only_near_home', default_value='true'),
        DeclareLaunchArgument('home_map_x', default_value='0.0'),
        DeclareLaunchArgument('home_map_y', default_value='0.0'),
        DeclareLaunchArgument('home_map_yaw', default_value='0.0'),
        DeclareLaunchArgument('home_scan_map_radius', default_value='0.75'),
        DeclareLaunchArgument('home_scan_map_sigma_xy', default_value='0.25'),
        DeclareLaunchArgument('home_scan_map_max_innovation_xy', default_value='0.35'),
        DeclareLaunchArgument('robust_huber_k', default_value='1.345'),
        DeclareLaunchArgument('reject_scan_map_when_turning', default_value='true'),
        DeclareLaunchArgument('reject_scan_map_when_not_forward', default_value='true'),
        DeclareLaunchArgument('scan_map_min_forward_vx', default_value='0.015'),
        DeclareLaunchArgument('scan_map_turn_reject_wz', default_value='0.25'),
        DeclareLaunchArgument('scan_map_turn_block_sec', default_value='1.5'),
        DeclareLaunchArgument(
            'scan_map_selective_updates',
            default_value='true',
            description='Add scan-to-map factors only after repeated stable corrections.',
        ),
        DeclareLaunchArgument(
            'scan_map_update_min_interval_sec',
            default_value='2.0',
            description='Minimum time between accepted scan-to-map graph corrections.',
        ),
        DeclareLaunchArgument('scan_map_update_stable_count', default_value='5'),
        DeclareLaunchArgument(
            'scan_map_update_max_innovation_spread_xy',
            default_value='0.12',
            description='Maximum XY spread among candidate scan-to-map innovations.',
        ),
        DeclareLaunchArgument(
            'scan_map_update_max_innovation_spread_yaw',
            default_value='0.35',
            description='Maximum yaw spread among candidate scan-to-map innovations.',
        ),
        DeclareLaunchArgument('scan_map_update_min_duration_sec', default_value='0.35'),
        DeclareLaunchArgument('enable_scan_map_candidate_gate', default_value='true'),
        DeclareLaunchArgument('scan_map_candidate_required_count', default_value='5'),
        DeclareLaunchArgument('scan_map_candidate_window_sec', default_value='1.0'),
        DeclareLaunchArgument('scan_map_candidate_max_spread_xy', default_value='0.15'),
        DeclareLaunchArgument('scan_map_candidate_max_spread_yaw', default_value='0.15'),
        DeclareLaunchArgument('scan_map_max_single_correction_xy', default_value='0.30'),
        DeclareLaunchArgument('scan_map_max_single_correction_yaw', default_value='0.25'),
        DeclareLaunchArgument('scan_map_reject_correction_xy', default_value='0.80'),
        DeclareLaunchArgument('scan_map_reject_correction_yaw', default_value='0.80'),
        DeclareLaunchArgument('scan_map_use_xy_only_when_yaw_bad', default_value='true'),
        DeclareLaunchArgument('scan_map_xy_only_yaw_reject_threshold', default_value='0.35'),
        DeclareLaunchArgument('scan_map_suspect_reject_count_threshold', default_value='10'),
        DeclareLaunchArgument('scan_map_suspect_cooldown_sec', default_value='3.0'),
        DeclareLaunchArgument('scan_map_relocalize_cooldown_sec', default_value='5.0'),
        DeclareLaunchArgument('enable_scan_map_auto_relocalization', default_value='false'),
        DeclareLaunchArgument('reject_scan_map_backward_when_forward', default_value='true'),
        DeclareLaunchArgument('scan_map_backward_reject_min_vx', default_value='0.03'),
        DeclareLaunchArgument('scan_map_backward_reject_distance', default_value='0.10'),
        DeclareLaunchArgument('enable_scan_map_bounded_correction', default_value='true'),
        DeclareLaunchArgument(
            'max_amcl_jump_xy',
            default_value='3.00',
            description='Reject one-step scan-to-map jumps larger than this many meters.',
        ),
        DeclareLaunchArgument(
            'max_amcl_jump_yaw',
            default_value='3.20',
            description='Reject one-step scan-to-map yaw jumps larger than this many radians.',
        ),
        DeclareLaunchArgument(
            'max_global_innovation_xy',
            default_value='0.60',
            description='Reject large scan-to-map translation innovations before robust weighting.',
        ),
        DeclareLaunchArgument(
            'max_global_innovation_yaw',
            default_value='3.20',
            description='Reject scan-to-map yaw corrections only while scan-to-map yaw is trusted.',
        ),
        robot_state_publisher,
        ekf_launch,
        map_server,
        map_lifecycle,
        TimerAction(period=1.0, actions=[rf2o_scan]),
        TimerAction(period=1.2, actions=[global_scan_initializer]),
        TimerAction(period=2.0, actions=[scan_to_map_matcher]),
        TimerAction(period=1.5, actions=[slam_toolbox_reference]),
        TimerAction(period=1.8, actions=[slam_toolbox_posegraph_loader]),
        TimerAction(period=2.0, actions=[slam_toolbox_pose_bridge]),
        TimerAction(period=3.0, actions=[graph]),
        TimerAction(period=3.0, actions=[amcl_reference, amcl_reference_lifecycle]),
        TimerAction(period=6.0, actions=[amcl_reference_home_seed]),
        TimerAction(period=LaunchConfiguration('nav2_start_delay'), actions=[nav2]),
        TimerAction(
            period=LaunchConfiguration('nav2_lifecycle_recovery_delay'),
            actions=[nav2_lifecycle_recovery],
        ),
    ])
