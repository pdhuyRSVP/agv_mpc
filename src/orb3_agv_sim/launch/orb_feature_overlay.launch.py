from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    max_features = LaunchConfiguration("max_features")
    fast_threshold = LaunchConfiguration("fast_threshold")
    publish_hz = LaunchConfiguration("publish_hz")
    resize_scale = LaunchConfiguration("resize_scale")
    draw_rich_keypoints = LaunchConfiguration("draw_rich_keypoints")

    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("output_topic", default_value="/orb_features/image"),
        DeclareLaunchArgument("max_features", default_value="900"),
        DeclareLaunchArgument("fast_threshold", default_value="10"),
        DeclareLaunchArgument("publish_hz", default_value="5.0"),
        DeclareLaunchArgument("resize_scale", default_value="1.0"),
        DeclareLaunchArgument("draw_rich_keypoints", default_value="false"),
        Node(
            package="orb3_agv_sim",
            executable="orb_feature_overlay.py",
            name="orb_feature_overlay",
            output="screen",
            parameters=[{
                "input_topic": input_topic,
                "output_topic": output_topic,
                "max_features": max_features,
                "fast_threshold": fast_threshold,
                "publish_hz": publish_hz,
                "resize_scale": resize_scale,
                "draw_rich_keypoints": draw_rich_keypoints,
            }],
        ),
    ])
