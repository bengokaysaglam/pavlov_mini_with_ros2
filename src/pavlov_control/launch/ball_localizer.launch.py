from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("output_frame", default_value="base_link"),
            DeclareLaunchArgument("camera_pitch_rad", default_value="-0.15"),
            DeclareLaunchArgument("ball_radius_m", default_value="0.03"),
            DeclareLaunchArgument("min_radius_px", default_value="3.0"),
            DeclareLaunchArgument("max_range_m", default_value="6.0"),
            DeclareLaunchArgument("kernel_size", default_value="5"),
            DeclareLaunchArgument("sat_min", default_value="120"),
            DeclareLaunchArgument("val_min", default_value="80"),
            DeclareLaunchArgument("publish_debug_image", default_value="true"),

            Node(
                package="pavlov_control",
                executable="ball_localizer.py",
                name="ball_localizer",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"image_topic": LaunchConfiguration("image_topic")},
                    {"camera_info_topic": LaunchConfiguration("camera_info_topic")},
                    {"output_frame": LaunchConfiguration("output_frame")},
                    {"camera_pitch_rad": LaunchConfiguration("camera_pitch_rad")},
                    {"ball_radius_m": LaunchConfiguration("ball_radius_m")},
                    {"min_radius_px": LaunchConfiguration("min_radius_px")},
                    {"max_range_m": LaunchConfiguration("max_range_m")},
                    {"kernel_size": LaunchConfiguration("kernel_size")},
                    {"sat_min": LaunchConfiguration("sat_min")},
                    {"val_min": LaunchConfiguration("val_min")},
                    {"publish_debug_image": LaunchConfiguration("publish_debug_image")},
                ],
            ),
        ]
    )
