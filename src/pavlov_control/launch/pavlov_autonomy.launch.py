from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_mode = LaunchConfiguration("initial_mode")
    use_external_odom = LaunchConfiguration("use_external_odom")

    gait_controller = Node(
        package="pavlov_control",
        executable="gait_controller.py",
        name="gait_controller",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    cmd_vel_mux = Node(
        package="pavlov_control",
        executable="cmd_vel_mux.py",
        name="cmd_vel_mux",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"initial_mode": initial_mode},
            {"teleop_topic": "/cmd_vel_teleop"},
            {"auto_topic": "/cmd_vel_auto"},
            {"output_topic": "/cmd_vel"},
            {"mode_topic": "/control_mode"},
        ],
    )

    go_to_goal = Node(
        package="pavlov_control",
        executable="go_to_goal.py",
        name="go_to_goal",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"use_external_odom": use_external_odom},
            {"goal_topic": "/goal_pose"},
            {"cmd_vel_topic": "/cmd_vel_auto"},
            {"control_mode_topic": "/control_mode"},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time",
            ),
            DeclareLaunchArgument(
                "initial_mode",
                default_value="teleop",
                description="cmd_vel_mux mode: teleop or auto",
            ),
            DeclareLaunchArgument(
                "use_external_odom",
                default_value="false",
                description="true: use /odom, false: internal integration",
            ),
            gait_controller,
            cmd_vel_mux,
            go_to_goal,
        ]
    )
