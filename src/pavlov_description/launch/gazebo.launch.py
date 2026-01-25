import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pavlov_description = get_package_share_directory("pavlov_description")
    
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(pavlov_description, "urdf", "pavlov_mini.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )
    
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(pavlov_description).parent.resolve())]
    )
    
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"
        ]),
        launch_arguments=[("gz_args", [" -v 4", " -r", " empty.sdf"])]
    )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "pavlov_mini_ros2",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.35"
        ],
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        output="screen"
    )
    
    # Controller configuration file
    controller_config = PathJoinSubstitution(
        [FindPackageShare("pavlov_control"), "config", "ros2_controllers.yaml"]
    )
    
    # Controller spawners with parameters
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", controller_config
        ],
        output="screen"
    )
    
    load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controller_config
        ],
        output="screen"
    )
    
    # Delay controller spawning
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[
            load_joint_state_broadcaster,
            load_joint_trajectory_controller
        ]
    )
    
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        delayed_controllers
    ])