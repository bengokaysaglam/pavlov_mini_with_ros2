import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pavlov_description = get_package_share_directory("pavlov_description")
    default_world = os.path.join(pavlov_description, "worlds", "empty_with_sensors.sdf")
    
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(pavlov_description, "urdf", "pavlov_mini.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    camera_sensor_pitch_arg = DeclareLaunchArgument(
        name="camera_sensor_pitch",
        default_value="-0.15",
        description="Camera tilt (radians). Negative looks down.",
    )

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=default_world,
        description="Absolute path to Gazebo world SDF (must include Sensors system for cameras)",
    )

    spawn_red_ball_arg = DeclareLaunchArgument(
        name="spawn_red_ball",
        default_value="true",
        description="Spawn a static red sphere model for camera testing",
    )

    red_ball_x_arg = DeclareLaunchArgument(name="red_ball_x", default_value="1.5")
    red_ball_y_arg = DeclareLaunchArgument(name="red_ball_y", default_value="0.0")
    red_ball_z_arg = DeclareLaunchArgument(name="red_ball_z", default_value="0.05")

    enable_camera_bridge_arg = DeclareLaunchArgument(
        name="enable_camera_bridge",
        default_value="true",
        description="Bridge Gazebo camera topics to ROS2 sensor_msgs",
    )

    camera_gz_image_topic_arg = DeclareLaunchArgument(
        name="camera_gz_image_topic",
        default_value="/world/empty/model/pavlov_mini_ros2/link/base_link/sensor/camera_sensor/image",
        description="Gazebo Transport camera image topic (see: `gz topic -l`)",
    )

    camera_gz_info_topic_arg = DeclareLaunchArgument(
        name="camera_gz_info_topic",
        default_value="/world/empty/model/pavlov_mini_ros2/link/base_link/sensor/camera_sensor/camera_info",
        description="Gazebo Transport camera info topic (see: `gz topic -l`)",
    )

    camera_ros_image_topic_arg = DeclareLaunchArgument(
        name="camera_ros_image_topic",
        default_value="/camera/image_raw",
        description="ROS2 Image output topic",
    )

    camera_ros_info_topic_arg = DeclareLaunchArgument(
        name="camera_ros_info_topic",
        default_value="/camera/camera_info",
        description="ROS2 CameraInfo output topic",
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
            is_ignition,
            " camera_sensor_pitch:=",
            LaunchConfiguration("camera_sensor_pitch"),
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
        launch_arguments=[("gz_args", [" -v 4", " -r ", LaunchConfiguration("world")])]
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

    red_ball_sdf = PathJoinSubstitution(
        [FindPackageShare("pavlov_description"), "models", "red_ball", "model.sdf"]
    )
    
    gz_spawn_red_ball = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "empty",
            "-file",
            red_ball_sdf,
            "-name",
            "red_ball",
            "-allow_renaming",
            "true",
            "-x",
            LaunchConfiguration("red_ball_x"),
            "-y",
            LaunchConfiguration("red_ball_y"),
            "-z",
            LaunchConfiguration("red_ball_z"),
        ],
        condition=IfCondition(LaunchConfiguration("spawn_red_ball")),
    )
    delayed_red_ball = TimerAction(period=2.0, actions=[gz_spawn_red_ball])
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        output="screen"
    )

    enable_camera_bridge = LaunchConfiguration("enable_camera_bridge")
    camera_gz_image_topic = LaunchConfiguration("camera_gz_image_topic")
    camera_gz_info_topic = LaunchConfiguration("camera_gz_info_topic")
    camera_ros_image_topic = LaunchConfiguration("camera_ros_image_topic")
    camera_ros_info_topic = LaunchConfiguration("camera_ros_info_topic")

    camera_bridge_info = LogInfo(
        condition=IfCondition(enable_camera_bridge),
        msg=[
            "Camera bridge: ",
            camera_gz_image_topic,
            " -> ",
            camera_ros_image_topic,
            " | ",
            camera_gz_info_topic,
            " -> ",
            camera_ros_info_topic,
        ],
    )

    gz_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Use Gazebo->ROS2 direction for sensor streams.
            PythonExpression(
                ["'", camera_gz_image_topic, "@sensor_msgs/msg/Image[gz.msgs.Image'"]
            ),
            PythonExpression(
                ["'", camera_gz_info_topic, "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'"]
            ),
            "--ros-args",
            "-r",
            PythonExpression(
                ["'", camera_gz_image_topic, ":=", camera_ros_image_topic, "'"]
            ),
            "-r",
            PythonExpression(
                ["'", camera_gz_info_topic, ":=", camera_ros_info_topic, "'"]
            ),
        ],
        output="screen",
        condition=IfCondition(enable_camera_bridge),
    )
    
    controller_config = PathJoinSubstitution(
        [FindPackageShare("pavlov_control"), "config", "ros2_controllers.yaml"]
    )
    
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
    
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[
            load_joint_state_broadcaster,
            load_joint_trajectory_controller
        ]
    )
    
    return LaunchDescription([
        model_arg,
        camera_sensor_pitch_arg,
        world_arg,
        spawn_red_ball_arg,
        red_ball_x_arg,
        red_ball_y_arg,
        red_ball_z_arg,
        enable_camera_bridge_arg,
        camera_gz_image_topic_arg,
        camera_gz_info_topic_arg,
        camera_ros_image_topic_arg,
        camera_ros_info_topic_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        delayed_red_ball,
        gz_ros2_bridge,
        camera_bridge_info,
        gz_camera_bridge,
        delayed_controllers
    ])
