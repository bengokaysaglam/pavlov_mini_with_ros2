import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('pavlov_description')
    pkg_gazebo = get_package_share_directory('pavlov_gazebo')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    
    urdf_file = os.path.join(pkg_description, 'urdf', 'pavlov_mini.urdf.xacro')
    world_file = os.path.join(pkg_gazebo, 'worlds', 'empty.world')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_description_content = Command(['xacro', ' ', urdf_file])
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}]
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'pavlov_mini', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.3'],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])