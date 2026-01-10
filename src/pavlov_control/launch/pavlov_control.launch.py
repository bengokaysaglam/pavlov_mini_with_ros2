from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    pkg_share = get_package_share_directory('pavlov_control')
    config_file = os.path.join(pkg_share, 'config', 'controller_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    controller_node = Node(
        package='pavlov_control',
        executable='pavlov_controller_node',
        name='pavlov_controller',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        controller_node
    ])