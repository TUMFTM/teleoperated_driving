# @file tod_safety_gate.launch.py
# @brief launch file for the safety gate
# @copyright 2025 TUM-FTM

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    package_name = 'tod_safety_gate'
    namespace = '/vehicle/safety'
    
    default_config_path = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'config'
    ])
    
    config_path = LaunchConfiguration('config_path')
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Path to the config folder.'
    )

    node_param_path = PathJoinSubstitution([
        LaunchConfiguration('config_path'),
        'package_config',
        package_name,
        'params.yaml'
    ])
            
    safety_gate_node = Node(
        package=package_name,
        namespace=namespace,
        executable='safety_gate',
        name='safety_gate',
        parameters=[node_param_path]
    )
    
    return LaunchDescription([
        config_path_arg,
        safety_gate_node
    ])