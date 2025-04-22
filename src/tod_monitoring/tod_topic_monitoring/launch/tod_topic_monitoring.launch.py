from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    package_name = 'tod_topic_monitoring'
    namespace = '/vehicle/monitoring'
    
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
        config_path,
        'package_config',
        package_name,
        'params.yaml'
    ])
        
    topic_monitoring_node = Node(
        package=package_name,
        namespace=namespace,
        executable='topic_monitor',
        name='topic_monitor',
        parameters=[node_param_path]
    )
        
    return LaunchDescription([
        config_path_arg,
        topic_monitoring_node
    ])