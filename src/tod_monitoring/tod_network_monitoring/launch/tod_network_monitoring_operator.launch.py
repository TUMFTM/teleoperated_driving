from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    package_name = 'tod_network_monitoring'
    namespace = '/operator/monitoring'
    
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

    network_interface = LaunchConfiguration('operatorNetworkInterface')
    network_interface_arg = DeclareLaunchArgument(
        'operatorNetworkInterface',
        default_value='eth0',
        description='Operator network interface to monitor.'
    )

    node_param_path = PathJoinSubstitution([
        config_path,
        'package_config',
        package_name,
        'params.yaml'
    ])
        
    network_tester_operator_node = Node(
        package=package_name,
        namespace=namespace,
        executable='network_tester',
        name='network_tester',
        parameters=[node_param_path]
    )
    
    network_monitoring_operator_node = Node(
        package=package_name,
        namespace=namespace,
        executable='network_monitor',
        name='network_monitor',
        parameters=[node_param_path, {'network_interface': network_interface}]
    )
    
    packet_logger_operator_node = Node(
        package=package_name,
        namespace=namespace,
        executable='packet_logger',
        name='packet_logger',
        parameters=[node_param_path, {'network_interface': network_interface}]
    )
    
    return LaunchDescription([
        config_path_arg,
        network_interface_arg,
        network_tester_operator_node,
        network_monitoring_operator_node,
        packet_logger_operator_node
    ])
