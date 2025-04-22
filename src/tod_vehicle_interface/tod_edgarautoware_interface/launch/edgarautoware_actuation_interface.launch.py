from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the actuation_interface_node with a parameter file found under config_path.
    """
    config_path = LaunchConfiguration('config_path')

    param_file = PathJoinSubstitution([
        config_path,
        'package_config',
        'tod_edgarautoware_interface',
        'params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='tod_edgarautoware_interface',
            executable='ActuationInterfaceNode',
            name='ActuationInterfaceNode',
            namespace='/vehicle/interface/actuation/',
            output='screen',
            parameters=[param_file] 
        )
    ])