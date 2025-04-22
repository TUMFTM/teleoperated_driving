from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the sensing_interface_node with a parameter file found under config_path.
    """
    config_path = LaunchConfiguration('config_path')
    
    param_file = PathJoinSubstitution([
        config_path,
        'package_config',
        'tod_edgar_interface',
        'params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='tod_edgar_interface',
            executable='SensingInterfaceNode',
            name='SensingInterfaceNode',
            namespace='/vehicle/interface/sensing',
            output='screen',
            parameters=[param_file]
        )
    ])
