import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launches the RC car sensing interface node, using a params.yaml
    found under config_path/tod_rc_car_interface/params.yaml.
    """
    # Retrieve 'config_path' from the top-level
    config_path = LaunchConfiguration('config_path')

    param_file = PathJoinSubstitution([
        config_path,
        'package_config',
        'tod_rc_car_interface',
        'params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='tod_rc-car_interface',
            executable='SensingInterfaceNode',
            name='SensingInterfaceNode',
            namespace='/vehicle/interface/sensing/',
            output='screen',
            parameters=[param_file]
        )
    ])
