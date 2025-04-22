import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launches the RC car actuation interface node, using a params.yaml
    found under config_path/tod_rc_car_interface/params.yaml.
    """
    config_path = LaunchConfiguration('config_path')
    vehicle_id = LaunchConfiguration('vehicleID')

    param_file = PathJoinSubstitution([
        config_path,
        'package_config',
        'tod_rc_car_interface',
        'params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='tod_rc-car_interface',
            executable='ActuationInterfaceNode',
            name='ActuationInterfaceNode',
            namespace='/vehicle/interface/actuation/',
            output='screen',
            parameters=[param_file]
        )
    ])
