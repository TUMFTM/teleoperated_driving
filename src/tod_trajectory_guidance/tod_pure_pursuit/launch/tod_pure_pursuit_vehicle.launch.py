import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    default_config_path = os.path.join(
        get_package_share_directory('tod_pure_pursuit'),
        'config'
    )
    config_path = LaunchConfiguration('config_path')
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path
    )

    ### For node params ###
    node_param_path = PathJoinSubstitution([
        LaunchConfiguration('config_path'),
        'package_config',
        'tod_pure_pursuit',
        'params.yaml'
    ])

    vehicle_id_arg = DeclareLaunchArgument('vehicleID', default_value = 'edgar')
    vehicle_id = LaunchConfiguration('vehicleID')
    namespace_vehicle = '/vehicle/trajectory_guidance/control'

    pure_pursuit_node = Node(
        package='tod_pure_pursuit',
        executable='tod_pure_pursuit_node',
        namespace=namespace_vehicle,
        name='PurePursuit',
        output='screen',
        parameters=[
            node_param_path,
            {'vehicleID': vehicle_id},
            {'config_path': config_path},
        ],
        remappings=[ ]
    )

    pure_pursuit_simulation_node = Node(
        package='tod_pure_pursuit',
        executable='tod_pure_pursuit_simulator_node',
        name='PurePursuitSimulator',
        namespace=namespace_vehicle,
        output='screen',
        parameters=[
            node_param_path,
            {'vehicleID': vehicle_id},
            {'config_path': config_path},
        ],
        remappings=[ ]
    )

    return LaunchDescription([
        vehicle_id_arg,
        config_path_arg,
        pure_pursuit_node,
        pure_pursuit_simulation_node
    ])