import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    default_config_path = os.path.join(
        get_package_share_directory('tod_trajectory_guidance'),
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
        'tod_trajectory_guidance',
        'params.yaml'
    ])

    vehicle_id = LaunchConfiguration('vehicleID')
    vehicle_id_arg = DeclareLaunchArgument('vehicleID', default_value = 'edgar')
    
    namespace_operator = '/operator/trajectory_guidance/'

    path_creator_node = Node(
                package='tod_trajectory_guidance',
                executable='OperatorPathCreator',
                namespace=namespace_operator,
                name='PathCreator',
                output='screen',
                parameters=[
                    node_param_path,
                    {'vehicleID': vehicle_id},
                    {'config_path': config_path},
                ],
                remappings=[ ]
            )
 
    return LaunchDescription([
        config_path_arg,
        vehicle_id_arg,
        path_creator_node
    ])