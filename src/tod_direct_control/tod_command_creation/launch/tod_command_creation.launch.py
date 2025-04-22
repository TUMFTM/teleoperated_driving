import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import LogInfo


def generate_launch_description():
    vehicle_id_arg = DeclareLaunchArgument('vehicleID', default_value = 'edgar')
    
    ### For vehicle params ###
    default_config_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..',
        'config'
    )
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path
    )

    ### For node params ###
    param_file_path = PathJoinSubstitution([
        LaunchConfiguration('config_path'),
        'package_config',
        'tod_command_creation',
        'params.yaml'
    ])

    ### Set up the node ### 
    command_creation_node = Node(
        namespace   = '/operator/direct_control',
        package     = 'tod_command_creation',
        executable  = 'OperatorCommandCreator',
        name        = 'CommandCreator',
        output      = 'screen',
        parameters  = [
            {'vehicleID': LaunchConfiguration('vehicleID')},
            {'config_path': LaunchConfiguration('config_path')},
            param_file_path
        ]
    )

    return LaunchDescription([
        vehicle_id_arg, 
        config_path_arg,
        command_creation_node
    ])