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

    ns = '/operator/transform'

    ### Set up the nodes ### 
    dynamic_transform_node = Node(
        namespace   = ns,
        package     = 'tod_transform',
        executable  = 'DynamicTransformPublisher',
        name        = 'DynamicTransformPublisher',
        output      = 'screen',
        parameters  = [
            {'vehicleID': LaunchConfiguration('vehicleID')}
        ],
        remappings = [ ]
    )
    
    static_transform_node = Node(
        namespace   = ns,
        package     = 'tod_transform',
        executable  = 'StaticTransformPublisher',
        name        = 'StaticTransformPublisher',
        parameters  = [
            {'vehicleID': LaunchConfiguration('vehicleID')},
            {'config_path': LaunchConfiguration('config_path')}
        ]        
    )
    
    return LaunchDescription([
        vehicle_id_arg, 
        config_path_arg,
        dynamic_transform_node,
        static_transform_node
    ])