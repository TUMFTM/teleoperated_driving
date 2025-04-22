import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
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
        'tod_lidar',
        'params.yaml'
    ])

    vehicle_id =  LaunchConfiguration('vehicleID')
    vehicle_id_arg = DeclareLaunchArgument(
        'vehicleID',
        default_value='edgar',
        description='ID of the currently used vehicle.'
    )

    namespace_operator = '/operator/network/lidar'

    lidar_operator_node = Node(
        package='tod_lidar',
        executable='PointCloudDecoderNode',
        name='PointCloudDecoder',
        namespace=namespace_operator,
        output='screen',
        parameters=[
            param_file_path,
            {'vehicleID': vehicle_id},
            {'config_path': LaunchConfiguration('config_path')},
        ],
        remappings=[ ]
    )

    return LaunchDescription([
        vehicle_id_arg,
        config_path_arg,
        lidar_operator_node
    ])