import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import LogInfo
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
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
        'tod_logger',
        'params.yaml'
    ])

    # Define the group with namespace
    logging_group = Node(
        namespace='/operator/logging',
        package='tod_logger',
        executable='TopicLogger',
        name='topic_logger',
        output='screen',
        parameters=[
            param_file_path
        ]
    )

    return LaunchDescription([
        config_path_arg,
        logging_group
    ])