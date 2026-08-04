import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    default_config_path = os.path.join(
        get_package_share_directory("tod_launch"), "config"
    )
    config_path = LaunchConfiguration("config_path")
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_path,
        description="Path to the shared TOD configuration directory",
    )
    params = PathJoinSubstitution(
        [config_path, "package_config", "tod_peanut01_interface", "params.yaml"]
    )

    return LaunchDescription(
        [
            config_path_arg,
            Node(
                package="tod_peanut01_interface",
                executable="Peanut01DryRunInterface",
                namespace="/vehicle/interface/peanut01",
                name="DryRunInterface",
                output="screen",
                parameters=[params],
            ),
        ]
    )
