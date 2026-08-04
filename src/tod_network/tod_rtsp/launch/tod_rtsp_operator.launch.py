from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Resolve default paths to the local config folder
    package_share_directory = get_package_share_directory('tod_rtsp')  # Replace with your package name
    default_router_config_path = os.path.join(package_share_directory, 'config')

    # Declare launch file parameters
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_router_config_path,
        description="Path to the router configuration settings"
    )
    vehicle_id_arg = DeclareLaunchArgument(
        "vehicleID",
        default_value="edgar",
        description="vehicle id to load the correct camera params (by looking into the folder <vehicle_id> in the config folder)."
    )
    params_file = PathJoinSubstitution([
        LaunchConfiguration('config_path'),
        'package_config',
        'tod_rtsp',
        'params.yaml'
    ])
    default_rtsp_config_path = PathJoinSubstitution([
        LaunchConfiguration("config_path"),
        "package_config",
        "tod_rtsp",
    ])
    rtsp_config_path = EnvironmentVariable(
        "TOD_RTSP_CONFIG_PATH",
        default_value=default_rtsp_config_path,
    )

    # Node definition using launch parameters
    operator_rtsp_clients_node = Node(
        package='tod_rtsp',  # Replace with your package name
        executable='OperatorRtspClients',  # Match the executable name
        name='OperatorRtspClients',
        namespace="/operator/network/video",
        output='screen',
        parameters=[
            params_file,
            {"camera_params_path": LaunchConfiguration("config_path")},
            {"router_config_path": rtsp_config_path},
            {"vehicleID": LaunchConfiguration("vehicleID")},
        ]
    )

    return LaunchDescription([
        config_path_arg,
        vehicle_id_arg,
        operator_rtsp_clients_node,
    ])
