from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Resolve default paths to the local config folder
    package_share_directory = get_package_share_directory('tod_rtsp')  # Replace with your package name
    default_config_folder_path = os.path.join(package_share_directory, 'config')

    # Declare launch file parameters
    config_folder_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_folder_path,
        description="Path to the config folder for config files"
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
    sensor_domain_id = EnvironmentVariable(
        "TOD_RTSP_SENSOR_DOMAIN_ID",
        default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="0"),
    )
    
    # Node definition using launch parameters
    vehicle_rtsp_server_node = Node(
        package='tod_rtsp',  # Replace with your package name
        executable='VehicleRtspServer',  # Match the executable name
        name='VehicleRtspServer',
        namespace="/vehicle/network/video",
        output='screen',
        parameters=[
            params_file,
            {"camera_params_path": LaunchConfiguration("config_path")},
            {"router_settings_path": rtsp_config_path},
            {"vehicleID": LaunchConfiguration("vehicleID")},
            {"stream_settings_path": rtsp_config_path},
        ],
        additional_env={"ROS_DOMAIN_ID": sensor_domain_id},
    )

    return LaunchDescription([
        config_folder_arg,
        vehicle_id_arg,
        vehicle_rtsp_server_node,
    ])
