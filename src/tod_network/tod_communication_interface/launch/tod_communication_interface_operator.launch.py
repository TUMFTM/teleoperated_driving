import os
import yaml
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description() -> LaunchDescription:

    # Default config path
    default_config_path = os.path.join(
        get_package_share_directory('tod_communication_interface'),
        'config', 
    )

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Path to the config folder.'
    )

    def launch_parsed_services(context: LaunchContext, *args, **kwargs):
        nodes=[]
        config_path_str = context.perform_substitution(LaunchConfiguration('config_path'))

        # Retrieve the entire path of the config file
        services_file_path = os.path.join(
            config_path_str,
            'package_config',
            'tod_communication_interface',
            'services.yaml'
        )

        with open(services_file_path, 'r') as f:
            parsed_services = yaml.safe_load(f)

        # Launch parsed services
        for key, value in parsed_services['services'].items():
            if key == "NetworkMonitorService":
                nodes.append(Node(
                    package = 'tod_communication_interface',
                    executable = 'NetworkMonitorServiceForwarder',
                    name = 'NetworkMonitorServiceForwarder',
                    namespace="/operator/network/config",
                    arguments= [value['service_operator'], value['protocol'], str(value['forwarder_port']), str(value['listener_port'])],
                    output = 'screen'
                ))
            elif key == "PacketCaptureService":
                print("PacketCaptureService")
                nodes.append(Node(
                    package = 'tod_communication_interface',
                    executable = 'PacketCaptureServiceForwarder',
                    name = 'PacketCaptureServiceForwarder',
                    namespace="/operator/network/config",
                    arguments= [value['service_operator'], value['protocol'], str(value['forwarder_port']), str(value['listener_port'])],
                    output = 'screen'
                ))
            elif key == "VideoParamService":
                print("VideoParamService")
                nodes.append(Node(
                    package = 'tod_communication_interface',
                    executable = 'VideoParamServiceForwarder',
                    name = 'VideoParamServiceForwarder',
                    namespace="/operator/network/config",
                    arguments= [value['service_operator'], value['protocol'], str(value['forwarder_port']), str(value['listener_port'])],
                    output = 'screen'
                ))
            elif key == "VideoConfigService":
                print("VideoConfigService")
                nodes.append(Node(
                    package = 'tod_communication_interface',
                    executable = 'VideoConfigServiceForwarder',
                    name = 'VideoConfigServiceForwarder',
                    namespace="/operator/network/config",
                    arguments= [value['service_operator'], value['protocol'], str(value['forwarder_port']), str(value['listener_port'])],
                    output = 'screen'
                ))
            else:
                nodes.append(LogInfo(f"Unrecognized service: {key}"))

        return nodes
        
    return LaunchDescription([config_path_arg, OpaqueFunction(function=launch_parsed_services)])
