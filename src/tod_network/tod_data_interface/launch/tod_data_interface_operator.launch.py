import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription,LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    
    # Launch arguments
    default_config_path = os.path.join(
        get_package_share_directory('tod_data_interface'),
        'config', 
    )
    
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Path to the config folder.'
    )

    def launch_parsed_nodes(context: LaunchContext, *args, **kwargs):
        nodes = []
        config_path_str = context.perform_substitution(LaunchConfiguration('config_path'))
        
        # Retrieve the entire path of the config file
        config_file = os.path.join(
            config_path_str,
            'package_config',
            'tod_data_interface',
            'operator_config.yaml'
        )

        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)

        nodes = []
        for receiver in config['receivers']:
            nodes.append(Node(
                package='tod_data_interface',
                executable='generic_tod_receiver',
                name=receiver['name'],
                namespace="/operator/network/data",
                parameters=[{
                    'protocol_strategy': receiver['protocol_strategy'],
                    'topic': receiver['topic'],
                    'topic_type': receiver['topic_type'],
                    'port': receiver['port'],
                    'status_topic': config['status_topic'],
                }]
            ))
        for sender in config['senders']:
            send_always = False
            if "send_always" in sender:
                send_always = sender["send_always"]
            nodes.append(Node(
                package='tod_data_interface',
                executable='generic_tod_sender',
                name=sender['name'],
                namespace="/operator/network/data",
                parameters=[{
                    'protocol_strategy': sender['protocol_strategy'],
                    'topic': sender['topic'],
                    'topic_type': sender['topic_type'],
                    'ip': sender['ip'],
                    'port': sender['port'],
                    'sending_control_modes': sender['sending_control_modes'],
                    'status_topic': config['status_topic'],
                    'send_always': send_always,
                    'in_vehicle' : False
                }]
            ))
        return nodes

    return LaunchDescription([config_path_arg, OpaqueFunction(function=launch_parsed_nodes)])