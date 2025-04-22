from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    forward_primary_cmd_node = Node(
        package     = 'tod_command_forwarder',
        namespace   = '/vehicle/direct_control',
        executable  = 'ForwardPrimaryCtrlCmd',
        name        = 'ForwardPrimaryCtrlCmd',
        output      = 'screen'
    )

    return LaunchDescription([
        forward_primary_cmd_node
    ])