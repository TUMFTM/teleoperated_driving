from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    namespace_operator = '/operator/statemachine'
    operator_group = GroupAction(
        actions=[
            PushRosNamespace(namespace_operator),
            Node(
                package='tod_state_machine',
                executable='OperatorStateMachine',
                namespace=namespace_operator,
                name='OperatorStateMachine',
                output='screen',
            )
        ]
    )
    return LaunchDescription([
        operator_group
    ])