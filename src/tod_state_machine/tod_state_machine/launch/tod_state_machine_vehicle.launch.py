from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    namespace_vehicle = '/vehicle/statemachine'
    vehicle_group = GroupAction(
            actions=[
                PushRosNamespace(namespace_vehicle),
                Node(
                    package='tod_state_machine',
                    executable='VehicleStateMachine',
                    namespace=namespace_vehicle,
                    name='VehicleStateMachine',
                    output='screen',
                )
            ]
        )

    return LaunchDescription([
        vehicle_group
    ])

