import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    vehicle_id_arg = DeclareLaunchArgument(
        'vehicleID', default_value = "edgar"
    )

    default_config_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..',
        'config'
    )
 
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path
    )

    lane_projection_node = Node(
        namespace   = '/operator/projection',
        package     = 'tod_projection',
        executable  = 'OperatorLaneProjection',
        name        = 'LaneProjection',
        output      = 'screen',
        parameters  = [
            {'vehicleID': LaunchConfiguration('vehicleID')},
            {'config_path': LaunchConfiguration('config_path')}
        ]
    )

    return LaunchDescription([
        vehicle_id_arg,
        config_path_arg,
        lane_projection_node
    ])

