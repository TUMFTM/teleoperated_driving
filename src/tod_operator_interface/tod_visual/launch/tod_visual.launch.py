import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    
    # Launch arguments
    vehicle_id =  LaunchConfiguration('vehicleID')
    vehicle_id_arg = DeclareLaunchArgument(
        'vehicleID',
        default_value='edgar',
        description='ID of the currently used vehicle.'
    )
    manager_only = LaunchConfiguration('managerOnly')
    manager_only_arg = DeclareLaunchArgument(
        'managerOnly',
        default_value='false',
        description='Launch Manager without Visual and VideoManager.'
    )

    default_config_path = os.path.join(
        get_package_share_directory('tod_visual'),
        'config', 
    )
    
    config_path = LaunchConfiguration('config_path')
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Path to the config folder.'
    )

    # Node params (path to params.yaml relative to config_path)
    node_param_path = PathJoinSubstitution([
        LaunchConfiguration('config_path'),
        'package_config',
        'tod_visual',
        'params.yaml'
    ])

    # Nodes
    visual_node = Node(
        package='tod_visual',
        executable='visual',
        namespace='/operator/interface/visual',
        name='Visual',
        output='screen',
        parameters=[
            {'vehicleID': vehicle_id},
            {'config_path': config_path},
            node_param_path
        ],
        remappings=[ ],
        condition=UnlessCondition(manager_only)
    )

    manager_node = Node(
        package='tod_visual',  
        executable='manager',  
        namespace='/operator/interface/manager', 
        name='Manager',        
        output='screen',
        parameters=[
            {'vehicleID': vehicle_id},
            {'config_path': config_path},
            node_param_path
        ],
        remappings=[ ]
    )

    video_manager_node = Node(
        package='tod_visual',  
        executable='video_manager',  
        namespace='/operator/interface/video_manager', 
        name='VideoManager',        
        output='screen',
        parameters=[
                {'vehicleID': vehicle_id},
                {'config_path': config_path},
                node_param_path
            ],
        remappings=[ ],
        condition=UnlessCondition(manager_only)
    )

    launch_description = LaunchDescription([
        vehicle_id_arg,
        manager_only_arg,
        config_path_arg,
        visual_node, 
        manager_node,
        video_manager_node
    ])
    
    return launch_description
