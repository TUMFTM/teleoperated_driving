import os
from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ''' ===== Arguments ===== 
    - config_file_name: Name of the configuration file for your input device. Choose from confif folder
    - config_mode_arg: True to print axis and value info logs to the console if new input is recieved
    - debug_arg: True to print debug logs
    '''
    config_file_name = 'virtual.yaml'
    config_mode_arg  = DeclareLaunchArgument('ConfigMode', default_value = 'False')
    debug_arg        = DeclareLaunchArgument('debug', default_value = 'False')

    config_file_path = os.path.join(get_package_share_directory('tod_input_devices'), 'config', config_file_name)
    config_file_arg  = DeclareLaunchArgument('ConfigFile', default_value = config_file_path)

    input_device_node = Node(
        namespace   = '/operator/input_devices',
        package     = 'tod_input_devices',
        executable  = 'InputDevice',
        name        = 'InputDevice',
        output      = 'screen',
        parameters  = [
            LaunchConfiguration('ConfigFile'),
            {'ConfigMode': LaunchConfiguration('ConfigMode')}, 
            {'debug': LaunchConfiguration('debug')}  
        ]
    )

    return LaunchDescription([
        config_file_arg,
        config_mode_arg,
        debug_arg,
        input_device_node
    ])