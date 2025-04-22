import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Top-level launch file that declares:
      - config_path: path to the configuration directory
      - mode: string, which if equals 'vehicle' will launch the actuation/sensing nodes
    """

    # Declare top-level arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(
            get_package_share_directory('tod_edgarautoware_interface'),
            'config'
        ),
        description='Path to config directory for EDGAR interface nodes'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='vehicle',  # or 'operator', 'both', etc. as you wish
        description='Operating mode; only launch EDGAR nodes if "vehicle"'
    )

    # Capture them as LaunchConfigurations
    config_path = LaunchConfiguration('config_path')
    mode = LaunchConfiguration('mode')

    # Path to your package's share directory
    package_dir = get_package_share_directory('tod_edgarautoware_interface')

    # Sub-launches (pass down the config_path explicitly as an argument)
    actuation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'edgarautoware_actuation_interface.launch.py')),
        launch_arguments={'config_path': config_path}.items()
    )

    sensing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'edgarautoware_sensing_interface.launch.py')),
        launch_arguments={'config_path': config_path}.items()
    )

    automation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'edgarautoware_automation_interface.launch.py')),
        launch_arguments={'config_path': config_path}.items()
    )

    # Group them behind an IfCondition that checks mode == 'vehicle'
    vehicle_only_group = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'vehicle'"]) 
        ),
        actions=[
            actuation_launch,
            sensing_launch,
            automation_launch
        ]
    )

    return LaunchDescription([
        config_path_arg,
        mode_arg,
        vehicle_only_group
    ])