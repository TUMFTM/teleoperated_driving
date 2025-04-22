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
    Top-level launch file for the 'tod_rc_car_interface' package.
    Declares:
      - config_path (default points to this package's config folder)
      - mode (default 'vehicle', and sub-launches only run if mode=='vehicle')
    """
    # Declare top-level arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(
            get_package_share_directory('tod_rc-car_interface'),
            'config'
        ),
        description='Path to config directory for the RC Car interface nodes'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='vehicle',  # or 'operator', or 'both', etc. as needed
        description='Operating mode; only launch RC car nodes if "vehicle"'
    )

    # Capture them as LaunchConfigurations
    config_path = LaunchConfiguration('config_path')
    mode = LaunchConfiguration('mode')

    # Path to your package share directory
    package_dir = get_package_share_directory('tod_rc-car_interface')

    # Sub-launches: pass down config_path to each included file
    actuation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'rc-car_actuation_interface.launch.py')),
        launch_arguments={'config_path': config_path}.items()
    )

    sensing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'rc-car_sensing_interface.launch.py')),
        launch_arguments={'config_path': config_path}.items()
    )

    # Group them behind IfCondition => only launch if mode == 'vehicle'
    vehicle_only_group = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'vehicle'"])
        ),
        actions=[
            actuation_launch,
            sensing_launch
        ]
    )

    return LaunchDescription([
        config_path_arg,
        mode_arg,
        vehicle_only_group
    ])
