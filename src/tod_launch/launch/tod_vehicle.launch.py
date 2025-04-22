import os
from ament_index_python.packages import get_package_share_directory
from utils.parse_launch_setup import parse_launch_setup
from utils.parse_remappings import parse_remappings
from utils.create_launch_description import create_launch_description

def generate_launch_description():

    launch_setup_yaml_path = os.path.join(
        get_package_share_directory('tod_launch'),
        'config',
        'launch_setup.yaml'
    )

    remappings_yaml_path = os.path.join(
        get_package_share_directory('tod_launch'),
        'config',
        'remappings.yaml'
    )

    launch_arg_list, packages_dict = parse_launch_setup(
        launch_setup_yaml_path,
        mode='vehicle'
    ) 

    remappings_dict = parse_remappings(
        remappings_yaml_path
    )

    launch_description = create_launch_description(
        launch_arg_list,
        packages_dict,
        remappings_dict,
        mode = 'vehicle'
    )

    return launch_description