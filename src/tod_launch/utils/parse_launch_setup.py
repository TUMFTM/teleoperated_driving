import os
import yaml
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory
from ament_index_python import PackageNotFoundError

def parse_launch_setup(yaml_path, mode):
    '''
    Description: Function to parse the launch_setup.yaml file to variables
    to be used by the tod_launch procedure
    - input: 
        - yaml_path: path to the yaml file
        - mode: 'operator' or 'vehicle' or 'both' to get respective packages to launch  
    - output:
        a dict which is composed as follows: 
        first entry is a list of launch arguments
        second entry is of type dict
    '''
    if not os.path.exists:
        raise FileNotFoundError(
            f"yaml file {yaml_path} for launch configuration not found"
        )
    with open(yaml_path, 'r') as file:
        config = yaml.safe_load(file)

    # ===== Get launch parameters from the yaml ===== #
    vehicle_id_str = config.get('launch_parameters', {}).get('vehicleID', 'default_vehicle')
    vehicle_mode_str = config.get('launch_parameters', {}).get('mode', 'default_mode')

    vehicle_id_arg = DeclareLaunchArgument(
        'vehicleID', 
        default_value=vehicle_id_str, 
        description="Vehicle ID of the vehicle to launch"
    )
    mode_arg = DeclareLaunchArgument(
        'mode', 
        default_value=vehicle_mode_str, 
        description="Operating mode, choice of 'vehicle' and 'only_sim'"
    )

    launch_argument_list = [
        vehicle_id_arg,
        mode_arg          # add further args here if needed
    ]                      

    # ===== Extract packages to launch and safe as dict ===== #
    packages_to_launch = config.get('packages_to_launch', {})

    mode_keys = {
        'operator': ['operator', 'both'],
        'vehicle': ['vehicle', 'both'],
        'both': ['operator', 'vehicle', 'both']
    }

    if mode in mode_keys:
        packages_dict = {
            key: value
            for key in mode_keys[mode]
            for key, value in packages_to_launch.get(key, {}).items()
        }
    else:
        raise AttributeError(
            "wrong 'mode' variable passed to function parse_launch_setup(). "  
            "choose 'operator', 'vehicle' or 'both'."
        )

    # ===== Add vehicle interface to launch based on mode and vehicleID ===== # 
    vehicle_interface_dict = add_vehicle_interface_to_packages_dict(
        launch_mode = mode,
        vehicle_mode = vehicle_mode_str,
        vehicle_id = vehicle_id_str
    )
    packages_dict.update(vehicle_interface_dict)

    # ===== Compose the dict to return ===== #
    return_dict = {
        'launch_arg_list':launch_argument_list,  # list of DeclareLaunchArgument
        'packages_to_launch_dict':packages_dict  # dict
        }   

    return launch_argument_list, packages_dict


def add_vehicle_interface_to_packages_dict(launch_mode, vehicle_mode, vehicle_id):
    """
    Automatically adds the desired vehicle interface to the dictionary of packages to launch.

    Args:
        launch_mode (str): Specifies whether the launch logic is run in 'operator', 'vehicle', or 'both' mode.
        vehicle_mode (str): Options are {'only_sim', 'vehicle'}:
            - 'only_sim': Selects the 'tod_vehicle_sim' package.
            - 'vehicle': Selects the corresponding 'tod_<vehicleID>_interface' package.
        vehicle_id (str): The ID of the vehicle for which the interface should be loaded.

    Returns:
        dict: A dictionary containing the vehicle interface to launch, e.g., {'tod_edgar_interface': True}.
    """

    vehicle_interface_dict = {}

    if launch_mode in {'vehicle', 'both'}:
        if vehicle_mode == 'only_sim':
            vehicle_interface_dict = {'tod_vehicle_sim': True}
        elif vehicle_mode == 'vehicle':
            vehicle_interface_pkg_str = f'tod_{vehicle_id}_interface'
            try: 
                get_package_share_directory(vehicle_interface_pkg_str)
                vehicle_interface_dict = {vehicle_interface_pkg_str: True}
            except PackageNotFoundError:
                print(
                    f"[Error] Vehicle interface package '{vehicle_interface_pkg_str}' could not be found. "
                    "No vehicle interface will be loaded. Please check the following:\n"
                    "- Typos in 'vehicleID' in tod_launch_config/launch_setup.yaml\n"
                    "- If the 'tod_<vehicleID>_interface' package is built and sourced\n"
                    "- Naming convention: Package name must be 'tod_<vehicleID>_interface'"
                )
            except Exception as e: 
                print(
                    f"[Unexpected Error] {e}. Please check:\n"
                    "- File: tod_launch/utils/parse_launch_setup.py\n"
                    "- Function: add_vehicle_interface_to_packages_dict()"
                    )
        else:
            raise AttributeError(
                "Invalid 'vehicle_mode' provided. Expected 'vehicle' or 'only_sim'. "
                "Check for typos in 'mode' entry in tod_launch/config/launch_setup.yaml."
            )

    return vehicle_interface_dict