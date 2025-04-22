import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

def create_launch_description(launch_arguments_list, packages_to_launch, remappings_dict, mode):
    '''
    - purpose: Generate a LaunchDescription object dynamically from arguments and package names.
    - input:
        - launch_arguments_list: A list of DeclareLaunchArgument objects
        - packages_to_launch: A dict of packages and their flag to include in the launch.
        - remappings_dict: A dict containing remappings for the individual packages.
        - mode: Weather to search for operator or vehicle or both launch files
    - output: A LaunchDescription object.
    '''

    launch_descriptions = []

    # Change the default config path to this package
    config_path = os.path.join(
        get_package_share_directory('tod_launch'),
        'config'
    )

    launch_arguments_list.append(
        DeclareLaunchArgument(
            'config_path', default_value = config_path
        )
    )

    # Mapping of modes to suffix mapping for packages with both 
    # operator and vehicle launch files
    mode_suffixes = {
        'operator': [f'_{mode}.launch.py'],
        'vehicle': [f'_{mode}.launch.py'],
        'both': ['_operator.launch.py', '_vehicle.launch.py']
    }

    for package_name in packages_to_launch:
        if packages_to_launch[package_name] is True:
            try:
                share_dir = get_package_share_directory(package_name)
                possible_launch_files = [f'{package_name}.launch.py']

                if mode in mode_suffixes:
                    possible_launch_files.extend(
                        f'{package_name}{suffix}' for suffix in mode_suffixes[mode]
                    )

                found_launch_file = False
                for file_name in possible_launch_files:
                    launch_file_path = os.path.join(
                        share_dir,
                        'launch',
                        file_name
                    )
                    if os.path.exists(launch_file_path):
                        # Important:
                        # Current launch args are automatically passed to the underlying launch 
                        # No need for manual passing
                        remap_actions = []
                        if package_name in remappings_dict:
                            for from_topic, to_topic in remappings_dict[package_name]:
                                remap_actions.append(SetRemap(src=from_topic, dst=to_topic)) 
                        
                        package_launch_description = GroupAction(
                            actions=[
                                *remap_actions,
                                IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file_path))
                            ]
                        )
                        
                        launch_descriptions.append(package_launch_description)
                        found_launch_file = True

                if not found_launch_file:
                    raise FileNotFoundError(
                        f"[Error] in create_launch_description(): None of the expected launch "
                        f"files {possible_launch_files} were found in "
                        f"{package_name}/launch. Check naming conventions or mode."
                    )

            except PackageNotFoundError:
                print(
                    f"[Error] in create_launch_description(): Package {package_name} not found.\n"
                    "        Please check the following:\n" 
                    "        - typos related to the package name in launch_setup.yaml\n"
                    "        - ensure the package has been built"
                    "        - ensure that the share directory has been installed properly"
                    )

    return LaunchDescription([
        *launch_arguments_list, # unpack the list of launch_arguments
        *launch_descriptions    # unpack the list of launch_descriptions
    ])