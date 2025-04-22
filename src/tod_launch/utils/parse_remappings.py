import os
import yaml
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def parse_remappings(yaml_path):
    '''
    Description: Function to parse the remappings.yaml file to remap topics from nodes to launch.
    - input: 
        - yaml_path: path to the yaml file
    - output:
        a dict which is composed as follows:
        {
            'package_name': [
                ('absolute_topic_name_1', 'other_absolute_topic_name_1'),
                ('absolute_topic_name_2', 'other_absolute_topic_name_2'),
                ...
            ],
            ...
        }
    '''
    if not os.path.exists:
        raise FileNotFoundError(
            f"yaml file {yaml_path} for launch configuration not found"
        )
    with open(yaml_path, 'r') as file:
        data = yaml.safe_load(file)
    
    remappings_dict = {}

    for package_name, remappings in data.items():
        remappings_dict[package_name] = [
            (item['from'], item['to'])
            for item in remappings
            if item.get('from') and item.get('to')  
        ]

    return remappings_dict