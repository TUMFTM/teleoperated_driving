import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ns="/dummy"

    return LaunchDescription([
        Node(
             package="tod_dummy",
             namespace=ns,
             executable="JoystickDummyPub",
             name="joystick_dummy_pub",
             remappings= []
        ),

        Node(
             package="tod_dummy",
             namespace=ns,
             executable="PathDummyPub",
             name="path_dummy_pub",
             remappings= []
        ),

        Node(
             package="tod_dummy",
             namespace=ns,
             executable="StatusDummyPub",
             name="status_dummy_pub",
             remappings= []
        )
    ])