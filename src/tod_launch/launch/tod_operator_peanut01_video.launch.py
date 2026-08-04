import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess

from utils.create_launch_description import create_launch_description
from utils.parse_launch_setup import parse_launch_setup
from utils.parse_remappings import parse_remappings


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("tod_launch"), "config")
    launch_args, packages = parse_launch_setup(
        os.path.join(config_dir, "launch_setup_peanut01_video.yaml"),
        mode="operator",
    )
    launch_args.append(DeclareLaunchArgument("managerOnly", default_value="true"))
    remappings = parse_remappings(os.path.join(config_dir, "remappings.yaml"))
    description = create_launch_description(
        launch_args, packages, remappings, mode="operator"
    )
    description.add_action(
        ExecuteProcess(
            cmd=["python3", "/opt/tod-tools/peanut01_video_viewer.py"],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        )
    )
    return description
