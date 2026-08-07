import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

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
    visual_remappings = remappings.get("tod_visual", [])
    description = create_launch_description(
        launch_args, packages, remappings, mode="operator"
    )
    lidar_params = os.path.join(
        config_dir, "package_config", "tod_lidar", "params.yaml"
    )
    description.add_action(
        Node(
            package="tod_lidar",
            executable="PointCloudDecoderNode",
            namespace="/operator/network/lidar",
            name="PointCloudDecoder",
            output="screen",
            parameters=[lidar_params, {"target_frame": "base_footprint"}],
            remappings=[
                (
                    "input/pointcloud_compressed",
                    "/operator/network/data/from_vehicle/pointcloud_compressed",
                ),
                (
                    "output/pointcloud_decompressed",
                    "/operator/network/lidar/pointcloud_decompressed",
                ),
            ],
        )
    )
    description.add_action(
        Node(
            package="tod_transform",
            executable="StaticTransformPublisher",
            namespace="/operator/transform",
            name="StaticTransformPublisher",
            output="screen",
            parameters=[
                {"vehicleID": "peanut01"},
                {"config_path": config_dir},
            ],
        )
    )
    description.add_action(
        Node(
            package="tod_visual",
            executable="visual",
            namespace="/operator/interface/visual",
            name="Visual",
            output="screen",
            parameters=[
                {"vehicleID": "peanut01"},
                {"config_path": config_dir},
                os.path.join(
                    config_dir, "package_config", "tod_visual", "params.yaml"
                ),
            ],
            remappings=visual_remappings,
        )
    )
    return description
