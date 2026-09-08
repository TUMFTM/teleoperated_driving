import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from utils.create_launch_description import create_launch_description
from utils.parse_launch_setup import parse_launch_setup
from utils.parse_remappings import parse_remappings


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("tod_launch"), "config")
    launch_args, packages = parse_launch_setup(
        os.path.join(config_dir, "launch_setup_peanut01_video.yaml"),
        mode="vehicle",
    )
    remappings = parse_remappings(os.path.join(config_dir, "remappings.yaml"))
    description = create_launch_description(
        launch_args, packages, remappings, mode="vehicle"
    )
    lidar_params = os.path.join(
        config_dir, "package_config", "tod_lidar", "params.yaml"
    )
    control_bridge_params = os.path.join(
        config_dir, "package_config", "tod_peanut01_interface", "params.yaml"
    )
    sensor_domain_id = os.getenv("TOD_LIDAR_SENSOR_DOMAIN_ID", "0")
    description.add_action(
        Node(
            package="tod_lidar",
            executable="PointCloudEncoderNode",
            namespace="/vehicle/network/lidar",
            name="PointCloudEncoder",
            output="screen",
            parameters=[
                lidar_params,
                {
                    "vehicleID": "peanut01",
                    "config_path": config_dir,
                    "target_points": 20000,
                    "input_reliability": "reliable",
                    "voxel_leaf_size": 0.5,
                    "crop_box_min_x": -10.0,
                    "crop_box_min_y": -15.0,
                    "crop_box_min_z": -2.0,
                    "crop_box_max_x": 30.0,
                    "crop_box_max_y": 15.0,
                    "crop_box_max_z": 3.0,
                    "draco_encode_speed": 7,
                    "draco_decode_speed": 7,
                },
            ],
            remappings=[
                (
                    "output/pointcloud_compressed",
                    "/peanut01/lidar/compressed/domain0",
                )
            ],
            additional_env={"ROS_DOMAIN_ID": sensor_domain_id},
        )
    )
    description.add_action(
        ExecuteProcess(
            cmd=["python3", "/opt/tod-tools/peanut01_lidar_domain_bridge.py"],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        )
    )
    description.add_action(
        ExecuteProcess(
            cmd=["python3", "/opt/tod-tools/peanut01_vehicle_state_bridge.py"],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        )
    )
    description.add_action(
        ExecuteProcess(
            cmd=[
                "python3",
                "/opt/tod-tools/peanut01_control_bridge.py",
                "--ros-args",
                "--params-file",
                control_bridge_params,
            ],
            additional_env={"TOD_CONTROL_TARGET_DOMAIN_ID": "0"},
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        )
    )
    return description
