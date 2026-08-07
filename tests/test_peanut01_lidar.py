import pathlib
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
PROFILE = ROOT / "config/config/launch_setup_peanut01_video.yaml"
SENSORS = ROOT / "config/config/vehicle_config/peanut01/sensors-lidar.yaml"
COMPOSE = ROOT / "docker-compose.peanut01-video.yaml"
VEHICLE_LAUNCH = ROOT / "src/tod_launch/launch/tod_vehicle_peanut01_video.launch.py"
OPERATOR_LAUNCH = ROOT / "src/tod_launch/launch/tod_operator_peanut01_video.launch.py"
DOMAIN_BRIDGE = ROOT / "work/peanut01_lidar_domain_bridge.py"
VIEWER = ROOT / "work/peanut01_pointcloud_viewer.py"
ENCODER_NODE = ROOT / "src/tod_network/tod_lidar/src/vehicle/point_cloud_encoder_node.cpp"
TRANSFORMS = ROOT / "config/config/vehicle_config/peanut01/vehicle-transforms.yaml"
VISUAL = ROOT / "config/config/vehicle_config/peanut01/visual-video.yaml"
CALIBRATION = ROOT / "config/config/vehicle_config/peanut01/camera-calibration/frontcenter.yaml"
MODEL_ROOT = ROOT / "config/config/vehicle_config/peanut01/model-mesh"
STATE_LAYER = ROOT / "src/tod_operator_interface/tod_visual/src/tod_applications/visual/application_layer/src/state_layer.cpp"
DOCKERFILE = ROOT / "docker/dockerfile"


class Peanut01LidarDeploymentTest(unittest.TestCase):
    def test_profile_does_not_duplicate_custom_lidar_nodes(self):
        profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
        packages = profile["packages_to_launch"]

        self.assertNotIn("tod_lidar", packages["operator"])
        self.assertNotIn("tod_lidar", packages["both"])

    def test_sensor_config_selects_main_lidar(self):
        sensors = yaml.safe_load(SENSORS.read_text(encoding="utf-8"))

        topic = sensors["lidar_topics_namespace"] + sensors["pointcloud_name"]
        self.assertEqual("/main_lidar/rslidar_points", topic)

    def test_vehicle_launch_encodes_in_sensor_domain_and_bridges_compressed_data(self):
        launch = VEHICLE_LAUNCH.read_text(encoding="utf-8")

        self.assertIn('package="tod_lidar"', launch)
        self.assertIn('executable="PointCloudEncoderNode"', launch)
        self.assertIn('os.getenv("TOD_LIDAR_SENSOR_DOMAIN_ID", "0")', launch)
        self.assertIn('additional_env={"ROS_DOMAIN_ID": sensor_domain_id}', launch)
        self.assertIn("peanut01_lidar_domain_bridge.py", launch)
        self.assertIn('"target_points": 20000', launch)
        self.assertIn('"input_reliability": "reliable"', launch)

    def test_operator_launch_starts_native_visual_and_static_transform(self):
        launch = OPERATOR_LAUNCH.read_text(encoding="utf-8")

        self.assertIn('package="tod_lidar"', launch)
        self.assertIn('executable="PointCloudDecoderNode"', launch)
        self.assertIn('"target_frame": "base_footprint"', launch)
        self.assertIn('package="tod_transform"', launch)
        self.assertIn('executable="StaticTransformPublisher"', launch)
        self.assertIn('package="tod_visual"', launch)
        self.assertIn('executable="visual"', launch)
        self.assertNotIn("peanut01_pointcloud_viewer.py", launch)

    def test_vehicle_image_contains_lidar_tool_and_operator_keeps_gpu_access(self):
        compose = yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))["services"]
        dockerfile = DOCKERFILE.read_text(encoding="utf-8")

        self.assertIn("work/peanut01_lidar_domain_bridge.py", dockerfile)
        self.assertNotIn("peanut01_lidar_overlay", COMPOSE.read_text(encoding="utf-8"))
        self.assertNotIn("peanut01_pointcloud_viewer.py", dockerfile)
        devices = compose["tod_operator"]["deploy"]["resources"]["reservations"]["devices"]
        self.assertEqual("nvidia", devices[0]["driver"])
        self.assertIn("gpu", devices[0]["capabilities"])

    def test_native_visual_renders_pointcloud_during_direct_control(self):
        state_layer = STATE_LAYER.read_text(encoding="utf-8")
        compose = yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))["services"]
        operator = compose["tod_operator"]

        registration = next(
            line for line in state_layer.splitlines()
            if 'register_entity("PointCloudRenderer"' in line
        )
        self.assertIn("CONTROL_MODE_DIRECT", registration)

        self.assertNotIn("volumes", operator)
        self.assertIn("tod_operator_peanut01_video.launch.py", " ".join(operator["command"]))

    def test_domain_bridge_only_copies_compressed_cloud_between_domains(self):
        bridge = DOMAIN_BRIDGE.read_text(encoding="utf-8")

        self.assertIn("CompressedPointCloud", bridge)
        self.assertIn('TOD_LIDAR_SENSOR_DOMAIN_ID", "0"', bridge)
        self.assertIn('ROS_DOMAIN_ID", "7"', bridge)
        self.assertIn("domain_id=sensor_domain_id", bridge)
        self.assertIn("domain_id=tod_domain_id", bridge)
        self.assertNotIn("PointCloud2", bridge)

    def test_encoder_can_match_reliable_merged_lidar_publisher(self):
        encoder = ENCODER_NODE.read_text(encoding="utf-8")

        self.assertIn('declare_parameter("input_reliability", "best_effort")', encoder)
        self.assertIn('input_reliability == "reliable"', encoder)
        self.assertIn("qos.reliable()", encoder)

    def test_native_visual_assets_cover_peanut01_runtime_dependencies(self):
        self.assertFalse(VIEWER.exists())
        transforms = yaml.safe_load(TRANSFORMS.read_text(encoding="utf-8"))
        visual = yaml.safe_load(VISUAL.read_text(encoding="utf-8"))

        self.assertEqual("base_footprint", transforms["Transform1"]["from"])
        self.assertEqual("main_lidar", transforms["Transform1"]["to"])
        self.assertEqual([0.22, 0.0, 1.3], transforms["Transform1"]["translation_x_y_z"])
        self.assertIn("frontcenter", visual)
        self.assertTrue(CALIBRATION.exists())
        for model in ("model_chassis", "model_steeringWheel", "model_wheel"):
            self.assertTrue((MODEL_ROOT / model / f"{model}.obj").exists())


if __name__ == "__main__":
    unittest.main()
