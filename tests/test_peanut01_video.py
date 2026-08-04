import pathlib
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
PROFILE = ROOT / "config/config/launch_setup_peanut01_video.yaml"
CAMERA = ROOT / "config/config/vehicle_config/peanut01/sensors-camera.yaml"
VISUAL = ROOT / "config/config/vehicle_config/peanut01/visual-video.yaml"
CALIBRATION = (
    ROOT
    / "config/config/vehicle_config/peanut01/camera-calibration/camera1.yaml"
)
STREAM = ROOT / "config/config/vehicle_config/peanut01/stream_settings.yml"
ROUTERS = ROOT / "config/config/vehicle_config/peanut01/router_settings.yml"
COMPOSE = ROOT / "docker-compose.peanut01-video.yaml"
VEHICLE_TOP_LEVEL = (
    ROOT / "src/tod_launch/launch/tod_vehicle_peanut01_video.launch.py"
)
OPERATOR_TOP_LEVEL = (
    ROOT / "src/tod_launch/launch/tod_operator_peanut01_video.launch.py"
)
VIDEO_VIEWER = ROOT / "work/peanut01_video_viewer.py"
VEHICLE_RTSP_LAUNCH = (
    ROOT / "src/tod_network/tod_rtsp/launch/tod_rtsp_vehicle.launch.py"
)
OPERATOR_RTSP_LAUNCH = (
    ROOT / "src/tod_network/tod_rtsp/launch/tod_rtsp_operator.launch.py"
)
RTSP_SERVER = ROOT / "src/tod_network/tod_rtsp/src/vehicle/rtsp_server.cpp"
RTSP_STREAM = ROOT / "src/tod_network/tod_rtsp/src/vehicle/rtsp_stream.cpp"


def load_yaml(testcase, path):
    testcase.assertTrue(path.exists(), f"missing {path.relative_to(ROOT)}")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


class Peanut01VideoDeploymentTest(unittest.TestCase):
    def test_profile_enables_video_without_lidar_or_projection(self):
        profile = load_yaml(self, PROFILE)

        self.assertEqual("peanut01", profile["launch_parameters"]["vehicleID"])
        packages = profile["packages_to_launch"]
        self.assertTrue(packages["both"]["tod_rtsp"])
        self.assertFalse(packages["both"]["tod_lidar"])
        self.assertFalse(packages["operator"]["tod_projection"])
        self.assertFalse(packages["both"]["tod_transform"])

    def test_camera_topic_and_low_latency_profile(self):
        camera = load_yaml(self, CAMERA)
        stream = load_yaml(self, STREAM)["video_settings"]
        camera_topic = (
            camera["camera_topics_namespace"]
            + camera["camera0"]["name"]
            + camera["camera_image_name"]
        )

        self.assertEqual("/sensing/camera/camera1/image_raw", camera_topic)
        self.assertFalse(camera["camera0"]["project_on"])
        self.assertEqual((1920, 1080), (stream["width"], stream["height"]))
        self.assertEqual((960, 540), (stream["actual_width"], stream["actual_height"]))
        self.assertEqual("0p500", stream["scaling_factor"])
        self.assertEqual(1500, stream["bitrate"])

    def test_visual_config_is_explicitly_non_projected_and_uncalibrated(self):
        visual = load_yaml(self, VISUAL)
        calibration = load_yaml(self, CALIBRATION)

        self.assertEqual(0, visual["camera1"]["VideoComponent"]["ProjectionMode"])
        self.assertEqual("uncalibrated", calibration["camera_name"])
        self.assertEqual(1920, calibration["image_width"])
        self.assertEqual(1080, calibration["image_height"])

    def test_router_config_has_no_machine_specific_address(self):
        routers = load_yaml(self, ROUTERS)

        self.assertEqual([], routers["ips"])

    def test_compose_overlay_selects_video_launchers_and_sensor_domain(self):
        compose = load_yaml(self, COMPOSE)["services"]
        vehicle = compose["tod_vehicle"]
        operator = compose["tod_operator"]
        vehicle_command = " ".join(vehicle["command"])
        operator_command = " ".join(operator["command"])

        self.assertEqual("${TOD_RTSP_SENSOR_DOMAIN_ID:-0}", vehicle["environment"]["TOD_RTSP_SENSOR_DOMAIN_ID"])
        self.assertIn("peanut01_video_overlay/install/setup.bash", vehicle_command)
        self.assertIn("tod_vehicle_peanut01_video.launch.py", vehicle_command)
        self.assertIn("tod_operator_peanut01_video.launch.py", operator_command)
        self.assertTrue(
            any(
                volume["target"].endswith("tod_rtsp_vehicle.launch.py")
                for volume in vehicle["volumes"]
            )
        )
        self.assertTrue(
            any(
                volume["target"] == "/opt/tod-tools/peanut01_video_viewer.py"
                for volume in operator["volumes"]
            )
        )

    def test_top_level_launchers_use_video_profile(self):
        self.assertTrue(VEHICLE_TOP_LEVEL.exists(), "missing vehicle video launcher")
        self.assertTrue(OPERATOR_TOP_LEVEL.exists(), "missing operator video launcher")

        for launcher in (VEHICLE_TOP_LEVEL, OPERATOR_TOP_LEVEL):
            text = launcher.read_text(encoding="utf-8")
            self.assertIn("launch_setup_peanut01_video.yaml", text)
        operator = OPERATOR_TOP_LEVEL.read_text(encoding="utf-8")
        self.assertIn('DeclareLaunchArgument("managerOnly", default_value="true")', operator)
        self.assertIn("ExecuteProcess", operator)
        self.assertIn("/opt/tod-tools/peanut01_video_viewer.py", operator)

    def test_standalone_viewer_subscribes_to_decoded_camera_topic(self):
        self.assertTrue(VIDEO_VIEWER.exists(), "missing standalone video viewer")
        viewer = VIDEO_VIEWER.read_text(encoding="utf-8")

        self.assertIn("/operator/network/video/camera1/image", viewer)
        self.assertIn("qos_profile_sensor_data", viewer)
        self.assertIn("cv2.imshow", viewer)
        self.assertIn("COLOR_RGB2BGR", viewer)

    def test_rtsp_launches_support_domain_and_config_overrides(self):
        vehicle = VEHICLE_RTSP_LAUNCH.read_text(encoding="utf-8")
        operator = OPERATOR_RTSP_LAUNCH.read_text(encoding="utf-8")

        self.assertIn("TOD_RTSP_SENSOR_DOMAIN_ID", vehicle)
        self.assertIn("additional_env", vehicle)
        for launch in (vehicle, operator):
            self.assertIn("TOD_RTSP_CONFIG_PATH", launch)

    def test_rtsp_server_supports_empty_router_list(self):
        server = RTSP_SERVER.read_text(encoding="utf-8")

        self.assertIn("_ips.empty()", server)
        self.assertIn('"0.0.0.0"', server)

    def test_rtsp_stream_owns_exact_image_buffer_and_applies_initial_scaling(self):
        stream = RTSP_STREAM.read_text(encoding="utf-8")

        self.assertIn("gst_buffer_new_allocate", stream)
        self.assertIn("gst_buffer_fill", stream)
        self.assertIn("latest_image_->data.size()", stream)
        self.assertNotIn("latest_image_->width * latest_image_->step", stream)
        self.assertIn("configured_output_dimensions", stream)
        self.assertIn("kVideoDimensionAlignment = 2", stream)
        self.assertIn("set_scaling_caps", stream)
        self.assertIn("gst_caps_unref", stream)


if __name__ == "__main__":
    unittest.main()
