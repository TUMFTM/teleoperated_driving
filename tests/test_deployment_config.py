import pathlib
import unittest


REPO = pathlib.Path(__file__).resolve().parents[1]


class DeploymentConfigTests(unittest.TestCase):
    def test_g923_is_a_regular_input_profile(self):
        config = (
            REPO
            / "src/tod_operator_interface/tod_input_devices/config/logitechg923.yaml"
        )
        self.assertTrue(config.is_file())

        override = (REPO / "docker-compose.override.yaml").read_text(encoding="utf-8")
        self.assertIn('source: "/dev/input"', override)
        self.assertIn("target: /dev/input", override)
        self.assertIn("logitechg923.yaml", override)
        self.assertNotIn("target: /dev/input/js0", override)
        self.assertNotIn("target: /home/tum/", override)
        self.assertNotIn("target: /home/${DOCKER_USERNAME:?}/wsp/install/tod_input_devices/share/tod_input_devices/config/virtual.yaml", override)

    def test_network_interfaces_are_configurable(self):
        compose = (REPO / "docker-compose.yaml").read_text(encoding="utf-8")
        env = (REPO / ".env").read_text(encoding="utf-8")

        self.assertIn("vehicleNetworkInterface:=${VEHICLE_NETWORK_INTERFACE:?}", compose)
        self.assertIn("operatorNetworkInterface:=${OPERATOR_NETWORK_INTERFACE:?}", compose)
        self.assertIn("VEHICLE_NETWORK_INTERFACE=", env)
        self.assertIn("OPERATOR_NETWORK_INTERFACE=", env)

    def test_launch_arguments_override_monitoring_yaml(self):
        operator_launch = (
            REPO
            / "src/tod_monitoring/tod_network_monitoring/launch/tod_network_monitoring_operator.launch.py"
        ).read_text(encoding="utf-8")
        vehicle_launch = (
            REPO
            / "src/tod_monitoring/tod_network_monitoring/launch/tod_network_monitoring_vehicle.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn("LaunchConfiguration('operatorNetworkInterface')", operator_launch)
        tester_block = operator_launch.split("network_tester_operator_node", 1)[1].split(
            "network_monitoring_operator_node", 1
        )[0]
        monitor_block = operator_launch.split(
            "network_monitoring_operator_node", 1
        )[1].split("packet_logger_operator_node", 1)[0]
        logger_block = operator_launch.split("packet_logger_operator_node", 1)[1]
        self.assertNotIn("{'network_interface': network_interface}", tester_block)
        self.assertIn("{'network_interface': network_interface}", monitor_block)
        self.assertIn("{'network_interface': network_interface}", logger_block)
        self.assertIn("LaunchConfiguration('vehicleNetworkInterface')", vehicle_launch)
        self.assertEqual(
            vehicle_launch.count("{'network_interface': network_interface}"), 2
        )

    def test_remote_low_resource_build_workarounds_are_preserved(self):
        env = (REPO / ".env").read_text(encoding="utf-8")
        dockerfile = (REPO / "docker/dockerfile").read_text(encoding="utf-8")

        self.assertEqual(env.count("--parallel-workers 1"), 2)
        self.assertIn("COPY ./docker/colcon_retry.sh", dockerfile)
        self.assertIn("COPY ./docker/rosdep_retry.sh", dockerfile)
        self.assertIn("colcon_retry vehicle", dockerfile)
        self.assertIn("colcon_retry operator", dockerfile)
        self.assertIn("rosdep_retry ${ROS_DISTRO}", dockerfile)

        colcon_retry = (REPO / "docker/colcon_retry.sh").read_text(encoding="utf-8")
        rosdep_retry = (REPO / "docker/rosdep_retry.sh").read_text(encoding="utf-8")
        self.assertIn('COLCON_RETRY_MAX:-5', colcon_retry)
        self.assertIn('ROSDEP_RETRY_MAX:-5', rosdep_retry)

    def test_xauthority_mount_uses_container_target(self):
        compose = (REPO / "docker-compose.yaml").read_text(encoding="utf-8")

        self.assertEqual(compose.count("- XAUTHORITY=/root/.Xauthority"), 2)


if __name__ == "__main__":
    unittest.main()
