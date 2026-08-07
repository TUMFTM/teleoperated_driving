import pathlib
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/tod_vehicle_interface/tod_peanut01_interface"
PACKAGE_LAUNCH = PACKAGE / "launch/tod_peanut01_interface.launch.py"
PROFILE = ROOT / "config/config/launch_setup_peanut01_control_only.yaml"
REMAPPINGS = ROOT / "config/config/remappings.yaml"
VEHICLE_PARAMS = ROOT / "config/config/vehicle_config/peanut01/vehicle-params.yaml"
COMPOSE = ROOT / "docker-compose.peanut01-dry-run.yaml"


class Peanut01DryRunDeploymentTest(unittest.TestCase):
    def test_interface_contains_only_debug_output(self):
        sources = "\n".join(
            path.read_text(encoding="utf-8")
            for path in PACKAGE.rglob("*")
            if path.suffix in {".cpp", ".hpp", ".py", ".yaml"}
        )

        self.assertIn("/debug/tod_peanut01/control_cmd", sources)
        for forbidden in (
            "/control/command/control_cmd",
            "/cmd_vel",
            "/minguo/teleop_override",
            "socketcan",
            "can0",
        ):
            self.assertNotIn(forbidden, sources.lower())

    def test_control_only_profile_disables_sensor_and_guidance_packages(self):
        profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))

        self.assertEqual("peanut01", profile["launch_parameters"]["vehicleID"])
        self.assertEqual("vehicle", profile["launch_parameters"]["mode"])
        packages = profile["packages_to_launch"]
        disabled = {
            "tod_rtsp",
            "tod_lidar",
            "tod_projection",
            "tod_pure_pursuit",
            "tod_trajectory_guidance",
            "tod_transform",
        }
        for side in packages.values():
            for package in disabled.intersection(side):
                self.assertFalse(side[package], package)

    def test_interface_input_is_remapped_from_safety_gate(self):
        remappings = yaml.safe_load(REMAPPINGS.read_text(encoding="utf-8"))
        interface_remappings = remappings["tod_peanut01_interface"]

        self.assertIn(
            {
                "from": "/vehicle/interface/peanut01/input/primary_control_cmd",
                "to": "/vehicle/safety/output/primary_control_cmd",
            },
            interface_remappings,
        )

    def test_vehicle_parameters_are_explicitly_dry_run_only(self):
        text = VEHICLE_PARAMS.read_text(encoding="utf-8")
        params = yaml.safe_load(text)

        self.assertIn("dry-run only", text.lower())
        self.assertAlmostEqual(0.8, params["distance_front_axle"] + params["distance_rear_axle"])
        self.assertAlmostEqual(1.047, params["maximum_road_wheel_angle"])
        self.assertAlmostEqual(6.108652, params["maximum_steering_wheel_angle"])
        for uncalibrated in (
            "mass",
            "yaw_inertia",
            "cornering_force_front",
            "cornering_force_rear",
        ):
            self.assertEqual(0.0, params[uncalibrated])

    def test_compose_uses_baked_control_only_launchers(self):
        compose = yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))["services"]

        vehicle_command = " ".join(compose["tod_vehicle"]["command"])
        operator_command = " ".join(compose["tod_operator"]["command"])
        self.assertIn("tod_vehicle_control_only.launch.py", vehicle_command)
        self.assertNotIn("overlay", vehicle_command)
        self.assertIn("tod_operator_control_only.launch.py", operator_command)

    def test_interface_launch_has_shared_config_default(self):
        launch = PACKAGE_LAUNCH.read_text(encoding="utf-8")

        self.assertIn('get_package_share_directory("tod_launch")', launch)
        self.assertIn('default_value=default_config_path', launch)


if __name__ == "__main__":
    unittest.main()
