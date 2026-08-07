import pathlib
import runpy
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
MAPPING = ROOT / "work/peanut01_vehicle_state_mapping.py"
BRIDGE = ROOT / "work/peanut01_vehicle_state_bridge.py"
VEHICLE_LAUNCH = ROOT / "src/tod_launch/launch/tod_vehicle_peanut01_video.launch.py"
COMPOSE = ROOT / "docker-compose.peanut01-video.yaml"
VISUAL_NODE = ROOT / "src/tod_operator_interface/tod_visual/src/tod_applications/visual/visual_node.cpp"
DRIVE_INFO = ROOT / "src/tod_operator_interface/tod_visual/src/tod_applications/visual/application_layer/include/drive_info_layer.hpp"
DOCKERFILE = ROOT / "docker/dockerfile"


class Peanut01VehicleStateTest(unittest.TestCase):
    def test_autoware_gear_reports_map_to_tod_gears(self):
        self.assertTrue(MAPPING.exists())
        mapping = runpy.run_path(str(MAPPING))
        map_gear = mapping["map_gear"]

        self.assertEqual(0, map_gear(22))
        self.assertEqual(1, map_gear(20))
        self.assertEqual(2, map_gear(1))
        self.assertEqual(3, map_gear(2))
        self.assertEqual(3, map_gear(23))

    def test_turn_indicators_and_hazards_map_to_tod_indicators(self):
        self.assertTrue(MAPPING.exists())
        mapping = runpy.run_path(str(MAPPING))
        map_indicator = mapping["map_indicator"]

        self.assertEqual(0, map_indicator(1, 1))
        self.assertEqual(1, map_indicator(2, 1))
        self.assertEqual(2, map_indicator(3, 1))
        self.assertEqual(3, map_indicator(1, 2))

    def test_bridge_reads_autoware_status_and_publishes_tod_state(self):
        self.assertTrue(BRIDGE.exists())
        bridge = BRIDGE.read_text(encoding="utf-8")

        for topic in (
            "/vehicle/status/velocity_status",
            "/vehicle/status/steering_status",
            "/vehicle/status/gear_status",
            "/vehicle/status/turn_indicators_status",
            "/vehicle/status/hazard_lights_status",
        ):
            self.assertIn(topic, bridge)
        self.assertIn("PrimaryVehicleState", bridge)
        self.assertIn("SecondaryVehicleState", bridge)
        self.assertIn('TOD_VEHICLE_STATE_SENSOR_DOMAIN_ID", "0"', bridge)

    def test_vehicle_launch_and_compose_start_state_bridge(self):
        launch = VEHICLE_LAUNCH.read_text(encoding="utf-8")
        vehicle = yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))["services"]["tod_vehicle"]
        dockerfile = DOCKERFILE.read_text(encoding="utf-8")

        self.assertIn("peanut01_vehicle_state_bridge.py", launch)
        self.assertIn("work/peanut01_vehicle_state_bridge.py", dockerfile)
        self.assertIn("work/peanut01_vehicle_state_mapping.py", dockerfile)
        self.assertNotIn("volumes", vehicle)
        self.assertEqual(
            "${TOD_VEHICLE_STATE_SENSOR_DOMAIN_ID:-0}",
            vehicle["environment"]["TOD_VEHICLE_STATE_SENSOR_DOMAIN_ID"],
        )

    def test_visual_uses_commanded_speed_and_gear_for_main_display(self):
        visual_node = VISUAL_NODE.read_text(encoding="utf-8")
        drive_info = DRIVE_INFO.read_text(encoding="utf-8")

        self.assertIn("tod_gl::PrimaryControlCommandComponent", visual_node)
        self.assertIn("class PrimaryControlComp", drive_info)
        self.assertIn("PrimaryControlComp &comp", drive_info)
        self.assertIn("speed_ = 3.6f * comp.get_velocity()", drive_info)
        self.assertIn("gearDisplay_ = comp.get_gear_position_string()", drive_info)
        self.assertNotIn("desired_speed_ = 3.6f * comp.get_velocity()", drive_info)
        self.assertNotIn("target_gear_display", drive_info)


if __name__ == "__main__":
    unittest.main()
