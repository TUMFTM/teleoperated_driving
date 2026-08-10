import ast
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "work/peanut01_control_bridge.py"


class Peanut01ControlBridgeContractTest(unittest.TestCase):
    def test_bridge_is_valid_python(self):
        ast.parse(BRIDGE.read_text(encoding="utf-8"))

    def test_bridge_uses_required_domains_and_topics(self):
        text = BRIDGE.read_text(encoding="utf-8")
        topics = (
            "/vehicle/safety/output/primary_control_cmd",
            "/vehicle/safety/output/secondary_control_cmd",
            "/vehicle/statemachine/output/vehicle_status",
            "/vehicle/status/velocity_status",
            "/vehicle/status/steering_status",
            "/vehicle/status/control_mode",
            "/minguo/emergency_stop",
            "/minguo/teleop_override",
            "/debug/tod_peanut01/autoware_control_cmd",
            "/debug/tod_peanut01/autoware_gear_cmd",
            "/debug/tod_peanut01/autoware_turn_indicators_cmd",
            "/debug/tod_peanut01/autoware_hazard_lights_cmd",
            "/debug/tod_peanut01/bridge_diagnostics",
            "/control/command/control_cmd",
            "/control/command/gear_cmd",
            "/control/command/turn_indicators_cmd",
            "/control/command/hazard_lights_cmd",
            "/control/control_mode_request",
        )
        for topic in topics:
            with self.subTest(topic=topic):
                self.assertIn(topic, text)
        self.assertIn("source_domain_id", text)
        self.assertIn("target_domain_id", text)
        self.assertGreaterEqual(text.count("Context()"), 2)

    def test_real_publishers_are_dynamic_and_actuation_starts_disabled(self):
        text = BRIDGE.read_text(encoding="utf-8")

        self.assertIn("create_real_publishers", text)
        self.assertIn("destroy_real_publishers", text)
        self.assertIn("destroy_publisher", text)
        self.assertIn("add_on_set_parameters_callback", text)
        self.assertIn('declare_parameter("enable_actuation", False)', text)
        self.assertIn("configured_enable", text)

    def test_bridge_uses_generated_constants_and_never_accesses_can(self):
        text = BRIDGE.read_text(encoding="utf-8")

        self.assertIn("Status.TOD_STATUS_TELEOPERATION", text)
        self.assertIn("ControlModeCommand.Request.AUTONOMOUS", text)
        self.assertIn("ControlModeCommand.Request.MANUAL", text)
        self.assertNotIn("socketcan", text.lower())
        self.assertNotIn("python-can", text.lower())
        self.assertNotIn("can_mingnuo", text.lower())


if __name__ == "__main__":
    unittest.main()
