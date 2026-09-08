import ast
import pathlib
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "work/peanut01_control_bridge.py"
PARAMS = ROOT / "config/config/package_config/tod_peanut01_interface/params.yaml"
LAUNCH = ROOT / "src/tod_launch/launch/tod_vehicle_peanut01_video.launch.py"
DOCKERFILE = ROOT / "docker/dockerfile"


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
            "/vehicle/can/raw",
            "/vehicle/interface/actuation/from_actuation/safety_driver_status",
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

    def test_bridge_projects_read_only_can_feedback_into_tod_safety_status(self):
        text = BRIDGE.read_text(encoding="utf-8")

        for token in (
            "from peanut01_can_feedback import CanFeedbackTracker",
            "from std_msgs.msg import Bool, String",
            "SafetyDriverStatus",
            'CAN_FEEDBACK_TOPIC = "/vehicle/can/raw"',
            'SAFETY_STATUS_TOPIC = "/vehicle/interface/actuation/from_actuation/safety_driver_status"',
            "self.target_node.create_subscription(",
            "String, self.can_feedback_topic, self.on_can_feedback, 500",
            "self.source_node.create_publisher(",
            "SafetyDriverStatus, SAFETY_STATUS_TOPIC, 10",
            "vehicle_emergency_stop_released",
            "vehicle_long_approved",
            "vehicle_lat_approved",
        ):
            with self.subTest(token=token):
                self.assertIn(token, text)

    def test_bridge_declares_can_and_execution_parameters(self):
        text = BRIDGE.read_text(encoding="utf-8")

        for parameter in (
            "can_feedback_topic",
            "can_feedback_timeout_ms",
            "execution_confirmation_timeout_ms",
        ):
            with self.subTest(parameter=parameter):
                self.assertIn(f'declare_parameter("{parameter}"', text)

    def test_bridge_diagnostics_expose_safety_and_execution_state(self):
        text = BRIDGE.read_text(encoding="utf-8")

        for key in (
            "emergency_released",
            "lateral_approved",
            "longitudinal_approved",
            "mcu_power_up",
            "mcu_enabled",
            "mcu_direction",
            "mcu_gear",
            "mcu_brake_locked",
            "mcu_error_codes",
            "mcu_stat1_age_ms",
            "mcu_stat2_age_ms",
            "mcu_error_age_ms",
            "software_neutral",
            "command_enabled",
            "command_mode",
            "command_gear",
            "command_brake_mode",
            "command_motor_rpm",
            "mcu_command_age_ms",
            "eps_mode",
            "eps_init_status",
            "eps_error_1",
            "eps_error_2",
            "eps_status1_age_ms",
            "execution_expected",
            "execution_remaining_ms",
            "f710_override",
        ):
            with self.subTest(key=key):
                self.assertIn(f'key="{key}"', text)
        self.assertIn('"software_neutral": feedback.software_neutral', text)

    def test_fault_shutdown_is_latched_and_destroys_real_publishers(self):
        text = BRIDGE.read_text(encoding="utf-8")

        self.assertIn("self._fault_shutdown_pending = False", text)
        self.assertIn("decision.state is State.FAULT", text)
        self.assertIn("not self._fault_shutdown_pending", text)
        self.assertIn("self._fault_shutdown_pending = True", text)
        self.assertIn("self._start_deactivation()", text)
        self.assertIn("self._request_manual_and_destroy()", text)

    def test_shared_parameters_default_actuation_to_disabled(self):
        params = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
        node = params["/vehicle/interface/peanut01/ControlBridge"][
            "ros__parameters"
        ]

        self.assertFalse(node["enable_actuation"])
        self.assertEqual(7, node["source_domain_id"])
        self.assertEqual(0, node["target_domain_id"])
        self.assertEqual(300, node["command_timeout_ms"])
        self.assertEqual(300, node["feedback_timeout_ms"])
        self.assertEqual("/vehicle/can/raw", node["can_feedback_topic"])
        self.assertEqual(300, node["can_feedback_timeout_ms"])
        self.assertEqual(1000, node["execution_confirmation_timeout_ms"])
        self.assertEqual(1000, node["arming_duration_ms"])
        self.assertEqual(0.02, node["stopped_velocity_mps"])
        self.assertEqual(16.0, node["steering_ratio"])
        self.assertEqual(20.0, node["publish_rate_hz"])
        self.assertNotIn("max_velocity", node)

    def test_vehicle_launch_starts_bridge_with_shared_parameters(self):
        text = LAUNCH.read_text(encoding="utf-8")

        self.assertIn("/opt/tod-tools/peanut01_control_bridge.py", text)
        self.assertIn("tod_peanut01_interface", text)
        self.assertIn('"--params-file"', text)

    def test_vehicle_image_contains_all_control_bridge_modules(self):
        text = DOCKERFILE.read_text(encoding="utf-8")

        for module in (
            "peanut01_can_feedback.py",
            "peanut01_control_mapping.py",
            "peanut01_control_supervisor.py",
            "peanut01_control_bridge.py",
        ):
            with self.subTest(module=module):
                self.assertIn(module, text)

    def test_vehicle_builder_requires_installed_safety_gate(self):
        text = DOCKERFILE.read_text(encoding="utf-8")

        self.assertIn("--packages-select tod_safety_gate", text)
        self.assertIn(
            "install/tod_safety_gate/lib/tod_safety_gate/safety_gate", text
        )


if __name__ == "__main__":
    unittest.main()
