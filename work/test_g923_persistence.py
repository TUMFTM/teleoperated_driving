import pathlib
import unittest

import yaml


REPO = pathlib.Path(__file__).resolve().parents[1]
OVERRIDE = REPO / "docker-compose.override.yaml"
SETUP = REPO / "work" / "setup_g923_runtime.sh"
CONFIG = REPO / "work" / "logitechg923.yaml"

INPUT_CONFIG_DIR = (
    "/home/tum/wsp/install/tod_input_devices/share/tod_input_devices/config"
)


class G923PersistenceTests(unittest.TestCase):
    def test_compose_applies_autocenter_before_operator_without_blocking_startup(self):
        override = OVERRIDE.read_text(encoding="utf-8")
        setup = SETUP.read_text(encoding="utf-8")
        autocenter = yaml.safe_load(override)["services"]["g923_autocenter"]

        for config in (override, setup):
            self.assertIn("g923_autocenter:", config)
            self.assertIn("condition: service_completed_successfully", config)
            self.assertIn("G923_AUTOCENTER_STRENGTH:-80", config)
            self.assertIn("set_g923_autocenter.py", config)
            self.assertIn("Autocenter unavailable; continuing", config)
        self.assertIn('set_g923_autocenter.py "${G923_AUTOCENTER_STRENGTH:-80}"', setup)
        self.assertTrue(autocenter["privileged"])
        self.assertNotIn("devices", autocenter)
        self.assertTrue(
            any(
                volume.get("source") == "/dev/input"
                and volume.get("target") == "/dev/input"
                for volume in autocenter["volumes"]
            )
        )

    def test_compose_bind_mounts_device_and_both_config_names(self):
        override = OVERRIDE.read_text(encoding="utf-8")
        operator = yaml.safe_load(override)["services"]["tod_operator"]

        self.assertIn("volumes:", override)
        self.assertNotIn("devices", operator)
        self.assertIn('source: "/dev/input"', override)
        self.assertIn("target: /dev/input", override)
        self.assertNotIn("INPUT_DEVICE", override)
        self.assertEqual(override.count('source: "./work/logitechg923.yaml"'), 2)
        self.assertIn(f"target: {INPUT_CONFIG_DIR}/virtual.yaml", override)
        self.assertIn(f"target: {INPUT_CONFIG_DIR}/logitechg923.yaml", override)
        self.assertEqual(len(operator["volumes"]), 3)
        self.assertTrue(
            all(volume["bind"]["create_host_path"] is False for volume in operator["volumes"])
        )

    def test_setup_uses_persistent_mounts_instead_of_container_mutation(self):
        setup = SETUP.read_text(encoding="utf-8")

        self.assertIn("docker compose up -d tod_vehicle tod_operator", setup)
        self.assertNotIn("ln -s", setup)
        self.assertNotIn("cat > /home/tum/wsp/install/tod_input_devices", setup)
        self.assertNotIn("change_input_device", setup)

    def test_g923_config_maps_separate_throttle_and_brake_axes(self):
        config = CONFIG.read_text(encoding="utf-8")

        self.assertIn("Throttle: 2", config)
        self.assertIn("Brake: 3", config)
        self.assertIn("input_device_has_separate_braking_axis: true", config)


if __name__ == "__main__":
    unittest.main()
