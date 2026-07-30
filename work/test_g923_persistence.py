import pathlib
import unittest


REPO = pathlib.Path(__file__).resolve().parents[1]
OVERRIDE = REPO / "docker-compose.override.yaml"
SETUP = REPO / "work" / "setup_g923_runtime.sh"
CONFIG = REPO / "work" / "logitechg923.yaml"

INPUT_CONFIG_DIR = (
    "/home/tum/wsp/install/tod_input_devices/share/tod_input_devices/config"
)


class G923PersistenceTests(unittest.TestCase):
    def test_compose_bind_mounts_device_and_both_config_names(self):
        override = OVERRIDE.read_text(encoding="utf-8")

        self.assertIn("volumes:", override)
        self.assertNotIn("devices:", override)
        self.assertIn('source: "${INPUT_DEVICE:-/dev/input/js0}"', override)
        self.assertIn("target: /dev/input/js0", override)
        self.assertEqual(override.count('source: "./work/logitechg923.yaml"'), 2)
        self.assertIn(f"target: {INPUT_CONFIG_DIR}/virtual.yaml", override)
        self.assertIn(f"target: {INPUT_CONFIG_DIR}/logitechg923.yaml", override)
        self.assertEqual(override.count("create_host_path: false"), 3)

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
