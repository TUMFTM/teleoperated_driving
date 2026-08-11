import pathlib
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEPLOYED_PARAMS = ROOT / "config/config/package_config/tod_command_creation/params.yaml"
G923_PARAMS = ROOT / "src/tod_operator_interface/tod_input_devices/config/logitechg923.yaml"
COMMAND_CREATOR = ROOT / "src/tod_direct_control/tod_command_creation/src/command_creator.cpp"


class G923GearConfigTest(unittest.TestCase):
    def test_peanut01_uses_rnd_with_neutral_default(self):
        config = yaml.safe_load(DEPLOYED_PARAMS.read_text(encoding="utf-8"))
        params = config["/operator/direct_control/CommandCreator"]["ros__parameters"]
        self.assertEqual(1, params["minGearPosition"])
        self.assertEqual(3, params["maxGearPosition"])
        self.assertEqual(2, params["defaultGearPosition"])
        self.assertEqual(0.8, params["maxVelocity"])

    def test_g923_paddles_keep_the_existing_button_mapping(self):
        config = yaml.safe_load(G923_PARAMS.read_text(encoding="utf-8"))
        buttons = config["/**"]["ros__parameters"]["button_config"]
        self.assertEqual(4, buttons["IncreaseGear"])
        self.assertEqual(5, buttons["DecreaseGear"])

    def test_command_creator_initializes_and_uses_the_selector(self):
        source = COMMAND_CREATOR.read_text(encoding="utf-8")
        for parameter in ("minGearPosition", "maxGearPosition", "defaultGearPosition"):
            self.assertIn(f'declare_parameter<int>("{parameter}"', source)
        self.assertIn("_gearSelector->default_gear()", source)
        self.assertIn("_gearSelector->select(", source)


if __name__ == "__main__":
    unittest.main()
