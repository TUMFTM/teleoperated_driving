import importlib.util
import pathlib
import struct
import unittest


MODULE_PATH = (
    pathlib.Path(__file__).resolve().parents[1]
    / "src/tod_operator_interface/tod_input_devices/tools/set_g923_autocenter.py"
)
SPEC = importlib.util.spec_from_file_location("set_g923_autocenter", MODULE_PATH)
autocenter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(autocenter)


class AutocenterTests(unittest.TestCase):
    def test_converts_percent_to_evdev_value(self):
        self.assertEqual(autocenter.autocenter_value(30), 19660)

    def test_rejects_out_of_range_strength(self):
        for strength in (-1, 101):
            with self.subTest(strength=strength):
                with self.assertRaisesRegex(ValueError, "between 0 and 100"):
                    autocenter.autocenter_value(strength)

    def test_encodes_linux_autocenter_event(self):
        event = autocenter.encode_autocenter_event(30)
        _, _, event_type, event_code, value = struct.unpack("@llHHi", event)

        self.assertEqual(event_type, autocenter.EV_FF)
        self.assertEqual(event_code, autocenter.FF_AUTOCENTER)
        self.assertEqual(value, 19660)


if __name__ == "__main__":
    unittest.main()
