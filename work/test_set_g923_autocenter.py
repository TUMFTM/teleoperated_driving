import importlib.util
import pathlib
import struct
import unittest


MODULE_PATH = pathlib.Path(__file__).with_name("set_g923_autocenter.py")
SPEC = importlib.util.spec_from_file_location("set_g923_autocenter", MODULE_PATH)
autocenter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(autocenter)


class AutocenterValueTests(unittest.TestCase):
    def test_converts_30_percent_to_evdev_value(self):
        self.assertEqual(autocenter.autocenter_value(30), 19660)

    def test_rejects_strength_outside_supported_range(self):
        for strength in (-1, 101):
            with self.subTest(strength=strength):
                with self.assertRaisesRegex(ValueError, "between 0 and 100"):
                    autocenter.autocenter_value(strength)


class AutocenterEventTests(unittest.TestCase):
    def test_encodes_linux_autocenter_event(self):
        event = autocenter.encode_autocenter_event(30)
        _, _, event_type, event_code, value = struct.unpack("@llHHi", event)

        self.assertEqual(event_type, autocenter.EV_FF)
        self.assertEqual(event_code, autocenter.FF_AUTOCENTER)
        self.assertEqual(value, 19660)

    def test_accepts_only_logitech_g923_device_names(self):
        self.assertTrue(autocenter.is_g923("Logitech G923 Racing Wheel for PlayStation and PC"))
        self.assertFalse(autocenter.is_g923("Logitech G29 Driving Force Racing Wheel"))
        self.assertFalse(autocenter.is_g923("VMware Virtual USB Mouse"))


if __name__ == "__main__":
    unittest.main()
