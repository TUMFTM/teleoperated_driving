import math
import pathlib
import runpy
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MAPPING = ROOT / "work/peanut01_control_mapping.py"


class Peanut01ControlMappingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mapping = runpy.run_path(str(MAPPING))

    def test_drive_and_reverse_use_signed_autoware_velocity(self):
        convert = self.mapping["convert_command"]

        self.assertEqual(0.8, convert(0.8, 1.6, 3, 0, 16.0).velocity_mps)
        self.assertEqual(-0.8, convert(0.8, -1.6, 1, 0, 16.0).velocity_mps)

    def test_reverse_steering_is_not_inverted_twice(self):
        command = self.mapping["convert_command"](0.5, -1.6, 1, 0, 16.0)

        self.assertEqual(-0.1, command.steering_tire_angle_rad)

    def test_park_neutral_and_unknown_gear_stop(self):
        convert = self.mapping["convert_command"]

        self.assertEqual(0.0, convert(0.8, 0.0, 0, 0, 16.0).velocity_mps)
        self.assertEqual(0.0, convert(0.8, 0.0, 2, 0, 16.0).velocity_mps)
        with self.assertRaises(ValueError):
            convert(0.8, 0.0, 99, 0, 16.0)

    def test_indicator_mapping_is_mutually_exclusive(self):
        convert = self.mapping["convert_command"]

        left = convert(0.0, 0.0, 0, 1, 16.0)
        right = convert(0.0, 0.0, 0, 2, 16.0)
        both = convert(0.0, 0.0, 0, 3, 16.0)
        self.assertEqual((2, 1), (left.turn, left.hazard))
        self.assertEqual((3, 1), (right.turn, right.hazard))
        self.assertEqual((1, 2), (both.turn, both.hazard))

    def test_invalid_values_fail_closed(self):
        convert = self.mapping["convert_command"]

        invalid_commands = (
            (math.nan, 0.0, 3, 0, 16.0),
            (0.0, math.inf, 3, 0, 16.0),
            (-0.1, 0.0, 3, 0, 16.0),
            (0.0, 0.0, 3, 0, 0.0),
            (0.0, 0.0, 3, 99, 16.0),
        )
        for command in invalid_commands:
            with self.subTest(command=command), self.assertRaises(ValueError):
                convert(*command)


if __name__ == "__main__":
    unittest.main()
