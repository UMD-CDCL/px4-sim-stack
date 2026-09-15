import math
import pathlib
import sys

import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from gimbal_rangefinder import range_reading


class RangeReadingTest(unittest.TestCase):
    def test_accepts_sensor_interval(self):
        for measured in (0.2, 7.5, 50.0):
            with self.subTest(measured=measured):
                self.assertEqual(range_reading(measured, 0.2, 50.0),
                                 (measured, True))


    def test_marks_out_of_bounds_or_missing_invalid(self):
        for measured in (0.0, -1.0, 51.0, math.inf, math.nan):
            with self.subTest(measured=measured):
                self.assertEqual(range_reading(measured, 0.2, 50.0),
                                 (50.0, False))


if __name__ == "__main__":
    unittest.main()
