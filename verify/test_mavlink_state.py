import struct
import unittest

from scripts.mavlink import PX4_AUTO_MODE, heartbeat


class MissionPhaseState(unittest.TestCase):
    def test_heartbeat_exposes_mode_and_mission_phase_from_one_decode(self):
        custom_mode = (4 << 24) | (PX4_AUTO_MODE << 16)
        payload = struct.pack("<IBBBBB", custom_mode, 2, 3, 0, 4, 3)

        result = heartbeat(payload)

        self.assertEqual(result["mode"], "AUTO.MISSION")
        self.assertEqual(result["mission_phase"], result["mode"])
        self.assertFalse(result["armed"])


if __name__ == "__main__":
    unittest.main()
