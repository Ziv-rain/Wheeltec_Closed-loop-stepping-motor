import re
import unittest
from pathlib import Path


HEADER = Path(__file__).parents[1] / "Control" / "accel_history_profile.h"


class AccelHistoryProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = HEADER.read_text(encoding="utf-8")

    def macro_int(self, name: str) -> int:
        match = re.search(rf"^#define\s+{name}\s+(\d+)U", self.source, re.M)
        self.assertIsNotNone(match, name)
        return int(match.group(1))

    def profile_values(self) -> list[int]:
        match = re.search(
            r"s_task5_future50_milli_mps2\[.*?\]\s*=\s*\{(.*?)\};",
            self.source,
            re.S,
        )
        self.assertIsNotNone(match)
        return [int(value) for value in re.findall(r"-?\d+", match.group(1))]

    def test_conservative_weights_are_normalized(self):
        live = self.macro_int("ACCEL_HISTORY_REAL_PERCENT")
        history = self.macro_int("ACCEL_HISTORY_HISTORY_PERCENT")
        self.assertEqual((live, history), (70, 30))
        self.assertEqual(live + history, 100)

    def test_profile_shape_and_physical_range(self):
        values = self.profile_values()
        self.assertEqual(len(values), self.macro_int("ACCEL_HISTORY_TASK5_COUNT"))
        self.assertEqual(len(values), 507)
        self.assertGreaterEqual(min(values), -500)
        self.assertLessEqual(max(values), 500)
        self.assertEqual(values[:15], [0] * 15)
        self.assertEqual(values[-7:], [0] * 7)

    def test_expected_duration_and_preview_horizon(self):
        count = self.macro_int("ACCEL_HISTORY_TASK5_COUNT")
        sample_ms = self.macro_int("ACCEL_HISTORY_SAMPLE_MS")
        self.assertEqual(sample_ms, 50)
        self.assertEqual((count - 1) * sample_ms, 25_300)
        self.assertIn("future50", self.source)


if __name__ == "__main__":
    unittest.main()
