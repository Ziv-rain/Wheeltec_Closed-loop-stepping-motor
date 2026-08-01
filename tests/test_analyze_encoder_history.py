import importlib.util
import math
import sys
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "tools" / "analyze_encoder_history.py"
SPEC = importlib.util.spec_from_file_location("analyze_encoder_history", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class HistoryAnalysisTests(unittest.TestCase):
    def test_parse_join_and_future_profile(self):
        lines = [
            "noise from terminal",
            "BALLLOG_BEGIN,1,4,5,3,0",
            "BALLLOG,4,0,100,100,200,300,400,500,600,550,15,5",
            "BALLLOG,4,1,150,110,210,310,410,510,610,560,15,5",
            "BALLLOG,4,2,200,120,220,320,420,520,620,570,15,5",
            "BALLLOG_END,1,4,5,3,0",
            "CARLOG_BEGIN,1,9,5,3,0,0",
            "CARLOG,9,0,100,10,10,40,40,40,24",
            "CARLOG,9,1,150,20,20,45,45,40,24",
            "CARLOG,9,2,200,30,30,50,50,40,24",
            "CARLOG_END,1,9,5,3,0,0",
        ]
        cars, balls = MODULE.parse_log(lines)
        rows = MODULE.build_rows(cars, balls)
        profile = MODULE.build_profile(rows)

        self.assertEqual(len(cars), 3)
        self.assertEqual(len(balls), 3)
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0]["ball_cm"], 1.0)
        self.assertEqual(rows[1]["elapsed_ms"], 50)
        self.assertTrue(
            math.isclose(
                rows[2]["distance_left_m"], 60 * MODULE.WHEEL_METERS_PER_COUNT
            )
        )
        self.assertFalse(math.isnan(float(rows[0]["future_accel_100ms_mps2"])))
        self.assertEqual(profile[0]["task"], 5)
        self.assertEqual(profile[0]["run_count"], 1)

    def test_bad_and_truncated_lines_are_ignored(self):
        cars, balls = MODULE.parse_log(
            ["CARLOG_BEGIN,1,2,4,1,0,0", "CARLOG,broken", "BALLLOG,1,2"]
        )
        self.assertEqual(cars, [])
        self.assertEqual(balls, [])


if __name__ == "__main__":
    unittest.main()
