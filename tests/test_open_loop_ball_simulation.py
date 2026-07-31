import importlib.util
import sys
import unittest
from pathlib import Path


MODULE_PATH = (
    Path(__file__).parents[1]
    / "simulation"
    / "open_loop_ball"
    / "open_loop_ball_simulation.py"
)
SPEC = importlib.util.spec_from_file_location("open_loop_ball_simulation", MODULE_PATH)
sim = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = sim
SPEC.loader.exec_module(sim)


class OpenLoopBallSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rows, cls.phases = sim.run_simulation()

    def test_default_profile_reaches_both_targets(self):
        self.assertAlmostEqual(self.phases[3]["position_cm"], 5.0, delta=0.25)
        self.assertAlmostEqual(self.phases[6]["position_cm"], -5.0, delta=0.25)
        self.assertLess(abs(self.phases[3]["velocity_cm_s"]), 0.2)
        self.assertLess(abs(self.phases[6]["velocity_cm_s"]), 0.2)

    def test_default_profile_obeys_safety_limits(self):
        positions = [row["position_cm"] for row in self.rows]
        self.assertGreater(min(positions), -9.0)
        self.assertLess(max(positions), 9.0)
        sim.validate_profile(sim.DEFAULT_ANGLES_DEG, sim.DEFAULT_TIMES_MS)

    def test_four_bar_neutral_is_level(self):
        self.assertAlmostEqual(sim.alpha_to_theta(sim.NEUTRAL_DEG), 0.0, places=6)

    def test_firmware_durations_are_tick_aligned(self):
        self.assertTrue(all(duration % 5 == 0 for duration in sim.DEFAULT_TIMES_MS))
        self.assertEqual(sum(sim.DEFAULT_TIMES_MS), 3085)


if __name__ == "__main__":
    unittest.main()
