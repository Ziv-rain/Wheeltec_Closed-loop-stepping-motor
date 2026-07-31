import importlib.util
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "tools" / "mechanical_motion_sim.py"
SPEC = importlib.util.spec_from_file_location("mechanical_motion_sim", MODULE_PATH)
sim = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = sim
SPEC.loader.exec_module(sim)


class MechanicalMotionSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = sim.Model()
        cls.profile, cls.waypoints = sim.fit_profile(
            cls.model, [5.0, -5.0], trials=700
        )
        cls.final, cls.samples = sim.simulate_steps(
            cls.model, cls.profile, record=True
        )

    def test_reaches_both_waypoints(self):
        self.assertAlmostEqual(self.waypoints[0].position_m * 100.0, 5.0, delta=0.35)
        self.assertAlmostEqual(self.waypoints[1].position_m * 100.0, -5.0, delta=0.35)
        self.assertLess(abs(self.waypoints[0].velocity_mps * 100.0), 1.5)
        self.assertLess(abs(self.waypoints[1].velocity_mps * 100.0), 1.5)

    def test_respects_motor_angle_limits(self):
        for step in self.profile:
            self.assertGreaterEqual(step.angle_deg, self.model.min_angle_deg)
            self.assertLessEqual(step.angle_deg, self.model.max_angle_deg)
        for sample in self.samples:
            self.assertGreaterEqual(sample.motor_angle_deg, self.model.min_angle_deg)
            self.assertLessEqual(sample.motor_angle_deg, self.model.max_angle_deg)

    def test_profile_is_deterministic(self):
        second, _ = sim.fit_profile(self.model, [5.0, -5.0], trials=700)
        self.assertEqual(self.profile, second)


if __name__ == "__main__":
    unittest.main()
