import math
from pathlib import Path
import unittest
from turtlebot_swerve.kinematics import load_modules, solve, wheel_vector

ROOT = Path(__file__).resolve().parents[1]


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.modules = load_modules(ROOT/'assets/m38_arm_description/urdf/m38_arm.urdf')

    def test_body_twists_reconstruct_at_every_contact(self):
        for vx, vy, wz in [(0.1, 0, 0), (0, .1, 0), (-.1, 0, 0),
                           (0, 0, .5), (.05, -.08, -.3)]:
            for m in self.modules:
                t = solve(self.modules, vx, vy, wz)[m.name]
                ux, uy = wheel_vector(m, t)
                self.assertAlmostEqual(ux, vx - wz*m.y)
                self.assertAlmostEqual(uy, vy + wz*m.x)
                self.assertLessEqual(abs(t.steering), math.pi/2)

    def test_axes_are_not_assumed_identical(self):
        result = solve(self.modules, .1, 0, 0)
        self.assertAlmostEqual(result['lh1'].steering, math.pi/2)
        self.assertAlmostEqual(result['lh2'].steering, 0)
        self.assertLess(result['rh2'].velocity, 0)

    def test_saturation_keeps_common_scale(self):
        ratios = []
        for m in self.modules:
            t = solve(self.modules, 2, 1, 3)[m.name]
            self.assertLessEqual(abs(t.velocity), m.max_speed)
            ux, uy = wheel_vector(m, t)
            ratios += [ux/(2-3*m.y), uy/(1+3*m.x)]
        for ratio in ratios:
            self.assertAlmostEqual(ratio, ratios[0])

    def test_stop_keeps_steering(self):
        previous = {m.name: .4 for m in self.modules}
        for t in solve(self.modules, 0, 0, 0, previous).values():
            self.assertEqual((t.steering, t.velocity), (.4, 0))

    def test_nonfinite_rejected(self):
        for value in [float('nan'), float('inf'), -float('inf'), 1e308]:
            with self.assertRaises(ValueError):
                solve(self.modules, value, 0, 0)


if __name__ == '__main__':
    unittest.main()
