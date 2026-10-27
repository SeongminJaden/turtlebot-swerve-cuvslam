import math
import unittest
from rl.active_view import ViewGate, reward_terms


class ViewGateTest(unittest.TestCase):
    def setUp(self):
        self.gate = ViewGate()
        self.inputs = dict(dt=.1, desired_heading=0., alignment_error=0.,
                           tracking_ok=True, sample_age=.03, reachable=True,
                           base_speed=0., base_yaw_rate=0., arm_speed=0.)

    def step(self, **changes):
        return self.gate.step(**(self.inputs | changes))

    def ready(self):
        for _ in range(4): self.assertFalse(self.step())
        self.assertTrue(self.step())

    def test_dwell_and_tracking_loss(self):
        self.ready()
        self.assertFalse(self.step(tracking_ok=False, base_speed=.1))
        self.assertEqual(self.gate.state, 'STOP')
        self.assertFalse(self.step(base_speed=.1))

    def test_reverse_requires_new_verification(self):
        self.ready()
        self.assertFalse(self.step(desired_heading=math.pi, base_speed=.1))
        self.assertEqual(self.gate.state, 'STOP')

    def test_invalid_and_stale_samples_revoke_drive(self):
        for invalid in [dict(sample_age=.2), dict(sample_age=-.1),
                        dict(alignment_error=float('nan')), dict(dt=.5),
                        dict(reachable=False)]:
            self.gate.reset(); self.ready()
            self.assertFalse(self.step(**invalid))

    def test_settle_and_hysteresis(self):
        for _ in range(8): self.assertFalse(self.step(arm_speed=.2))
        self.ready()
        self.assertTrue(self.step(alignment_error=math.radians(20), base_speed=.1))
        self.assertFalse(self.step(alignment_error=math.radians(26)))

    def test_one_bad_sample_restarts_dwell(self):
        for _ in range(3): self.step()
        self.step(tracking_ok=False)
        self.ready()

    def test_yaw_motion_is_not_stopped(self):
        for _ in range(8): self.assertFalse(self.step(base_yaw_rate=.2))

    def test_missing_tracker_allows_looking_but_never_driving(self):
        for _ in range(10):
            self.assertFalse(self.step(sample_age=100., tracking_ok=False))
            self.assertEqual(self.gate.state, 'LOOK')

    def test_arm_motion_revokes_drive(self):
        self.ready()
        self.assertFalse(self.step(arm_speed=.2))

    def test_stationary_alignment_has_no_recurring_bonus(self):
        r = reward_terms(dt=.1, progress=0., angle_error=0.,
                         previous_angle_error=0., drive_allowed=True,
                         base_speed=0., arm_speed_squared=0.)
        self.assertLess(sum(r.values()), 0.)

    def test_blind_progress_is_not_paid(self):
        r = reward_terms(dt=.1, progress=.1, angle_error=1.,
                         previous_angle_error=1., drive_allowed=False,
                         base_speed=.1, arm_speed_squared=0.)
        self.assertEqual(r['progress'], 0.)
        self.assertLess(r['blind_motion'], 0.)


if __name__ == '__main__': unittest.main()
