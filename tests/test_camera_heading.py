import math,unittest
from rl.camera_heading import target_for_heading

class HeadingTest(unittest.TestCase):
 def test_rear_branch_no_half_turn_flip(self):
  previous=3.
  for yaw in [-.05,-.15,-.08,.02,-.1]:
   target=target_for_heading(3.05,yaw,previous)
   self.assertGreater(target,2.9);previous=target
 def test_side_request_changes_branch(self):
  self.assertAlmostEqual(target_for_heading(-math.pi/2,0,3.),-math.pi/2)
 def test_unreachable_is_clipped(self):
  self.assertLessEqual(abs(target_for_heading(math.pi,0,3.)),3.05)
