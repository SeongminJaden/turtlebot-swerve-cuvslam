import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pose_quality import PoseQuality
class PoseQualityTest(unittest.TestCase):
 def test_small_motion_then_jump_latches(self):
  g=PoseQuality();self.assertTrue(g.update([0,0,0,0,0,0,1],1000000000));self.assertTrue(g.update([.01,0,0,0,0,0,1],1066666667));self.assertFalse(g.update([1.,0,0,0,0,0,1],1133333333));self.assertFalse(g.update([.02,0,0,0,0,0,1],1200000000))
 def test_missing_interval_allows_only_bounded_motion(self):
  g=PoseQuality();self.assertTrue(g.update([0,0,0,0,0,0,1],1000000000));self.assertTrue(g.update([.015,0,0,0,0,0,1],1266666667))

 def test_realistic_motion_not_rejected_by_old_demo_limits(self):
  import math
  g=PoseQuality();g.update([0,0,0,0,0,0,1],1000000000)
  angle=math.radians(7)
  self.assertTrue(g.update([.05,0,0,0,0,math.sin(angle/2),math.cos(angle/2)],1065000000))
