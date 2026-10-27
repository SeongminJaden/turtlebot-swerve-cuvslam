import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import numpy as np
from pose_covariance import transport
class CovarianceTest(unittest.TestCase):
 def test_lever_arm_couples_angle_and_position(self):
  arm=np.eye(4);arm[0,3]=1
  c=transport(np.zeros((6,6)),np.eye(4),arm,position_std=.1,rotation_std=.2)
  self.assertAlmostEqual(c[1,1],.05);self.assertAlmostEqual(c[1,5],.04)
  self.assertGreater(np.linalg.eigvalsh(c).min(),0)
 def test_unknown_is_not_zero_uncertainty(self):
  c=transport(np.full((6,6),np.nan),np.eye(4),np.eye(4));self.assertTrue((np.diag(c)>0).all())
