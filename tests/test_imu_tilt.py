import unittest
import numpy as np
from turtlebot_swerve.imu_tilt import ImuTilt
class TiltTest(unittest.TestCase):
 def test_rest_pitch(self):
  f=ImuTilt();theta=.2;q=f.update(1.,[0,0,0],[-9.80665*np.sin(theta),0,9.80665*np.cos(theta)])
  np.testing.assert_allclose(q,[0,np.sin(theta/2),0,np.cos(theta/2)],atol=1e-9)
 def test_gap_invalidates(self):
  f=ImuTilt();f.update(1.,[0,0,0],[0,0,9.80665]);self.assertIsNone(f.update(2.,[0,0,0],[0,0,9.80665]))
