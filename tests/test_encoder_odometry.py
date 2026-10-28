import unittest
import numpy as np
from turtlebot_swerve.encoder_odometry import body_twist,MODULES,POSITIONS,OFFSETS
class EncoderTest(unittest.TestCase):
 def test_translation_and_rotation(self):
  for expected in ([.4,-.2,0.],[0.,0.,.8],[.2,.3,-.5]):
   names=[];q=[];v=[]
   for n,(x,y),offset in zip(MODULES,POSITIONS,OFFSETS):
    vx=expected[0]-expected[2]*y;vy=expected[1]+expected[2]*x
    names.extend([n+'_steer_joint',n+'_wheel_joint']);q.extend([np.arctan2(vy,vx)-offset,0]);v.extend([0,np.hypot(vx,vy)/.03068719889482])
   result,residual=body_twist(names,q,v);np.testing.assert_allclose(result,expected,atol=1e-12);self.assertLess(residual,1e-12)
 def test_nonfinite_rejected(self):
  names=[n+s for n in MODULES for s in ('_steer_joint','_wheel_joint')]
  with self.assertRaises(ValueError):body_twist(names,[0]*8,[float('nan')]*8)

def test_physx_four_pi_wrap_does_not_reverse_encoder_velocity():
    from turtlebot_swerve.encoder_odometry import WheelPositionUnwrapper
    u=WheelPositionUnwrapper(['lh1_wheel_joint','arm_joint1'])
    a=u.update([2*np.pi-.05,3.1]);b=u.update([-2*np.pi+.05,3.12])
    assert np.isclose(b[0]-a[0],.1)
    assert b[1]==3.12
    c=u.update([2*np.pi-.05,3.13])
    assert np.isclose(c[0]-b[0],-.1)

def test_multiple_revolutions_keep_motor_counts_continuous():
    from turtlebot_swerve.encoder_odometry import WheelPositionUnwrapper
    u=WheelPositionUnwrapper(['rh2_wheel_joint'])
    truth=np.arange(0,40,.1)
    measured=[u.update([(x+2*np.pi)%(4*np.pi)-2*np.pi])[0] for x in truth]
    assert np.allclose(measured,truth)
