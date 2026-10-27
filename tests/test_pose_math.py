import unittest
import numpy as np
from scripts.pose_math import matrix,quaternion

class PoseTest(unittest.TestCase):
 def test_round_trip_half_turn_and_general(self):
  for q in [[1,0,0,0],[0,1,0,0],[0,0,1,0],[.2,.3,.4,.5],[0,0,0,1]]:
   t=matrix([1.,2.,3.,*q]);np.testing.assert_allclose(matrix([1.,2.,3.,*quaternion(t[:3,:3])]),t,atol=1e-12)
 def test_moving_camera_static_base(self):
  base=matrix([2.,3.,.1,0,0,0,1]);odom_world=matrix([-.3,.4,0,0,0,.2,.98])
  for angle in np.linspace(-3.,3.,21):
   fk=matrix([.3*np.cos(angle),.3*np.sin(angle),.6,0,0,np.sin(angle/2),np.cos(angle/2)])
   measured=odom_world@base@fk
   np.testing.assert_allclose(measured@np.linalg.inv(fk),odom_world@base,atol=1e-12)

 def test_pitched_camera_origin_must_be_normalized_before_planar_filter(self):
  angle=.3
  fk=matrix([.3,0,.6,0,np.sin(angle/2),0,np.cos(angle/2)])
  start=matrix([.35,2.85,0,0,0,0,1]);finish=start.copy();finish[0,3]+=1.
  odom_world=np.linalg.inv(start@fk)
  initial_base=odom_world@start
  measured_base=odom_world@finish
  self.assertAlmostEqual((measured_base-initial_base)[0,3],np.cos(angle))
  normalized=np.linalg.inv(initial_base)@measured_base
  np.testing.assert_allclose(normalized[:3,3],[1.,0.,0.],atol=1e-12)
