"""Gravity/gyro complementary tilt estimate; no absolute yaw estimate.
Acceleration changes can bias tilt; this is not a substitute for calibrated EKF/VIO.
"""
import numpy as np
from .arm_kinematics import rotation
class ImuTilt:
 def __init__(self,tau=2.):self.up=None;self.time=None;self.tau=tau
 def update(self,stamp,gyro,force):
  w=np.asarray(gyro);f=np.asarray(force);norm=np.linalg.norm(f)
  if not np.isfinite([*w,*f,stamp]).all():raise ValueError('Invalid IMU')
  if self.up is None:
   if abs(norm-9.80665)>.5:return None
   self.up=f/norm;self.time=stamp
  else:
   dt=stamp-self.time
   if dt<=0 or dt>.1:self.up=None;self.time=None;return None
   self.time=stamp;speed=np.linalg.norm(w)
   if speed>1e-12:self.up=rotation(w,-speed*dt)@self.up
   if abs(norm-9.80665)<.5:
    blend=dt/(self.tau+dt);self.up=(1-blend)*self.up+blend*f/norm
   self.up/=np.linalg.norm(self.up)
  # Body-to-level XYZW quaternion with yaw fixed to zero.
  roll=np.arctan2(self.up[1],self.up[2]);pitch=np.arctan2(-self.up[0],np.hypot(self.up[1],self.up[2]))
  sr,cr=np.sin(roll/2),np.cos(roll/2);sp,cp=np.sin(pitch/2),np.cos(pitch/2)
  return np.array([sr*cp,cr*sp,-sr*sp,cr*cp])
