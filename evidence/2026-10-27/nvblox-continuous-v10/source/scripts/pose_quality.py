"""Conservative pose-jump rejection, not a proof of localization accuracy.

A large jump latches driving off until the tracker/bridge is restarted and
localized again. Gradual drift remains possible: evaluate against ground truth.
"""
import math
import numpy as np
from pose_math import matrix

class PoseQuality:
 def __init__(self,max_speed=.95,max_angular_speed=3.):
  self.max_speed=max_speed;self.max_angular_speed=max_angular_speed
  self.previous=None;self.stamp=None;self.latched=False;self.reason='initializing'
 def update(self,pose,stamp_ns):
  if self.latched:return False
  if len(pose)!=7 or not np.isfinite(pose).all():self.latched=True;self.reason='invalid_pose';return False
  try:current=matrix(pose)
  except ValueError:self.latched=True;self.reason='invalid_quaternion';return False
  if self.previous is not None:
   dt=(stamp_ns-self.stamp)/1e9
   distance=float(np.linalg.norm(current[:3,3]-self.previous[:3,3]))
   rot=self.previous[:3,:3].T@current[:3,:3];angle=float(np.arccos(np.clip((np.trace(rot)-1)/2,-1,1)))
   if dt<=0 or dt>1. or distance>.02+self.max_speed*dt or angle>.05+self.max_angular_speed*dt:
    self.latched=True;self.reason='pose_jump_or_gap';return False
  self.previous=current;self.stamp=stamp_ns;self.reason='plausible';return True
