"""Swerve body twist from measured steering and wheel encoders.
No-slip kinematic estimate; wheel slip remains observable disagreement, not corrected truth.
"""
import numpy as np
MODULES=('lh1','rh1','lh2','rh2')
POSITIONS=((.1,.1),(.1,-.1),(-.1,.1),(-.1,-.1))
OFFSETS=(-np.pi/2,-np.pi/2,0.,np.pi)

def body_twist(names,positions,velocities,radius=.03068719889482):
    if radius<=0:raise ValueError('Positive wheel radius required')
    index={n:i for i,n in enumerate(names)};a=[];b=[]
    for name,(x,y),offset in zip(MODULES,POSITIONS,OFFSETS):
        angle=positions[index[name+'_steer_joint']]+offset
        speed=velocities[index[name+'_wheel_joint']]*radius
        a.extend(([1.,0.,-y],[0.,1.,x]));b.extend((speed*np.cos(angle),speed*np.sin(angle)))
    a=np.array(a);b=np.array(b)
    if not np.isfinite(b).all():raise ValueError('Nonfinite encoders')
    twist=np.linalg.lstsq(a,b,rcond=None)[0]
    return twist,float(np.sqrt(np.mean((a@twist-b)**2)))

class WheelPositionUnwrapper:
    """Restore continuous wheel angle from PhysX periodic joint coordinates.

    Samples must be fast enough that actual wheel rotation between samples is
    less than pi radians. Steering and arm joints retain their bounded angles.
    """
    def __init__(self,names):
        self.mask=np.array([n.endswith('_wheel_joint') for n in names])
        self.previous=None;self.continuous=None
    def update(self,positions):
        q=np.asarray(positions,dtype=float)
        if q.shape!=self.mask.shape or not np.isfinite(q).all():
            raise ValueError('Invalid joint positions')
        if self.previous is None:self.continuous=q.copy()
        else:
            delta=q-self.previous
            delta=np.arctan2(np.sin(delta),np.cos(delta))
            self.continuous[self.mask]+=delta[self.mask]
            self.continuous[~self.mask]=q[~self.mask]
        self.previous=q.copy()
        return self.continuous.copy()
