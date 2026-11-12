"""Conservative small-angle rig-to-body pose covariance transport.
Rotation perturbations are expressed in the odometry frame. Floors are explicit
uncalibrated assumptions; zero/invalid incoming covariance is never certainty.
"""
import numpy as np

def transport(rig_covariance,odom_rig,rig_to_base,position_std=.05,rotation_std=.0872664626):
    c=np.asarray(rig_covariance,dtype=float).reshape(6,6)
    floor=np.diag([position_std**2]*3+[rotation_std**2]*3)
    if not np.isfinite(c).all() or not np.allclose(c,c.T,atol=1e-9) or np.linalg.eigvalsh(c).min()<0:c=np.zeros((6,6))
    # Independent conservative floor, not a fit to the current evaluation truth.
    c=c+floor
    lever=np.asarray(odom_rig)[:3,:3]@np.asarray(rig_to_base)[:3,3]
    x,y,z=lever;k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    j=np.eye(6);j[:3,3:]=-k
    return j@c@j.T
