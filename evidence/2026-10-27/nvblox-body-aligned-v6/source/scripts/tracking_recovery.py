"""Coordinate continuity and stationary reacquisition checks, without truth."""
import numpy as np
from pose_math import matrix

def continuation_anchor(last_visual,last_odometry,current_odometry):
    return matrix(last_visual)@np.linalg.inv(matrix(last_odometry))@matrix(current_odometry)

def stable_poses(poses,count=8,position_tolerance=.01,angle_tolerance=np.deg2rad(1)):
    if len(poses)<count:return False
    ref=poses[-1]
    for x in poses[-count:]:
        if np.linalg.norm(x[:3,3]-ref[:3,3])>position_tolerance:return False
        angle=np.arccos(np.clip((np.trace(x[:3,:3].T@ref[:3,:3])-1)/2,-1,1))
        if angle>angle_tolerance:return False
    return True

def interpolate_pose(history,stamp_ns,max_gap_ns=100_000_000):
    for before,after in zip(history,history[1:]):
        if before['stamp_ns']<=stamp_ns<=after['stamp_ns']:
            dt=after['stamp_ns']-before['stamp_ns']
            if dt<=0 or dt>max_gap_ns:return None
            fraction=(stamp_ns-before['stamp_ns'])/dt
            a=np.asarray(before['pose']);b=np.asarray(after['pose']);q=b[3:].copy()
            if np.dot(a[3:],q)<0:q=-q
            q=(1-fraction)*a[3:]+fraction*q;q/=np.linalg.norm(q)
            return np.r_[(1-fraction)*a[:3]+fraction*b[:3],q].tolist()
    return None
