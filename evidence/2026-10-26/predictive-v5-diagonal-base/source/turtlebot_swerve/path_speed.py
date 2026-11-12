"""Path-only steering/braking envelope for a learned acceleration policy.
No simulator pose, contacts, or hidden obstacles are used. Limits are deployment
actuator constraints, not a claim that PPO learned them without supervision.
"""
import numpy as np

class PathSpeedEnvelope:
    def __init__(self, route, maximum=.92, steering_rate=.6, braking=.4):
        p=np.asarray(route,dtype=float)
        self.arc=np.r_[0.,np.linalg.norm(np.diff(p,axis=0),axis=1).cumsum()]
        lo=np.maximum(np.arange(len(p))-2,0);hi=np.minimum(np.arange(len(p))+2,len(p)-1)
        tangent=p[hi]-p[lo]
        angle=np.unwrap(np.arctan2(tangent[:,1],tangent[:,0]))
        curvature=np.abs(np.gradient(angle,self.arc))
        limit=np.minimum(maximum,steering_rate/np.maximum(curvature,1e-6))
        limit[-1]=0.
        # Brake ahead of curves/goal, accounting for measured command slew.
        for i in range(len(p)-2,-1,-1):
            limit[i]=min(limit[i],np.sqrt(limit[i+1]**2+2*braking*(self.arc[i+1]-self.arc[i])))
        self.limit=limit
    def at(self,index,speed=0.,latency=.2):
        # Evaluate slightly ahead to cover image/control latency.
        s=min(self.arc[-1],self.arc[int(index)]+max(0.,speed)*latency)
        return float(np.interp(s,self.arc,self.limit))
