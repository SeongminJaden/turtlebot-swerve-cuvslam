"""Route-tangent chassis alignment, independent of lateral tracking error."""
import math
import numpy as np


def route_heading(route, index, preview=.25):
    p=np.asarray(route,dtype=float)
    arc=np.r_[0.,np.linalg.norm(np.diff(p,axis=0),axis=1).cumsum()]
    i=int(np.clip(index,0,len(p)-2))
    end=min(len(p)-1,max(i+1,int(np.searchsorted(arc,arc[i]+preview))))
    segments=np.diff(p[i:end+1],axis=0)
    segments=segments[np.linalg.norm(segments,axis=1)>1e-6]
    if not len(segments):raise ValueError('route has no heading')
    angles=np.unwrap(np.arctan2(segments[:,1],segments[:,0]))
    direction=p[end]-p[i]
    target=math.atan2(direction[1],direction[0])
    straight=bool(np.ptp(angles)<math.radians(5))
    return target,straight


def alignment_speed_cap(target,yaw,straight,maximum=.92):
    error=abs(math.atan2(math.sin(target-yaw),math.cos(target-yaw)))
    # Straight travel starts only after alignment; turns retain holonomic motion.
    if straight and error>math.radians(5):return 0.
    return maximum*max(0.,math.cos(error))**2
