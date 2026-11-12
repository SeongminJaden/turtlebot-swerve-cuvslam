"""Validate and uniformly resample Nav2 paths for the trained 5 cm preview."""
import numpy as np

def resample_path(points, spacing=.05):
    p=np.asarray(points,dtype=float)
    if p.ndim!=2 or p.shape[1]!=2 or len(p)<2 or not np.isfinite(p).all():
        raise ValueError('invalid path coordinates')
    p=p[np.r_[True,np.linalg.norm(np.diff(p,axis=0),axis=1)>1e-6]]
    if len(p)<2:raise ValueError('zero-length path')
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
    s=np.r_[np.arange(0.,arc[-1],spacing),arc[-1]]
    return np.column_stack([np.interp(s,arc,p[:,i]) for i in range(2)])

def accept_path(packet,now_ns,goal,max_age=2.5):
    if not packet.get('valid',False) or packet.get('frame_id')!='map':
        raise ValueError('path unavailable or wrong frame')
    if not 0<=(now_ns-packet['stamp_ns'])/1e9<=max_age:
        raise ValueError('stale path')
    if not 0<=(now_ns-packet['slice_stamp_ns'])/1e9<=1.0:
        raise ValueError('stale nvblox map')
    p=resample_path(packet['points'])
    if np.linalg.norm(p[-1]-goal)>.1:raise ValueError('wrong goal')
    return p
