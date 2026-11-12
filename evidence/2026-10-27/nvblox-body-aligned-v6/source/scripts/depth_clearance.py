"""Metric obstacle points from Isaac Sim pinhole depth, without world obstacles.
USD cameras look along -Z with +Y up. camera_to_base is encoder/FK geometry.
"""
import numpy as np

def obstacle_points(depth, camera_to_base, focal_px, min_height=.04, max_height=.45, stride=4):
    depth=np.asarray(depth);h,w=depth.shape
    vv,uu=np.mgrid[0:h:stride,0:w:stride];z=depth[::stride,::stride]
    valid=np.isfinite(z)&(z>.03)&(z<5.)
    zv=z[valid];p=np.stack(((uu[valid]-(w-1)/2)*zv/focal_px,-(vv[valid]-(h-1)/2)*zv/focal_px,-zv,np.ones_like(zv)),axis=-1)
    b=p@np.asarray(camera_to_base).T
    b=b[(b[:,2]>min_height)&(b[:,2]<max_height)]
    # Robot mesh is not an external obstacle; residual self occlusion is unknown space.
    return b[(np.abs(b[:,0])>.23)|(np.abs(b[:,1])>.23),:2]

def corridor_clearance(points, heading, half_width=.315, max_range=3.):
    points=np.asarray(points).reshape(-1,2);u=np.array([np.cos(heading),np.sin(heading)]);v=np.array([-u[1],u[0]])
    longitudinal=points@u;lateral=points@v;hits=longitudinal[(longitudinal>0)&(np.abs(lateral)<half_width)]
    return float(min(max_range,hits.min())) if len(hits) else float(max_range)
