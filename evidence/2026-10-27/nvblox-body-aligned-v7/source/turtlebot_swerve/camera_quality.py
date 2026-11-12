"""Observable image statistics for camera-policy data collection.

These are perception inputs, not a substitute for measured cuVSLAM tracking
or a claim that a textured image guarantees accurate localization.
"""
import numpy as np


def camera_quality(rgb,depth,minimum=.03,maximum=5.):
    rgb=np.asarray(rgb,dtype=float);depth=np.asarray(depth,dtype=float)
    if rgb.ndim!=3 or rgb.shape[2]<3 or rgb.shape[:2]!=depth.shape or min(depth.shape)<8:
        raise ValueError('RGB and depth dimensions do not match')
    gray=rgb[...,:3] @ np.array([.299,.587,.114])
    valid=np.isfinite(depth)&(depth>minimum)&(depth<maximum)
    h,w=depth.shape
    near=valid[h//2:,w*2//5:w*3//5]
    gy,gx=np.gradient(gray);edges=np.hypot(gx,gy)>10.
    tiles=[edges[y0:y1,x0:x1].mean()>.02
           for y0,y1 in zip(np.linspace(0,h,7,dtype=int)[:-1],np.linspace(0,h,7,dtype=int)[1:])
           for x0,x1 in zip(np.linspace(0,w,9,dtype=int)[:-1],np.linspace(0,w,9,dtype=int)[1:])]
    lap=gray[:-2,1:-1]+gray[2:,1:-1]+gray[1:-1,:-2]+gray[1:-1,2:]-4*gray[1:-1,1:-1]
    return {'depth_valid_fraction':float(valid.mean()),
            'near_field_depth_valid_fraction':float(near.mean()),
            'texture_tile_fraction':float(np.mean(tiles)),
            'saturated_fraction':float(((gray<5)|(gray>250)).mean()),
            'laplacian_variance':float(lap.var())}
