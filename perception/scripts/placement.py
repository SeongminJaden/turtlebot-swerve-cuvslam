"""Deterministic supported, non-overlapping random placements in box coordinates."""
import math
import random

CUBOID_RESTS=[('x',0),('x',180),('x',90),('x',270),('y',90),('y',270)]

def sample_layout(seed,side=.205):
    rng=random.Random(seed)
    for retry in range(100):
        result=[]
        for c in range(4):
            axis,tilt=rng.choice(CUBOID_RESTS if c%2 else [('x',0),('x',180)])
            yaw=rng.uniform(-180,180)
            lying=c%2 and tilt in (90,270)
            hx,hy=(.015,.03) if lying and axis=='x' else ((.03,.015) if lying else (.015,.015))
            radius=math.hypot(hx,hy) if c%2 else .015
            angle=math.radians(yaw)
            ex=abs(math.cos(angle))*hx+abs(math.sin(angle))*hy if c%2 else .015
            ey=abs(math.sin(angle))*hx+abs(math.cos(angle))*hy if c%2 else .015
            for attempt in range(500):
                x=rng.uniform(-side/2+ex+.003,side/2-ex-.003)
                y=rng.uniform(-side/2+ey+.003,side/2-ey-.003)
                if all(math.hypot(x-p['x'],y-p['y'])>radius+p['radius']+.003 for p in result):
                    result.append(dict(class_id=c,x=x,y=y,yaw=yaw,axis=axis,tilt=tilt,radius=radius,extent_x=ex,extent_y=ey,center_z=.015 if lying else .030))
                    break
            else:break
        if len(result)==4:return result
    raise RuntimeError('Could not pack four supported pucks')
