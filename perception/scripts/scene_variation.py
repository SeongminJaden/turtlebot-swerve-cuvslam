"""Seeded lighting and inward carton dents; deterministic dataset provenance."""
import math
import numpy as np

def sample_lighting(rng,condition):
    dome=math.exp(rng.uniform(math.log(70),math.log(1000)))
    sun=math.exp(rng.uniform(math.log(30),math.log(2400)))
    if condition=='dark':dome*=.35;sun*=.35
    if condition=='bright':dome*=1.5;sun*=1.5
    return dict(dome_intensity=dome,sun_intensity=sun,dome_kelvin=rng.uniform(3000,7500),
                sun_kelvin=rng.uniform(2800,7500),sun_rotation_deg=[rng.uniform(5,80),rng.uniform(-40,40),rng.uniform(-180,180)],
                shadow_angle_deg=rng.uniform(1,25))

def carton_surface(rng,mode,subdivisions=6):
    # Independent face grids keep the UV atlas; a shared spatial warp keeps seams closed.
    points=[];uv=[];counts=[];indices=[]
    corners=[[(.5,-.5,-.5),(.5,.5,-.5),(.5,.5,.5),(.5,-.5,.5)],
             [(-.5,.5,-.5),(-.5,-.5,-.5),(-.5,-.5,.5),(-.5,.5,.5)],
             [(.5,.5,-.5),(-.5,.5,-.5),(-.5,.5,.5),(.5,.5,.5)],
             [(-.5,-.5,-.5),(.5,-.5,-.5),(.5,-.5,.5),(-.5,-.5,.5)],
             [(-.5,-.5,.5),(.5,-.5,.5),(.5,.5,.5),(-.5,.5,.5)],
             [(-.5,.5,-.5),(.5,.5,-.5),(.5,-.5,-.5),(-.5,-.5,-.5)]]
    amount=rng.uniform(.08,.28);side=rng.choice([-1,1]);axis=rng.randrange(2)
    for face,quad in enumerate(corners):
        quad=np.array(quad);base=len(points)
        for j in range(subdivisions+1):
            v=j/subdivisions
            for i in range(subdivisions+1):
                u=i/subdivisions;p=(1-v)*((1-u)*quad[0]+u*quad[1])+v*((1-u)*quad[3]+u*quad[2]);x,y,z=p
                if mode=='side_dent':
                    weight=max(0,side*p[axis]*2)*max(0,1-(2*z)**2)*max(0,1-(2*p[1-axis])**2)
                    p[axis]-=side*amount*weight
                elif mode=='corner_dent':
                    weight=max(0,(side*x+.5))*max(0,(y+.5))*(z+.5)
                    p[0]-=side*amount*weight;p[1]-=amount*.65*weight
                elif mode=='top_sag':
                    p[2]-=amount*max(0,1-(2*x)**2)*max(0,1-(2*y)**2)*(z+.5)
                elif mode=='compressed':p[2]=-.5+(z+.5)*(1-amount)
                elif mode!='intact':raise ValueError(mode)
                points.append(p.tolist());uv.append(((face%3+.005+.99*u)/3,(face//3+.005+.99*v)/2))
        for j in range(subdivisions):
            for i in range(subdivisions):
                a=base+j*(subdivisions+1)+i;indices.extend([a,a+1,a+subdivisions+2,a+subdivisions+1]);counts.append(4)
    return points,uv,counts,indices,dict(mode=mode,normalized_amplitude=amount if mode!='intact' else 0)
