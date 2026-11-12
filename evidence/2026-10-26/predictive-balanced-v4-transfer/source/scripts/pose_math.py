"""Small XYZW pose conversions shared by the ROS simulation adapter."""
import numpy as np

def matrix(pose):
 x,y,z,w=np.asarray(pose[3:],dtype=float);norm=x*x+y*y+z*z+w*w
 if norm<1e-12:raise ValueError('Zero quaternion')
 s=2/norm;t=np.eye(4)
 t[:3,:3]=[[1-s*(y*y+z*z),s*(x*y-z*w),s*(x*z+y*w)],
             [s*(x*y+z*w),1-s*(x*x+z*z),s*(y*z-x*w)],
             [s*(x*z-y*w),s*(y*z+x*w),1-s*(x*x+y*y)]]
 t[:3,3]=pose[:3];return t

def quaternion(r):
 # Eigen formulation also handles half turns without a trace singularity.
 k=np.array([[r[0,0]-r[1,1]-r[2,2],r[1,0]+r[0,1],r[2,0]+r[0,2],r[2,1]-r[1,2]],
             [r[1,0]+r[0,1],r[1,1]-r[0,0]-r[2,2],r[2,1]+r[1,2],r[0,2]-r[2,0]],
             [r[2,0]+r[0,2],r[2,1]+r[1,2],r[2,2]-r[0,0]-r[1,1],r[1,0]-r[0,1]],
             [r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1],np.trace(r)]])/3
 _,v=np.linalg.eigh(k);q=v[:,-1];return q if q[3]>=0 else -q
