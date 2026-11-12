"""URDF arm forward kinematics from measured joint encoders (XYZW elsewhere)."""
import xml.etree.ElementTree as ET
import numpy as np

def rotation(axis,angle):
    axis=np.asarray(axis,dtype=float);axis/=np.linalg.norm(axis);x,y,z=axis
    k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+np.sin(angle)*k+(1-np.cos(angle))*(k@k)

class ArmKinematics:
    def __init__(self,urdf):
        root=ET.parse(urdf).getroot();self.chain=[]
        for i in range(1,6):
            name=f'arm_joint{i}';joint=root.find(f"joint[@name='{name}']");origin=joint.find('origin')
            xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ');r,p,y=np.fromstring(origin.get('rpy','0 0 0'),sep=' ')
            t=np.eye(4);t[:3,3]=xyz;t[:3,:3]=rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
            self.chain.append((name,t,np.fromstring(joint.find('axis').get('xyz'),sep=' ')))
    def rig(self,names,positions):
        q=dict(zip(names,positions));t=np.eye(4)
        for name,origin,axis in self.chain:
            joint=np.eye(4);joint[:3,:3]=rotation(axis,q[name]);t=t@origin@joint
        mount=np.eye(4);mount[:3,3]=[.04742,0,.059]
        return t@mount
    def left_camera(self,names,positions):
        camera=np.eye(4);camera[:3,3]=[0,.025,0]
        camera[:3,:3]=rotation([0,0,1],-np.pi/2)@rotation([1,0,0],np.pi/2)
        return self.rig(names,positions)@camera
