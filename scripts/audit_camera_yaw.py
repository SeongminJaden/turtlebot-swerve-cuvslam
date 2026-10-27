"""Sampled mesh clearance audit, not a mechanical/cable qualification."""
import json,math,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,trimesh
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parents[1];out=root/'evidence/2026-10-26/arm-range-audit'
urdf=root/'assets/m38_arm_description/urdf/m38_arm.urdf';r=ET.parse(urdf).getroot()
base=trimesh.load_mesh(root/'assets/m38_arm_description/meshes/base_link.stl')
manager=trimesh.collision.CollisionManager();manager.add_object('base',base)
origins={j.attrib['name']:np.fromstring(j.find('origin').attrib['xyz'],sep=' ') for j in r.findall('joint') if j.attrib['name'].startswith('arm_joint')}
links=[];pos=np.zeros(3)
for i in range(1,6):
 pos=pos+origins['arm_joint'+str(i)]
 if i>=2:
  mesh=trimesh.load_mesh(root/f'assets/m38_arm_description/meshes/arm_link{i}.stl');cm=trimesh.collision.CollisionManager();cm.add_object('arm',mesh);links.append((i,cm,pos.copy()))
records=[]
for deg in range(-180,181):
 R=Rotation.from_euler('z',deg,degrees=True).as_matrix();hits=[];dist=[]
 for i,cm,pos in links:
  T=np.eye(4);T[:3,:3]=R;T[:3,3]=R@(pos-np.array([0,0,.275]))+np.array([0,0,.275])
  cm.set_transform('arm',T);hit=manager.in_collision_other(cm);dist.append(manager.min_distance_other(cm))
  if hit:hits.append(i)
 records.append({'yaw_deg':deg,'hit_links':hits,'min_distance_m':min(dist)})
report={'type':'sampled triangle mesh screen','source':'current decimated CAD136 meshes','step_deg':1,'range_deg':[-180,180],
 'other_arm_joints_rad':[0,0,0,0],'excluded':'adjacent base-arm_link1 mating interface; does not certify cables, narrow-phase exact CAD or dynamics',
 'collision_samples':sum(bool(x['hit_links']) for x in records),'minimum_distance_m':min(x['min_distance_m'] for x in records),'samples':records}
(out/'yaw_mesh_screen.json').write_text(json.dumps(report,indent=2));print({k:v for k,v in report.items() if k!='samples'})
