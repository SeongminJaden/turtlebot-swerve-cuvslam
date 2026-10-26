"""Check portable assets, URDF tree and synthetic camera transforms; no physics claim."""
from pathlib import Path
import json, math, xml.etree.ElementTree as ET
from pxr import Usd, UsdGeom, UsdUtils, Gf

ROOT=Path(__file__).resolve().parents[1]
urdf=ROOT/'assets/m38_arm_description/urdf/m38_arm.urdf'
robot=ET.parse(urdf).getroot()
links={link.attrib['name'] for link in robot.findall('link')}
assert len(links)==len(robot.findall('link'))
parents={}
for joint in robot.findall('joint'):
    p=joint.find('parent').attrib['link'];c=joint.find('child').attrib['link']
    assert p in links and c in links
    assert c not in parents, f'Multiple parents: {c}'
    parents[c]=p
assert links-set(parents)=={'base_link'}
for link in links:
    seen=set();current=link
    while current in parents:
        assert current not in seen,'Cycle in URDF'
        seen.add(current);current=parents[current]
    assert current=='base_link'
for mesh in robot.findall('.//mesh'):
    filename=mesh.attrib['filename']
    assert not Path(filename).is_absolute()
    assert (urdf.parent/filename).resolve().is_file(),filename
stage=Usd.Stage.Open(str(ROOT/'assets/m38_stereo.usda'))
assert UsdGeom.GetStageMetersPerUnit(stage)==1.
assert UsdGeom.GetStageUpAxis(stage)=='Z'
_,_,unresolved=UsdUtils.ComputeAllDependencies(str(ROOT/'assets/m38_stereo.usda'))
assert not unresolved,unresolved
cache=UsdGeom.XformCache()
positions=[]
for side,sign in [('left',1),('right',-1)]:
    name=f'sim_{side}_optical'
    camera=UsdGeom.Camera(stage.GetPrimAtPath('/World/Robot/base_link/'+name))
    assert camera
    local=UsdGeom.Xformable(camera).GetLocalTransformation()
    pos=local.ExtractTranslation();positions.append(pos)
    assert (pos-Gf.Vec3d(.25,sign*.025,.25)).GetLength()<1e-6
    assert (local.TransformDir(Gf.Vec3d(0,0,-1))-Gf.Vec3d(1,0,0)).GetLength()<1e-6
    assert (local.TransformDir(Gf.Vec3d(1,0,0))-Gf.Vec3d(0,-1,0)).GetLength()<1e-6
    joint=robot.find(f"joint[@name='{name}_fixed']")
    xyz=[float(x) for x in joint.find('origin').attrib['xyz'].split()]
    assert max(abs(xyz[i]-pos[i]) for i in range(3))<1e-6
    rpy=[float(x) for x in joint.find('origin').attrib['rpy'].split()]
    assert max(abs(a-b) for a,b in zip(rpy,[-math.pi/2,0,-math.pi/2]))<1e-9
    fx=640*camera.GetFocalLengthAttr().Get()/camera.GetHorizontalApertureAttr().Get()
    fy=480*camera.GetFocalLengthAttr().Get()/camera.GetVerticalApertureAttr().Get()
    assert abs(fx-400)<1e-3 and abs(fy-400)<1e-3
assert abs((positions[1]-positions[0]).GetLength()-.05)<1e-6
report=dict(status='PASS_STATIC_ONLY',links=len(links),joints=len(parents),
            movable_joints=sum(j.attrib['type']!='fixed' for j in robot.findall('joint')),
            meshes=len(robot.findall('.//mesh')),unresolved_usd_assets=unresolved,
            checks=['URDF tree','relative mesh files','USD reference resolution','meter and Z-up','stereo baseline','optical axis','URDF/USD camera position','synthetic intrinsics'],
            not_checked=['Ubuntu runtime','Isaac Sim 6.1 rendering','Isaac ROS 5.0 tracking','Isaac Lab 3 training','real camera calibration','full joint/collision equivalence'])
(ROOT/'checks').mkdir(exist_ok=True)
(ROOT/'checks/model_validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
