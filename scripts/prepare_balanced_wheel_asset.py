"""Consistent cylinder-proxy wheel inertias; original CAD assets untouched.
Masses remain assumed, not measured hardware values.
"""
from pathlib import Path
import json
from pxr import Usd,UsdPhysics,Gf
root=Path(__file__).resolve().parents[1];path=root/'assets/m38_balanced_wheels.usda'
s=Usd.Stage.CreateNew(str(path));robot=s.DefinePrim('/Robot','Xform');robot.GetReferences().AddReference('./m38_active_camera_demo.usda');s.SetDefaultPrim(robot);s.SetMetadata('metersPerUnit',1.);s.SetMetadata('upAxis','Z')
s.SetMetadata('comment','Uniform cylinder collision-proxy wheel inertias. Mass remains an assumed scenario, not measured hardware calibration.')
r=.03068719889482;length=.0175;mass=.15;axis=.5*mass*r*r;transverse=mass*(3*r*r+length*length)/12;rows=[]
for name in ('lh1_wheel','rh1_wheel','lh2_wheel','rh2_wheel'):
 api=UsdPhysics.MassAPI.Apply(s.GetPrimAtPath('/Robot/'+name));old=list(api.GetCenterOfMassAttr().Get());inertia=(axis,transverse,transverse) if name in ('lh1_wheel','rh1_wheel') else (transverse,axis,transverse)
 api.CreateCenterOfMassAttr(Gf.Vec3f(0,0,0));api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia));api.CreatePrincipalAxesAttr(Gf.Quatf(1,0,0,0))
 rows.append({'link':name,'previous_com_m':old,'proxy_com_m':[0,0,0],'mass_kg':mass,'inertia_kg_m2':inertia})
s.GetRootLayer().Save();(root/'assets/m38_wheel_inertia_audit.json').write_text(json.dumps({'status':'simulation proxy correction, unmeasured mass','radius_m':r,'length_m':length,'wheels':rows},indent=2));print(path)
