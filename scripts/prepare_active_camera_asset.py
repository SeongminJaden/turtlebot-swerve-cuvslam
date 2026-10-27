"""Simulation-only active camera variant; original M38 limits remain untouched."""
from pathlib import Path
from pxr import Usd, UsdGeom, UsdPhysics, Gf
root=Path(__file__).resolve().parents[1]
p=root/'assets/m38_active_camera_demo.usda'
s=Usd.Stage.CreateNew(str(p));r=s.DefinePrim('/Robot','Xform');r.GetReferences().AddReference('./m38_runtime.usda');s.SetDefaultPrim(r)
s.SetMetadata('metersPerUnit',1.);s.SetMetadata('upAxis','Z')
s.SetMetadata('comment','OpenMANIPULATOR-X stock yaw +/-180 deg reference; custom CAD sampled mesh screen, hardware cable limits unqualified. Stereo rig fixed to arm_link5. Other joints held at zero.')
j=UsdPhysics.RevoluteJoint(s.GetPrimAtPath('/Robot/Joints/arm_joint1'));j.CreateLowerLimitAttr(-180.);j.CreateUpperLimitAttr(180.)
# Camera optical +Z corresponds to the arm link's +X; USD camera looks down -Z.
rig=UsdGeom.Xform.Define(s,'/Robot/arm_link5/camera_rig');rig.AddTranslateOp().Set(Gf.Vec3d(.04742,0,.059))
for side,y in [('left',.025),('right',-.025)]:
 cam=UsdGeom.Camera.Define(s,f'/Robot/arm_link5/camera_rig/{side}')
 cam.AddTranslateOp().Set(Gf.Vec3d(0,y,0));cam.AddRotateXYZOp().Set(Gf.Vec3f(90,0,-90))
 cam.CreateFocalLengthAttr(20.);cam.CreateHorizontalApertureAttr(32.);cam.CreateVerticalApertureAttr(24.);cam.CreateClippingRangeAttr(Gf.Vec2f(.02,100.))
 # 640x480 with fx=fy=400 and stereo baseline 0.05 m.
s.GetRootLayer().Save();print(p)
