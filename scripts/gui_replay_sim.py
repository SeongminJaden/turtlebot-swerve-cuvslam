"""Render measured PhysX body poses in Isaac Sim; this is recorded-state playback."""
import argparse,json,time
from pathlib import Path
import numpy as np
from PIL import Image
Image.init()
p=argparse.ArgumentParser();p.add_argument('--recording',type=Path,required=True);p.add_argument('--output',type=Path);p.add_argument('--max-time',type=float,default=40.);p.add_argument('--stereo',action='store_true');p.add_argument('--frames',type=int,default=48);args=p.parse_args()
data=np.load(args.recording/'poses.npz');poses=data['bodies'];names=data['body_names'];trajectory=data['trajectory']
from isaacsim import SimulationApp
app=SimulationApp({'headless':False,'width':1280,'height':800,'window_width':1280,'window_height':800})
import omni.usd
import omni.replicator.core as rep
from pxr import UsdGeom,UsdLux,UsdPhysics,Gf
root=Path(__file__).resolve().parents[1];out=args.output or args.recording/'render';out.mkdir(exist_ok=True);(out/'frames').mkdir(exist_ok=True)
try:
    ctx=omni.usd.get_context();ctx.new_stage();stage=ctx.get_stage();UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
    stage.DefinePrim('/World','Xform')
    stage.DefinePrim('/World/Maze','Xform').GetReferences().AddReference(str(root/'assets/maze/maze.usda'))
    stage.DefinePrim('/World/Robot','Xform').GetReferences().AddReference(str(root/'assets/m38_runtime.usda'))
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
    floor=UsdGeom.Cube.Define(stage,'/World/Floor');floor.CreateSizeAttr(1.)
    floor.AddTranslateOp().Set(Gf.Vec3d(2.2,3.18,-.06));floor.AddScaleOp().Set(Gf.Vec3f(8,10,.1));floor.CreateDisplayColorAttr([(0.7,0.7,0.68)])
    UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1800.)
    ops=[]
    for name in names:
        x=UsdGeom.Xformable(stage.GetPrimAtPath('/World/Robot/'+str(name)));x.ClearXformOpOrder();ops.append((x.AddTranslateOp(),x.AddOrientOp()))
    def apply_frame(index):
        for (t,q),pose in zip(ops,poses[index]):
            t.Set(Gf.Vec3d(*[float(v) for v in pose[:3]]));q.Set(Gf.Quatf(float(pose[6]),Gf.Vec3f(*[float(v) for v in pose[3:6]])))
    cam=UsdGeom.Camera.Define(stage,'/World/Camera');cam.CreateClippingRangeAttr(Gf.Vec2f(.01,100));cam.CreateFocalLengthAttr(24);op=cam.AddTransformOp()
    product=rep.create.render_product(str(cam.GetPath()),(960,720));ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach(product)
    def capture(eye,target):
        op.Set(Gf.Matrix4d().SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse())
        for _ in range(4):rep.orchestrator.step(rt_subframes=1)
        pixels=ann.get_data()
        if not hasattr(pixels,'shape') or pixels.shape[:2]!=(720,960):raise RuntimeError('No renderer frame')
        return Image.fromarray(pixels.copy()).convert('RGB')
    import carb
    from omni.kit.viewport.utility import get_active_viewport
    carb.settings.get_settings().set('/app/window/title','Isaac Sim | Recorded PhysX baseline playback')
    viewport=get_active_viewport()
    viewport.set_active_camera('/World/Camera')
    begin=time.monotonic()
    print('GUI_REPLAY_READY',flush=True)
    while app.is_running() and time.monotonic()-begin<180:
        ts=((time.monotonic()-begin)*2)%float(trajectory[-1,0])
        idx=min(int(np.searchsorted(trajectory[:,0],ts)),len(poses)-1)
        apply_frame(idx)
        target=Gf.Vec3d(float(trajectory[idx,1]),float(trajectory[idx,2]),.15)
        op.Set(Gf.Matrix4d().SetLookAt(target+Gf.Vec3d(1.8,-2.2,2.8),target,Gf.Vec3d(0,0,1)).GetInverse())
        app.update()
    print('GUI_REPLAY_COMPLETE',flush=True)
finally:app.close()
