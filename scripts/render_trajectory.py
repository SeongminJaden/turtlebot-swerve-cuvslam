"""Render measured PhysX body poses in Isaac Sim; this is recorded-state playback."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image
Image.init()
p=argparse.ArgumentParser();p.add_argument('--recording',type=Path,required=True);p.add_argument('--output',type=Path);p.add_argument('--max-time',type=float,default=40.);p.add_argument('--stereo',action='store_true');p.add_argument('--frames',type=int,default=48);args=p.parse_args()
data=np.load(args.recording/'poses.npz');poses=data['bodies'];names=data['body_names'];trajectory=data['trajectory']
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':960,'height':720})
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
    apply_frame(0)
    for _ in range(8):app.update()
    capture(Gf.Vec3d(6,-5,10),Gf.Vec3d(2.2,3.18,0)).save(out/'maze-overview.png')
    if args.stereo:
        for side,offset in [('left',.025),('right',-.025)]:
            sc=UsdGeom.Camera.Define(stage,'/World/Robot/base_link/sim_'+side)
            sc.CreateClippingRangeAttr(Gf.Vec2f(.05,30));sc.CreateFocalLengthAttr(2.4);sc.CreateHorizontalApertureAttr(3.84);sc.CreateVerticalApertureAttr(2.88)
            sc.AddTranslateOp().Set(Gf.Vec3d(.25,offset,.25));sc.AddOrientOp().Set(Gf.Quatf(.5,.5,-.5,-.5))
            rp=rep.create.render_product(str(sc.GetPath()),(640,480));ra=rep.AnnotatorRegistry.get_annotator('rgb');ra.attach(rp)
            for _ in range(6):rep.orchestrator.step(rt_subframes=1)
            pixels=ra.get_data()
            if not hasattr(pixels,'shape') or pixels.shape[:2]!=(480,640):raise RuntimeError('Missing stereo frame')
            Image.fromarray(pixels.copy()).convert('RGB').save(out/('stereo-'+side+'.png'))
    last=len(poses)-1 if args.max_time<=0 else min(len(poses)-1,int(args.max_time/(trajectory[1,0]-trajectory[0,0])))
    frames=[];indices=np.linspace(0,last,args.frames,dtype=int)
    for k,idx in enumerate(indices):
        apply_frame(idx);x,y=trajectory[idx,1:3];target=Gf.Vec3d(float(x),float(y),.12)
        frame=capture(target+Gf.Vec3d(1.15,-1.4,1.8),target);frame.save(out/'frames'/f'{k:03d}.png');frames.append(frame)
        if k in (0,len(indices)//2,len(indices)-1):frame.save(out/f'robot-{k:02d}.png')
        print('RENDER_FRAME',k,'SIM_TIME',float(trajectory[idx,0]),flush=True)
    frames[0].save(out/'isaacsim-recorded-motion.gif',save_all=True,append_images=frames[1:],duration=160,loop=0)
    (out/'capture.json').write_text(json.dumps({'scope':'Isaac Sim rendering of measured PhysX body poses, recorded-state playback; no fabricated trajectory','recording':str(args.recording),'frame_indices':indices.tolist(),'simulated_times_s':trajectory[indices,0].tolist(),'gif_frame_duration_ms':160},indent=2));print('RENDER_COMPLETE',flush=True)
finally:app.close()
