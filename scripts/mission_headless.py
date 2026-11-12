"""Headless R12 + supplied maze physics preflight. Does not claim mission completion."""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,default=ROOT/'runs/mission_headless')
parser.add_argument('--steps',type=int,default=1200)
parser.add_argument('--full-audit',action='store_true',help='Run mission readiness and rendered signal/barcode audit')
parser.add_argument('--actuators',action='store_true',help='Small joint-drive response test, not autonomous driving')
parser.add_argument('--render',action='store_true',help='Save offscreen overview and onboard RGB images')
args=parser.parse_args()
if args.steps < 120: parser.error('At least 120 steps required')
args.output.mkdir(parents=True,exist_ok=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':640,'height':480,'fast_shutdown':True})
try:
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    from isaacsim.core.api import World
    from turtlebot_swerve.dice_mission import plan, START_BOXES, LOCATIONS
    world=World(stage_units_in_meters=1.,physics_dt=1/120,rendering_dt=1/30)
    stage=omni.usd.get_context().get_stage()
    world.scene.add_default_ground_plane()
    maze=UsdGeom.Xform.Define(stage,'/World/Maze')
    stage.DefinePrim('/World/Maze/Asset').GetReferences().AddReference(str(ROOT/'assets/mission/maze/maze.usda'))
    # Align the supplied lower-left corner pillar to the mission origin, no scaling.
    source=Usd.Stage.Open(str(ROOT/'assets/mission/maze/maze.usda'))
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','guide'])
    anchor=cache.ComputeWorldBound(source.GetPrimAtPath('/maze/wall_edge/collision/geo_3')).ComputeAlignedRange().GetMidpoint()
    maze.AddTranslateOp().Set(Gf.Vec3d(-anchor[0],-anchor[1],0))
    robot=UsdGeom.Xform.Define(stage,'/World/Robot')
    stage.DefinePrim('/World/Robot/Asset').GetReferences().AddReference(str(ROOT/'assets/mission/R12_NO_USB_URDF_USD/usd/r12_no_usb.usdc'))
    robot.AddTranslateOp().Set(Gf.Vec3d(.734,.950,.015))
    robot.AddRotateZOp().Set(180.)
    practice=json.loads((ROOT/'config/mission/practice.json').read_text())
    (args.output/'allocation.json').write_text(json.dumps(plan(practice['sequence'],practice['barcode'],practice['pucks']),indent=2))
    def box(path, center, size, color):
        obj=UsdGeom.Cube.Define(stage,path)
        obj.CreateSizeAttr(1.)
        obj.AddTranslateOp().Set(Gf.Vec3d(*center))
        obj.AddScaleOp().Set(Gf.Vec3d(*size))
        obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
        return obj
    for i,(x,y) in enumerate(START_BOXES):
        box(f'/World/Tables/Start{i+1}',(x,y,.035),(.205,.205,.07),(.6,.5,.3))
    for i in (0,2):
        x,y=LOCATIONS[i]
        box(f'/World/Tables/L{i+1}',(x,y,.035),(.205,.205,.07),(.3,.6,.5))
    # Center inferred from PDF diagram; exact central table center is not specified in text.
    for i,(dx,dy) in enumerate(((-.205,-.205),(-.205,.205),(.205,-.205),(.205,.205))):
        box(f'/World/Tables/Central{i}',(2.202+dx,3.497+dy,.035),(.410,.410,.070),(.6,.5,.3))
    puck_paths=[]
    for i,puck in enumerate(practice['pucks']):
        x,y=START_BOXES[i//2]
        center=(x+(-.04 if i%2==0 else .04),y,.101)
        path=f'/World/Pucks/Puck{i}'
        color=(.8,.03,.03) if puck['color']=='R' else (.03,.8,.03)
        if puck['shape']=='cuboid':
            obj=box(path,center,(.03,.03,.06),color)
        else:
            obj=UsdGeom.Cylinder.Define(stage,path)
            obj.CreateRadiusAttr(.015); obj.CreateHeightAttr(.06)
            obj.AddTranslateOp().Set(Gf.Vec3d(*center))
            obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(obj.GetPrim())
        UsdPhysics.MassAPI.Apply(obj.GetPrim()).CreateMassAttr(.03)
        puck_paths.append(path)
    collisions=sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse())
    joints=[str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    stage.GetRootLayer().Export(str(args.output/'scene.usda'))
    if args.actuators:
        from isaacsim.core.prims import SingleArticulation
        articulation=world.scene.add(SingleArticulation('/World/Robot/Asset',name='r12'))
    world.reset()
    samples=[]
    for i in range(args.steps):
        world.step(render=False)
        if i % 12==0 or i==args.steps-1:
            matrix=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Robot/Asset/base_link'))
            samples.append({'step':i,'position_m':list(matrix.ExtractTranslation()),'up_z':float(matrix.TransformDir(Gf.Vec3d(0,0,1))[2])})
    finite=all(math.isfinite(v) for s in samples for v in s['position_m'])
    displacement=math.dist(samples[0]['position_m'],samples[-1]['position_m'])
    puck_positions={path:list(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path)).ExtractTranslation()) for path in puck_paths}
    pucks_supported=all(abs(pos[2]-.1)<.005 for pos in puck_positions.values())
    settled=[s for s in samples if s['step'] >= args.steps-120]
    settled_drift=max(math.dist(s['position_m'],samples[-1]['position_m']) for s in settled)
    upright=all(s['up_z'] > math.cos(math.radians(10)) for s in samples)
    puck_xy_error=max(math.dist(pos[:2],[START_BOXES[i//2][0]+(-.04 if i%2==0 else .04),START_BOXES[i//2][1]]) for i,pos in enumerate(puck_positions.values()))
    passed=upright and settled_drift < .001 and puck_xy_error < .005 and pucks_supported and finite and displacement<.10 and len(joints)==13
    report={'status':'PASS_PHYSICS_SMOKE' if passed else 'FAIL_PHYSICS_SMOKE','headless':True,'physics_steps':args.steps,'physics_seconds':args.steps/120,'collision_prims':collisions,'revolute_joints':len(joints),'base_displacement_m':displacement,'samples':samples,'maze_translation_m':[-anchor[0],-anchor[1],0], 'puck_positions_ground_truth_m':puck_positions,'pucks_supported':pucks_supported,'settled_drift_m':settled_drift,'upright_within_10deg':upright,'puck_max_xy_error_m':puck_xy_error,'central_table_center_assumption_m':[2.202,3.497],'mission_complete':False,'unverified':['maze matches PDF layout','signal and barcode camera observations','puck perception','gripper actuation and contact grasp','navigation with new R12','placement and fresh camera reinspection','return zone containment']}
    if args.actuators:
        import numpy as np
        initial=np.asarray(articulation.get_joint_positions()).copy()
        names=articulation.dof_names
        for name in names:
            drive=UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath('/World/Robot/Asset/Joints/'+name),'angular')
            if 'wheel' in name:
                drive.GetTargetVelocityAttr().Set(math.degrees(.5))
            else:
                drive.GetTargetPositionAttr().Set(math.degrees(.04))
        for _ in range(120): world.step(render=False)
        measured=np.asarray(articulation.get_joint_positions()).copy()
        changes={name:float(measured[i]-initial[i]) for i,name in enumerate(names)}
        for name in names:
            drive=UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath('/World/Robot/Asset/Joints/'+name),'angular')
            drive.GetTargetVelocityAttr().Set(0.)
            drive.GetTargetPositionAttr().Set(0.)
        for _ in range(120): world.step(render=False)
        report['actuator_response']={'joint_delta_rad':changes,'all_13_responded':len(changes)==13 and all(math.isfinite(v) and v>.005 for v in changes.values()),'scope':'1 second small positive drive targets, followed by zero commands; not trajectory tracking or grasp validation'}
        passed=passed and report['actuator_response']['all_13_responded']
        report['status']='PASS_PHYSICS_SMOKE' if passed else 'FAIL_PHYSICS_SMOKE'
    if args.full_audit:
        from mission_audit import run
        report['full_audit']=run(ROOT,stage,world,args.output/'mission-audit',practice)
    if args.render:
        import omni.replicator.core as rep
        import numpy as np
        from PIL import Image
        from pxr import UsdLux
        UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1500.)
        overview=rep.create.camera(position=(8.,-5.,9.),look_at=(2.2,3.2,0.))
        views=[('overview',overview),('robot_front','/World/Robot/Asset/base_link/r07_camera1/r07_camera1_left_optical/Camera')]
        report['rendered_images']={}
        for name,camera in views:
            product=rep.create.render_product(camera,(640,480))
            annotator=rep.AnnotatorRegistry.get_annotator('rgb')
            annotator.attach(product)
            for _ in range(4): rep.orchestrator.step(rt_subframes=2,delta_time=0.)
            pixels=annotator.get_data()
            if pixels.shape[:2]!=(480,640) or float(np.std(pixels[:,:,:3]))<1.:
                raise RuntimeError(f'Invalid RGB frame: {name}')
            Image.fromarray(pixels).save(args.output/f'{name}.png')
            report['rendered_images'][name]={'shape':list(pixels.shape),'std':float(np.std(pixels[:,:,:3]))}
            annotator.detach(product)
            product.destroy()
    (args.output/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='samples'}),flush=True)
    if args.full_audit and not report['full_audit']['mission_complete']:
        raise RuntimeError('Full mission blocked; see mission-audit/audit.json')
    if not passed: raise RuntimeError('R12 physics smoke failed; see report.json')
except Exception:
    import traceback
    (args.output/'error.txt').write_text(traceback.format_exc())
    traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
    app.close(skip_cleanup=True, exit_code=1)
    raise
else:
    app.close(skip_cleanup=True)
