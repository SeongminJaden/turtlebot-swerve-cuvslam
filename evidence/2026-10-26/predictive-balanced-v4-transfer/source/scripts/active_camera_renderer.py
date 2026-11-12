"""Live lockstep renderer of measured Lab poses, with an actual Isaac Sim GUI.

Physics runs in Lab, rendering in Kit, tracking in cuVSLAM. No recorded pose
playback is used here. Separate processes avoid Python/ROS ABI conflicts.
"""
import argparse,json,socket,time
from pathlib import Path
import numpy as np
from PIL import Image
from depth_clearance import obstacle_points
from pose_math import matrix as pose_matrix
p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=18766);p.add_argument('--directory',type=Path,required=True);p.add_argument('--headless',action='store_true');p.add_argument('--textured-floor',action='store_true');p.add_argument('--async-mode',action='store_true');p.add_argument('--width',type=int,default=640);a=p.parse_args();a.directory.mkdir(parents=True,exist_ok=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':a.headless,'renderer':'RaytracedLighting','anti_aliasing':0,'disable_viewport_updates':a.headless,'width':1280,'height':800,'window_width':1280,'window_height':800})
import omni.usd,omni.replicator.core as rep
from pxr import UsdGeom,UsdLux,UsdPhysics,UsdShade,Sdf,Gf
root=Path(__file__).resolve().parents[1]
try:
 ctx=omni.usd.get_context();ctx.new_stage();s=ctx.get_stage();UsdGeom.SetStageUpAxis(s,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(s,1.)
 s.DefinePrim('/World','Xform');s.DefinePrim('/World/Maze','Xform').GetReferences().AddReference(str(root/'assets/maze/maze.usda'));s.DefinePrim('/World/Robot','Xform').GetReferences().AddReference(str(root/'assets/m38_active_camera_demo.usda'))
 for prim in s.Traverse():
  if prim.IsA(UsdPhysics.Joint):UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
  if prim.HasAPI(UsdPhysics.ArticulationRootAPI):prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
  if prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
 floor=UsdGeom.Cube.Define(s,'/World/Floor');floor.CreateSizeAttr(1.);floor.AddTranslateOp().Set(Gf.Vec3d(2.2,3.18,-.06));floor.AddScaleOp().Set(Gf.Vec3f(8,10,.1));floor.CreateDisplayColorAttr([(.7,.7,.68)])
 if a.textured_floor:
  mesh=UsdGeom.Mesh.Define(s,'/World/SLAMTexture');mesh.CreatePointsAttr([(-1.8,-1.82,-.009),(6.2,-1.82,-.009),(6.2,8.18,-.009),(-1.8,8.18,-.009)]);mesh.CreateFaceVertexCountsAttr([4]);mesh.CreateFaceVertexIndicesAttr([0,1,2,3]);mesh.CreateSubdivisionSchemeAttr('none');UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex).Set([(0,0),(1,0),(1,1),(0,1)])
  mat=UsdShade.Material.Define(s,'/World/TextureMaterial');surf=UsdShade.Shader.Define(s,'/World/TextureMaterial/Surface');surf.CreateIdAttr('UsdPreviewSurface');surf.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.9);tex=UsdShade.Shader.Define(s,'/World/TextureMaterial/Texture');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(str(root/'assets/maze/slam_floor_texture.png'));uv=UsdShade.Shader.Define(s,'/World/TextureMaterial/UV');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result');surf.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb');mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
 UsdLux.DomeLight.Define(s,'/World/Light').CreateIntensityAttr(1800.)
 cam=UsdGeom.Camera.Define(s,'/World/Overview');cam.CreateClippingRangeAttr(Gf.Vec2f(.01,100));cam.CreateFocalLengthAttr(22);view=cam.AddTransformOp()
 anns=[]
 for side in ['left','right']:
  product=rep.create.render_product(f'/World/Robot/arm_link5/camera_rig/{side}',(a.width,a.width*3//4));ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach(product);anns.append(ann)
  if side=='left':
   depth_ann=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');depth_ann.attach(product)
   camera_params=rep.AnnotatorRegistry.get_annotator('CameraParams');camera_params.attach(product)
 import carb
 carb.settings.get_settings().set('/app/window/title','Isaac Sim | Live camera PPO + cuVSLAM gate')
 if not a.headless:
  from omni.kit.viewport.utility import get_active_viewport
  get_active_viewport().set_active_camera('/World/Overview');get_active_viewport().set_texture_resolution((640,400))
 for _ in range(15):app.update()
 ros=socket.create_connection(('127.0.0.1',18767));rr=ros.makefile('r')
 print('RENDERER_READY',flush=True)
 if not a.async_mode:
  server=socket.socket();server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);server.bind(('127.0.0.1',a.port));server.listen(1);server.settimeout(.1)
  while app.is_running():
   app.update()
   try:conn,_=server.accept();break
   except socket.timeout:continue
  reader=conn.makefile('r')
 else:
  def latest_states():
   previous=-1
   while app.is_running() and not (a.directory/'finished').exists():
    try:req=json.loads((a.directory/'latest-state.json').read_text())
    except (FileNotFoundError,json.JSONDecodeError):app.update();continue
    if req['stamp_ns']==previous:app.update();continue
    previous=req['stamp_ns'];yield json.dumps(req)
  reader=latest_states()
 ops={};index=0
 for line in reader:
  req=json.loads(line)
  for name,pose in zip(req['names'],req['bodies']):
   if name not in ops:
    x=UsdGeom.Xformable(s.GetPrimAtPath('/World/Robot/'+name));x.ClearXformOpOrder();ops[name]=(x.AddTranslateOp(),x.AddOrientOp())
   t,q=ops[name];t.Set(Gf.Vec3d(*pose[:3]));q.Set(Gf.Quatf(pose[6],Gf.Vec3f(*pose[3:6])))
  target=Gf.Vec3d(*req['base_pose'][:2],.32);view.Set(Gf.Matrix4d().SetLookAt(target+Gf.Vec3d(1.2,-1.6,1.6),target,Gf.Vec3d(0,0,1)).GetInverse())
  for _ in range(1):rep.orchestrator.step(rt_subframes=1)
  depth=depth_ann.get_data().copy()
  camera_world=np.asarray(UsdGeom.Xformable(s.GetPrimAtPath('/World/Robot/arm_link5/camera_rig/left')).ComputeLocalToWorldTransform(0)).T
  param_data=camera_params.get_data();render_view=np.asarray(param_data['cameraViewTransform']).reshape(4,4)
  transform_error=float(min(np.linalg.norm(render_view@camera_world-np.eye(4)),np.linalg.norm(render_view.T@camera_world-np.eye(4))))
  with (a.directory/'render-timing.jsonl').open('a') as audit:audit.write(json.dumps({'stamp_ns':req['stamp_ns'],'camera_transform_error':transform_error})+'\n')
  camera_base=np.asarray(req['encoder_left_camera_matrix']) if 'encoder_left_camera_matrix' in req else np.linalg.inv(pose_matrix(req['base_pose']))@camera_world
  points=obstacle_points(depth,camera_base,a.width*400/640)
  pixels=np.stack([ann.get_data().copy()[:,:,:3] for ann in anns]);np.save(a.directory/'stereo.npy',pixels)
  if index%30==0:
   Image.fromarray(pixels[0]).save(a.directory/f'left-{index:05}.png')
  rig=UsdGeom.Xformable(s.GetPrimAtPath('/World/Robot/arm_link5/camera_rig')).ComputeLocalToWorldTransform(0);rot=rig.ExtractRotationQuat();xyz=rig.ExtractTranslation();imag=rot.GetImaginary();req['rig_pose']=[*map(float,xyz),*map(float,imag),float(rot.GetReal())]
  ros.sendall((json.dumps({k:req[k] for k in ['stamp_ns','base_pose','rig_pose','encoder_rig_matrix','drop_images'] if k in req})+'\n').encode());response=json.loads(rr.readline());response['camera_forward_base']=(-camera_base[:3,2]).tolist();response['depth_obstacles_base']=points.round(4).tolist();response['depth_valid_fraction']=float(np.isfinite(depth).mean());response['depth_forward_valid_fraction']=float(np.isfinite(depth[depth.shape[0]//2:,depth.shape[1]*2//5:depth.shape[1]*3//5]).mean());response['rig_pose']=req['rig_pose'];response['source_stamp_ns']=req['stamp_ns'];response['received_wall']=time.monotonic()
  if a.async_mode:
   tmp=a.directory/'latest-tracking.tmp';tmp.write_text(json.dumps(response));tmp.replace(a.directory/'latest-tracking.json')
  else:conn.sendall((json.dumps(response)+'\n').encode())
  index+=1
  if index%30==0:print('RENDERED',index,'VO',response['vo_state'],flush=True)
 print('RENDERER_COMPLETE',index,flush=True)
finally:app.close()
