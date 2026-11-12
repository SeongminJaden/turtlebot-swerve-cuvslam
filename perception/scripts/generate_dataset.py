"""Generate fixed 30x30x60 mm / diameter30x60 mm tagged pucks and pose GT.
Requires the separate Isaac Sim Python environment and accepted NVIDIA EULA.
"""
import argparse
import json
from pathlib import Path
import math
import random
import time
from placement import sample_layout
from box_appearance import create_carton_atlas,cube_mesh_data
from scene_variation import sample_lighting,carton_surface

NAMES=['red_cylinder','red_cuboid','green_cylinder','green_cuboid','box','apriltag']

def split_seed_ranges(train, val, test, seed_base):
    if min(train, val, test, seed_base) < 0:
        raise ValueError('Counts and seed base must be nonnegative')
    return [('train',train,seed_base),('val',val,seed_base+train),
            ('test',test,seed_base+train+val)]

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--domain',choices=['legacy','realistic'],default='legacy')
    p.add_argument('--focal-px',type=float,default=640*24/20.955)
    p.add_argument('--gui',action='store_true')
    p.add_argument('--watch-training',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--train',type=int,default=800)
    p.add_argument('--val',type=int,default=100)
    p.add_argument('--test',type=int,default=100)
    p.add_argument('--video-frames',type=int,default=120)
    p.add_argument('--prism-tags',choices=['top','six_faces'],default='six_faces')
    p.add_argument('--seed-base',type=int,default=100000,help='First scene seed; choose a fresh range for independent datasets')
    p.add_argument('--materials',type=Path,default=Path(__file__).resolve().parents[2]/'config/perception/materials.json')
    p.add_argument('--tag-size',type=float,default=.016)
    p.add_argument('--tag-patterns',type=Path,default=Path(__file__).resolve().parents[2]/'config/perception/tag36h11.json')
    a=p.parse_args()
    out=a.output.resolve()
    if not math.isclose(a.tag_size,.016):p.error('Actual register texture uses 16mm black / 20mm white size')
    patterns=json.loads(a.tag_patterns.read_text())
    material_config=json.loads(a.materials.read_text())
    if out.exists():raise SystemExit('Refusing to overwrite dataset')
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':not a.gui,'renderer':'RaytracedLighting','width':640,'height':640,
                       'anti_aliasing':0,'multi_gpu':False})
    try:
        import omni.usd
        import omni.replicator.core as rep
        import numpy as np
        from PIL import Image
        from pxr import UsdGeom,UsdLux,UsdShade,UsdSemantics,Gf,Sdf
        stage=omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage,1.0)
        UsdGeom.Xform.Define(stage,'/World')
        def material(path,color):
            m=UsdShade.Material.Define(stage,path)
            sh=UsdShade.Shader.Define(stage,path+'/Shader');sh.CreateIdAttr('UsdPreviewSurface')
            sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.55)
            m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
            return m,sh
        ground=UsdGeom.Cube.Define(stage,'/World/Table');ground.CreateSizeAttr(1)
        ground.AddTransformOp().Set(Gf.Matrix4d().SetScale(Gf.Vec3d(1.8,1.5,.025))*Gf.Matrix4d().SetTranslate(Gf.Vec3d(0,0,-.0125)))
        table_mat,table_sh=material('/World/TableMaterial',(.25,.25,.24))
        UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(table_mat)
        ground.GetPrim().GetAttribute('xformOp:transform').Set(Gf.Matrix4d().SetScale(Gf.Vec3d(1.8,1.5,.025))*Gf.Matrix4d().SetTranslate(Gf.Vec3d(0,0,-.0825)))
        out.mkdir(parents=True)
        if a.domain=='realistic':
            workbox=UsdGeom.Mesh.Define(stage,'/World/WorkBox');points,uv=cube_mesh_data()
            workbox.CreatePointsAttr([Gf.Vec3f(*v) for v in points]);workbox.CreateFaceVertexCountsAttr([4]*6)
            workbox.CreateFaceVertexIndicesAttr(list(range(24)));workbox.CreateSubdivisionSchemeAttr('none')
            UsdGeom.PrimvarsAPI(workbox).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.faceVarying).Set([Gf.Vec2f(*v) for v in uv])
        else:
            workbox=UsdGeom.Cube.Define(stage,'/World/WorkBox');workbox.CreateSizeAttr(1.)
        boxop=workbox.AddTransformOp()
        boxmat,boxsh=material('/World/BoxMaterial',(.62,.52,.32))
        if a.domain=='realistic':
            texture=out/'textures/carton.png';create_carton_atlas(texture)
            uvreader=UsdShade.Shader.Define(stage,'/World/BoxMaterial/UV');uvreader.CreateIdAttr('UsdPrimvarReader_float2');uvreader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
            tex=UsdShade.Shader.Define(stage,'/World/BoxMaterial/Texture');tex.CreateIdAttr('UsdUVTexture')
            tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(str(texture));tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB')
            tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uvreader.ConnectableAPI(),'result')
            boxsh.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(),'rgb')
            boxsh.GetInput('roughness').Set(.85)
        UsdShade.MaterialBindingAPI.Apply(workbox.GetPrim()).Bind(boxmat)
        UsdSemantics.LabelsAPI.Apply(workbox.GetPrim(),'class').CreateLabelsAttr(['box'])
        puck_shaders=[]
        targets=[]
        tags=[]
        face_transforms={}
        face_paths={}
        (out/'textures').mkdir(exist_ok=True)
        tag_materials={}
        for tag_id in range(15):
            bitmap=np.pad(np.array(patterns[str(tag_id)],dtype=np.uint8)*255,1,constant_values=255)
            texture=out/'textures'/f'tag36h11-{tag_id}.png'
            Image.fromarray(bitmap).resize((500,500),Image.Resampling.NEAREST).save(texture)
            mat,shader=material(f'/World/TagMaterial{tag_id}',(1,1,1))
            uv=UsdShade.Shader.Define(stage,f'/World/TagMaterial{tag_id}/UV');uv.CreateIdAttr('UsdPrimvarReader_float2')
            uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
            tex=UsdShade.Shader.Define(stage,f'/World/TagMaterial{tag_id}/Texture');tex.CreateIdAttr('UsdUVTexture')
            tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(str(texture))
            tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('raw')
            tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
            shader.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(),'rgb')
            tag_materials[tag_id]=mat
        def tag_surface(path,half,curved=False):
            mesh=UsdGeom.Mesh.Define(stage,path+'/Surface');points=[];uvs=[]
            strips=24 if curved else 1
            for j in range(strips):
                u0,u1=j/strips,(j+1)/strips
                for u,v in [(u0,0),(u1,0),(u1,1),(u0,1)]:
                    x,y=(2*u-1)*half,(2*v-1)*half
                    points.append(Gf.Vec3f(.01503*math.sin(x/.01503) if curved else x,y,.01503*(math.cos(x/.01503)-1) if curved else 0))
                    uvs.append(Gf.Vec2f(u,v))
            if path=='/World/BoxTag':points=[Gf.Vec3f(p[0],p[1],p[2]+.00002) for p in points]
            mesh.CreatePointsAttr(points);mesh.CreateFaceVertexCountsAttr([4]*strips)
            mesh.CreateFaceVertexIndicesAttr(list(range(4*strips)));mesh.CreateDoubleSidedAttr(True)
            UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.faceVarying).Set(uvs)
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(tag_materials[0])
            return mesh
        black,_=material('/World/TagBlack',(.001,.001,.001))
        white,_=material('/World/TagWhite',(.95,.95,.95))
        def tile(path, bounds, mat):
            x1,y1,x2,y2,z=bounds
            mesh=UsdGeom.Mesh.Define(stage,path)
            def point(x,y,z):
                if '/side/' in path:
                    r=.01503
                    return Gf.Vec3f(r*math.sin(x/r),y,r*(math.cos(x/r)-1)+z)
                return Gf.Vec3f(x,y,z)
            mesh.CreatePointsAttr([point(x1,y1,z),point(x2,y1,z),point(x2,y2,z),point(x1,y2,z)])
            mesh.CreateFaceVertexCountsAttr([4]);mesh.CreateFaceVertexIndicesAttr([0,1,2,3])
            mesh.CreateDoubleSidedAttr(True)
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        for c in range(4):
            tag=UsdGeom.Xform.Define(stage,f'/World/Tag{c}')
            specs=[('pz',(0,0,1),.03)]
            if c%2 and a.prism_tags=='six_faces':
                specs += [('nz',(0,0,-1),.03),('px',(1,0,0),.015),
                          ('nx',(-1,0,0),.015),('py',(0,1,0),.015),('ny',(0,-1,0),.015)]
            if c%2==0:specs += [('nz',(0,0,-1),.03),('side',(1,0,0),.015)]
            face_transforms[c]=[];face_paths[c]=[]
            for face,normal,distance in specs:
                path=f'/World/Tag{c}/{face}'
                rot=Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1),Gf.Vec3d(*normal)))
                if face=='side':rot=Gf.Matrix4d(0,1,0,0, 0,0,1,0, 1,0,0,0, 0,0,0,1)
                transform=rot*Gf.Matrix4d().SetTranslate(Gf.Vec3d(*normal)*(distance+.00003))
                face_transforms[c].append((face,transform))
                face_paths[c].append(path)
                UsdGeom.Xform.Define(stage,path).AddTransformOp().Set(transform)
                UsdSemantics.LabelsAPI.Apply(stage.GetPrimAtPath(path),'class').CreateLabelsAttr(['apriltag'])
                tag_surface(path,.010,curved=face=='side')
            tags.append((tag,tag.AddTransformOp()))
        box_tag=UsdGeom.Xform.Define(stage,'/World/BoxTag')
        box_tag_op=box_tag.AddTransformOp()
        UsdSemantics.LabelsAPI.Apply(box_tag.GetPrim(),'class').CreateLabelsAttr(['apriltag'])
        tag_surface('/World/BoxTag',.030)
        for c,name in enumerate(NAMES[:4]):
            path='/World/'+name
            if c%2==0:
                g=UsdGeom.Cylinder.Define(stage,path);g.CreateRadiusAttr(.015);g.CreateHeightAttr(.060);g.CreateAxisAttr('Z')
            else:
                g=UsdGeom.Cube.Define(stage,path);g.CreateSizeAttr(1);g.AddScaleOp().Set(Gf.Vec3f(.030,.030,.060))
            # Matrix transform avoids translate scaling through preceding scale ops.
            xf=UsdGeom.Xformable(g);xf.ClearXformOpOrder()
            op=xf.AddTransformOp()
            mat,shader=material('/World/Mat'+str(c),tuple(material_config['red' if c<2 else 'green']['usd_linear_rgb']))
            UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
            UsdSemantics.LabelsAPI.Apply(g.GetPrim(),'class').CreateLabelsAttr([name])
            puck_shaders.append(shader)
            targets.append((g,op))
        distractors=[]
        for j in range(2):
            g=UsdGeom.Sphere.Define(stage,f'/World/Distractor{j}');g.CreateRadiusAttr(.045)
            op=g.AddTranslateOp();m,_=material(f'/World/DistractorMat{j}',(.03,.08,.4) if j==0 else (.4,.35,.25))
            UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(m);distractors.append((g,op))
        dome=UsdLux.DomeLight.Define(stage,'/World/Dome');dome.CreateIntensityAttr(700)
        sun=UsdLux.DistantLight.Define(stage,'/World/Sun');sun.CreateIntensityAttr(1800)
        sun_rotation=sun.AddRotateXYZOp();sun_rotation.Set(Gf.Vec3f(20,-30,25));sun.CreateAngleAttr(8)
        dome.CreateEnableColorTemperatureAttr(True);sun.CreateEnableColorTemperatureAttr(True)
        camera=UsdGeom.Camera.Define(stage,'/World/Camera');camera.CreateFocalLengthAttr(a.focal_px*20.955/640)
        camera.CreateHorizontalApertureAttr(20.955);camera.CreateVerticalApertureAttr(20.955)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.01,100));camop=camera.AddTransformOp()
        rp=rep.create.render_product('/World/Camera',(640,640))
        rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([rp])
        bbox=rep.AnnotatorRegistry.get_annotator('instance_segmentation',init_params={'colorize':False});bbox.attach([rp])
        manifest={'source':'isaac_sim','isaac_sim_version':'6.1.0.0','resolution':[640,640],
                  'domain':a.domain,'materials':material_config,'classes':NAMES,'dimensions_m':[.03,.03,.06],'prism_tag_mount':a.prism_tags,'tag_family':'tag36h11','tag_size_m':a.tag_size,'tag_ids':list(range(12)),'face_id_policy':'same object ID repeated on every tagged face','cylinder_mount':'top,bottom,curved_side','white_margin_size_m':.020,'pose_convention':'upright, inverted, cuboid lying; repeated IDs require symmetry-aware pose loss','camera_intrinsics':{'fx':a.focal_px,'fy':a.focal_px,'cx':320,'cy':320},'box_appearance':('procedural cardboard fibers/folds/approximate Bambu side printing; not a measured scan' if a.domain=='realistic' else 'plain brown proxy'),'center_box_tag_ids':[12,13,14],'placement':'random full support and conservative collision-free footprints','annotation':'visible per-prim instance segmentation masks; grouped by tag face path','samples':[], 'video':[]}
        def set_pose(c, x, y, yaw, tilt=0, axis="x"):
            g, op = targets[c]
            is_prism = c % 2 == 1
            center_z = .015 if is_prism and tilt in (90,270) else .030
            tilt_matrix = Gf.Matrix4d().SetRotate(
                Gf.Rotation(Gf.Vec3d(1, 0, 0) if axis=="x" else Gf.Vec3d(0, 1, 0), tilt)
            )
            yaw_matrix = Gf.Matrix4d().SetRotate(
                Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw)
            )
            translation = Gf.Matrix4d().SetTranslate(
                Gf.Vec3d(x, y, center_z)
            )
            pose = tilt_matrix * yaw_matrix * translation
            size = (0.030, 0.030, 0.060) if is_prism else (1, 1, 1)
            scaling = Gf.Matrix4d().SetScale(Gf.Vec3d(*size))
            op.Set(scaling * pose)
            tags[c][1].Set(pose)
        scene_meta={}
        def setup(seed,condition,negative=False,phase=None):
            rng=random.Random(seed)
            side=.205
            deformation={'mode':'intact'}
            if a.domain=='realistic':
                # Pucks require a flat top: top sag/compression are empty-box scenes.
                mode=rng.choice(['intact','intact','side_dent','corner_dent'] if not negative else ['intact','side_dent','corner_dent','top_sag','compressed'])
                if not negative and mode=='corner_dent':mode='side_dent'
                points,uv,counts,indices,deformation=carton_surface(rng,mode)
                workbox.GetPointsAttr().Set([Gf.Vec3f(*p) for p in points]);workbox.GetFaceVertexCountsAttr().Set(counts);workbox.GetFaceVertexIndicesAttr().Set(indices)
                UsdGeom.PrimvarsAPI(workbox).GetPrimvar('st').SetInterpolation(UsdGeom.Tokens.vertex)
                UsdGeom.PrimvarsAPI(workbox).GetPrimvar('st').Set([Gf.Vec2f(*v) for v in uv])
            box_yaw=rng.uniform(-180,180)
            bx,by=rng.uniform(-.035,.035),rng.uniform(-.035,.035)
            box_frame=Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1),box_yaw))*Gf.Matrix4d().SetTranslate(Gf.Vec3d(bx,by,0))
            boxop.Set(Gf.Matrix4d().SetScale(Gf.Vec3d(side,side,.07))*Gf.Matrix4d().SetTranslate(Gf.Vec3d(0,0,-.035))*box_frame)
            layout=sample_layout(seed,side)
            scene_meta.clear();scene_meta.update(box_deformation=deformation,box_size_m=[side,side,.07],box_yaw_deg=box_yaw,box_center_xy_m=[bx,by],pucks=layout)
            destination_tag=rng.choice([12,13,14])
            has_center_tag=seed%4!=0 and deformation['mode'] not in ('top_sag','compressed')
            box_tag_op.Set(box_frame)
            UsdGeom.Imageable(box_tag.GetPrim()).GetVisibilityAttr().Set(UsdGeom.Tokens.inherited if has_center_tag else UsdGeom.Tokens.invisible)
            scene_meta.update(center_tag_id=destination_tag if has_center_tag else None,center_tag_black_size_m=.048,center_tag_white_size_m=.060)
            UsdShade.MaterialBindingAPI(stage.GetPrimAtPath('/World/BoxTag/Surface')).Bind(tag_materials[destination_tag])
            tag_ids=rng.sample(range(12),4)
            for c,tid in enumerate(tag_ids):
                for fi,path in enumerate(face_paths[c]):
                    face_id=tid
                    stage.GetPrimAtPath(path).CreateAttribute('tag_id',Sdf.ValueTypeNames.Int).Set(face_id)
                    UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(path+'/Surface')).Bind(tag_materials[face_id])
                tags[c][0].GetPrim().CreateAttribute('tag_id',Sdf.ValueTypeNames.Int).Set(tid)
            for c,(g,op) in enumerate(targets):
                UsdGeom.Imageable(g.GetPrim()).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible) if negative else UsdGeom.Imageable(g.GetPrim()).GetVisibilityAttr().Set(UsdGeom.Tokens.inherited)
            for tag,_ in tags:
                UsdGeom.Imageable(tag.GetPrim()).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible) if negative else UsdGeom.Imageable(tag.GetPrim()).GetVisibilityAttr().Set(UsdGeom.Tokens.inherited)
            for item in layout:
                point=box_frame.Transform(Gf.Vec3d(item['x'],item['y'],0))
                set_pose(item['class_id'],point[0],point[1],item['yaw']+box_yaw,item['tilt'],item['axis'])
            height=.23 if condition=='low_angle' else rng.uniform(.30,.40)
            eye=Gf.Vec3d(rng.uniform(-.04,.04),-.20,height)
            look=Gf.Vec3d(bx,by,.03)
            if a.domain=='realistic':
                distance=rng.uniform(.16,.30) if seed%4==0 else rng.uniform(.28,.85);azimuth=rng.uniform(-math.pi,math.pi)
                height=(rng.uniform(.13,.28) if seed%4==0 else rng.uniform(.16,.65)) if condition!='low_angle' else rng.uniform(.12,.23)
                eye=Gf.Vec3d(bx+distance*math.cos(azimuth),by+distance*math.sin(azimuth),height)
                # Include partially framed cartons as seen in the live close-up.
                offset=.13 if seed%3==0 else .045
                look=Gf.Vec3d(bx+rng.uniform(-offset,offset),by+rng.uniform(-offset,offset),rng.uniform(-.01,.12))
                for c,shader in enumerate(puck_shaders):
                    base=material_config['red' if c<2 else 'green']['usd_linear_rgb']
                    gain=rng.uniform(.70,1.25)
                    shader.GetInput('diffuseColor').Set(Gf.Vec3f(*[min(.95,max(.001,v*gain*rng.uniform(.9,1.1))) for v in base]))
                    shader.GetInput('roughness').Set(rng.uniform(.35,.85))
            scene_meta.update(camera_eye=list(eye),camera_look_at=list(look))
            roll=rng.uniform(-15,15) if a.domain=='realistic' else 0
            up=Gf.Rotation((look-eye).GetNormalized(),roll).TransformDir(Gf.Vec3d(0,0,1))
            scene_meta['camera_roll_deg']=roll
            camop.Set(Gf.Matrix4d().SetLookAt(eye,look,up).GetInverse())
            intensity={'dark':120,'bright':1400}.get(condition,rng.uniform(400,850))
            dome.GetIntensityAttr().Set(intensity);sun.GetIntensityAttr().Set(intensity*2)
            if a.domain=='realistic':
                lighting=sample_lighting(rng,condition)
                dome.GetIntensityAttr().Set(lighting['dome_intensity']);sun.GetIntensityAttr().Set(lighting['sun_intensity'])
                dome.CreateColorTemperatureAttr(lighting['dome_kelvin']);sun.CreateColorTemperatureAttr(lighting['sun_kelvin'])
                sun_rotation.Set(Gf.Vec3f(*lighting['sun_rotation_deg']));sun.GetAngleAttr().Set(lighting['shadow_angle_deg'])
                scene_meta['lighting']=lighting
            shade=rng.uniform(.12,.45);table_sh.GetInput('diffuseColor').Set(Gf.Vec3f(shade,shade*.95,shade*.9))
            for g,op in distractors:op.Set(Gf.Vec3d(rng.uniform(-.5,.5),rng.uniform(-.35,.35),.04))
        def capture(attempt=0):
            # Drain render updates after changing geometry, visibility and camera together.
            rep.orchestrator.step(rt_subframes=4)
            rep.orchestrator.step(rt_subframes=4)
            pixels=np.asarray(rgb.get_data())
            if pixels.shape[:2]!=(640,640):raise RuntimeError(f'Unexpected RGB shape {pixels.shape}')
            data=bbox.get_data();rows=[];boxes=[]
            segmentation=np.asarray(data['data'])
            groups={}
            for instance_id,path in data['info']['idToLabels'].items():
                path=str(path)
                root=None;c=None
                if path.startswith('/World/Tag'):
                    parts=path.split('/')
                    if len(parts)>=4:root='/'.join(parts[:4]);c=5
                elif path.startswith('/World/BoxTag'):root='/World/BoxTag';c=5
                elif path.startswith('/World/WorkBox'):root='/World/WorkBox';c=4
                else:
                    for index,name in enumerate(NAMES[:4]):
                        if path.startswith('/World/'+name):root='/World/'+name;c=index;break
                if root is not None:groups.setdefault((root,c),[]).append(int(instance_id))
            for (path,c),instance_ids in groups.items():
                ys,xs=np.nonzero(np.isin(segmentation,instance_ids))
                if not len(xs):continue
                x1,y1,x2,y2=float(xs.min()),float(ys.min()),float(xs.max()+1),float(ys.max()+1)
                if x2-x1<2 or y2-y1<2:continue
                rows.append(f'{c} {(x1+x2)/1280:.8f} {(y1+y2)/1280:.8f} {(x2-x1)/640:.8f} {(y2-y1)/640:.8f}')
                boxes.append({'class_id':c,'xyxy':[x1,y1,x2,y2],'instance_path':path,'mask_pixel_count':len(xs)})
            if not groups:raise RuntimeError(f'No known instance paths: {data["info"]}')
            poses=[]
            world_to_cam=camop.Get().GetInverse()
            focal=a.focal_px
            def optical(point):
                q=world_to_cam.Transform(Gf.Vec3d(*point))
                return [q[0],-q[1],-q[2]]
            def project(q):return [320+focal*q[0]/q[2],320+focal*q[1]/q[2]]
            for c,(_,op) in enumerate(tags):
                if UsdGeom.Imageable(targets[c][0].GetPrim()).ComputeVisibility()=='invisible':continue
                m=op.Get()
                corners=[[x,y,z] for x in [-.015,.015] for y in [-.015,.015] for z in [-.03,.03]]
                pts=[optical(m.Transform(Gf.Vec3d(*p))) for p in [[0,0,0]]+corners]
                h=a.tag_size/2
                tag_faces=[]
                for face,local in face_transforms[c]:
                    world=local*m
                    def surface(x,y):
                        if face=='side':return Gf.Vec3d(.01503*math.sin(x/.01503),y,.01503*(math.cos(x/.01503)-1))
                        return Gf.Vec3d(x,y,0)
                    tc=[optical(world.Transform(surface(x,y))) for x,y in [(-h,h),(h,h),(h,-h),(-h,-h)]]
                    tag_faces.append(dict(face=face,tag_id=stage.GetPrimAtPath(f'/World/Tag{c}/{face}').GetAttribute('tag_id').Get(),tag_corners_camera_m=tc,tag_corners_px=[project(q) for q in tc]))
                tc=tag_faces[0]['tag_corners_camera_m']
                poses.append(dict(tag_faces=tag_faces,class_id=c,tag_id=tags[c][0].GetPrim().GetAttribute('tag_id').Get(),center_camera_m=pts[0],cuboid_camera_m=pts,projected_cuboid=[project(q) for q in pts],tag_corners_camera_m=tc,tag_corners_px=[project(q) for q in tc]))
            bm=boxop.Get()
            bp=[optical(bm.Transform(Gf.Vec3d(*p))) for p in [[0,0,0]]+[[x,y,z] for x in [-.5,.5] for y in [-.5,.5] for z in [-.5,.5]]]
            dimensions=[float(bm.TransformDir(Gf.Vec3d(*v)).GetLength()) for v in [(1,0,0),(0,1,0),(0,0,1)]]
            poses.append(dict(class_id=4,pose_supervision_valid=scene_meta['box_deformation']['mode']=='intact',dimensions_m=dimensions,center_camera_m=bp[0],cuboid_camera_m=bp,projected_cuboid=[project(q) for q in bp],tag_faces=[]))
            transform_cache=UsdGeom.XformCache()
            for box in boxes:
                if box['class_id']!=5:continue
                mesh=UsdGeom.Mesh.Get(stage,box['instance_path']+'/Surface')
                world=transform_cache.GetLocalToWorldTransform(mesh.GetPrim())
                uv=np.array([project(optical(world.Transform(Gf.Vec3d(*p)))) for p in mesh.GetPointsAttr().Get()])
                bound=[float(uv[:,0].min()),float(uv[:,1].min()),float(uv[:,0].max()),float(uv[:,1].max())]
                box['surface_bounds_px']=bound
                x1,y1,x2,y2=box['xyxy']
                if not (x1>=bound[0]-2 and y1>=bound[1]-2 and x2<=bound[2]+2 and y2<=bound[3]+2):
                    if attempt < 2:
                        print(f'RENDER SYNC RETRY {attempt+1}: {box["instance_path"]}',flush=True)
                        return capture(attempt+1)
                    raise RuntimeError(f'Instance mask escaped its physical tag surface after retries: {box}')
            return pixels[:,:,:3],rows,boxes,poses
        if a.gui:
            from omni.kit.viewport.utility import get_active_viewport
            viewport=get_active_viewport()
            if viewport:viewport.camera_path='/World/Camera'
            import omni.ui as ui
            training_window=ui.Window('Mission Dataset and Training',width=470,height=200)
            with training_window.frame:
                with ui.VStack():
                    ui.Label('800 train / 100 validation / 100 test')
                    progress_label=ui.Label('Generating randomized supported pucks...')
                    metrics_label=ui.Label('Actual mission tags: puck 0-11 / location 12-14')
        setup(317,'normal')
        for _ in range(10):app.update()
        for split,count,base in split_seed_ranges(a.train,a.val,a.test,a.seed_base):
            (out/'images'/split).mkdir(parents=True);(out/'labels'/split).mkdir(parents=True)
            for i in range(count):
                condition=['normal','dark','bright','occlusion','low_angle'][i%5]
                negative=i%(6 if a.domain=='realistic' else 11)==0;seed=base+i
                setup(seed,condition,negative)
                pixels,labels,boxes,poses=capture()
                if not negative and not labels:raise RuntimeError(f'No renderer labels: {split} {i}')
                image=f'images/{split}/{i:04}.png'
                Image.fromarray(pixels).save(out/image)
                (out/'labels'/split/f'{i:04}.txt').write_text('\n'.join(labels)+'\n' if labels else '')
                manifest['samples'].append(dict(image=image,split=split,session_id=f'{split}_scene_{seed}',seed=seed,condition=condition,negative=negative,layout=dict(scene_meta),boxes=boxes,poses=poses))
                if a.gui:progress_label.text=f'Render {split}: {i+1}/{count}'
                print(f'RENDER {split} {i+1}/{count} labels={len(labels)}',flush=True)
        if a.video_frames:
            (out/'video').mkdir()
            for i in range(a.video_frames):
                setup(a.seed_base+a.train+a.val+a.test,'normal',phase=i*.05)
                pixels,labels,boxes,poses=capture();name=f'video/{i:04}.png'
                Image.fromarray(pixels).save(out/name)
                manifest['video'].append(dict(image=name,frame=i,session_id=f'video_scene_{a.seed_base+a.train+a.val+a.test}',boxes=boxes,poses=poses))
        stage.GetRootLayer().Export(str(out/'scene.usda'))
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
        import yaml
        (out/'dataset.yaml').write_text(yaml.safe_dump(dict(path=str(out),train='images/train',val='images/val',test='images/test',names=dict(enumerate(NAMES))),sort_keys=False))
        (out/'COMPLETE').write_text('Renderer generation completed\n')
        print('DATASET COMPLETE',out,flush=True)
        if a.gui and a.watch_training:
            setup(a.seed_base+42,'normal');rep.orchestrator.step(rt_subframes=4)
            rgb.detach([rp]);bbox.detach([rp]);rp.destroy()
            import csv
            while app.is_running():
                status=a.watch_training/'status.txt'
                progress_label.text=status.read_text().strip() if status.exists() else 'Dataset ready; waiting for training'
                try:
                    history=a.watch_training/'centerpose/history.json'
                    if history.exists():
                        row=json.loads(history.read_text())[-1]
                        metrics_label.text=f"CenterPose epoch {row['epoch']} | val loss {row['val_loss']:.4f}"
                    elif (a.watch_training/'yolo/results.csv').exists():
                        with (a.watch_training/'yolo/results.csv').open() as stream:rows=list(csv.DictReader(stream))
                        if rows:metrics_label.text=f"YOLO epoch {rows[-1]['epoch']} | mAP50 {rows[-1]['metrics/mAP50(B)']}"
                except (ValueError,IndexError,KeyError):pass
                app.update();time.sleep(.03)
    except BaseException:
        import traceback
        traceback.print_exc()
        raise
    finally:
        app.close(skip_cleanup=True,exit_code=0 if (out/'COMPLETE').exists() else 1)

if __name__=='__main__':main()
