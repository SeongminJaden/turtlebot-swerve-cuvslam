"""Independent readiness and RGB checks. A failed prerequisite blocks completion."""
import itertools
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
from pathlib import Path


def reachability(root,stage):
    from pxr import UsdGeom, Usd
    from scipy.optimize import differential_evolution
    from turtlebot_swerve.arm_kinematics import rotation
    urdf=ET.parse(root/'assets/mission/R12_NO_USB_URDF_USD/urdf/r12_no_usb.urdf').getroot()
    chain=[]
    for i in range(1,6):
        joint=urdf.find(f"joint[@name='arm_joint{i}']")
        chain.append((np.fromstring(joint.find('origin').get('xyz'),sep=' '), np.fromstring(joint.find('axis').get('xyz'),sep=' '),tuple(float(joint.find('limit').get(k)) for k in ('lower','upper'))))
    bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render']).ComputeUntransformedBound(stage.GetPrimAtPath('/World/Robot/Asset/arm_link5')).ComputeAlignedRange()
    vertices=np.array(list(itertools.product(*zip(bounds.GetMin(),bounds.GetMax()))))
    def height(q):
        t=np.zeros(3);r=np.eye(3)
        for (xyz,axis,_),angle in zip(chain,q):
            t+=r@xyz;r=r@rotation(axis,angle)
        return float(np.min((vertices@r.T+t)[:,2]))
    opt=differential_evolution(height,[item[2] for item in chain],seed=42,tol=1e-9,popsize=12)
    # Conservative analytic lower bound for ALL points in the last link's AABB.
    # q2 in ±.35, q2+q3 in ±.70, q2+q3+q4 in ±1.05.
    expected=[(0,0,.275),(0,0,.0595),(.024,0,.1854),(.149,0,0),(.06635,0,0)]
    assert all(np.allclose(item[0],v) for item,v in zip(chain,expected))
    assert all(np.allclose(item[1],axis) for item,axis in zip(chain,[(0,0,1),(0,-1,0),(0,-1,0),(0,-1,0),(1,0,0)]))
    assert all(np.allclose(item[2],(-.35,.35)) for item in chain[1:])
    radius=float(np.linalg.norm(vertices,axis=1).max())
    lower=.3345+.1854*math.cos(.35)-.024*math.sin(.35)-.149*math.sin(.70)-.06635*math.sin(1.05)-radius
    return {'status':'BLOCKED_UNREACHABLE' if lower>.13 else 'NEEDS_IK_CHECK','assumption':'level chassis at base Z=0; original joint limits; end-link AABB conservatively encloses all gripper geometry','puck_top_m':.13,'analytic_min_height_lower_bound_m':lower,'optimized_aabb_min_height_m':opt.fun,'optimizer_joint_rad':opt.x.tolist(),'end_link_radius_bound_m':radius,'gripper_actuated':any('grip' in j.get('name','').lower() and j.get('type')!='fixed' for j in urdf.findall('joint'))}


def run(root,stage,world,output,practice):
    from pxr import UsdGeom,UsdLux,UsdShade,Sdf,Gf
    import omni.replicator.core as rep
    import zxingcpp
    import cv2
    from PIL import Image
    from turtlebot_swerve.dice_mission import SignalRecorder,plan
    result={'mission_complete':False,'reachability':reachability(root,stage)}
    output.mkdir(exist_ok=True,parents=True)
    (output/'audit.json').write_text(json.dumps(result,indent=2))
    UsdLux.DomeLight.Define(stage,'/World/AuditLight').CreateIntensityAttr(1500.)
    def material(name,color):
        mat=UsdShade.Material.Define(stage,'/World/AuditMaterials/'+name)
        sh=UsdShade.Shader.Define(stage,mat.GetPath().AppendChild('Shader'))
        sh.CreateIdAttr('UsdPreviewSurface')
        sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(1.)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
        return mat,sh
    white,_=material('White',(.95,.95,.95));black,_=material('Black',(.002,.002,.002))
    lampmat,lampsh=material('Lamp',(.5,0,0))
    def cube(name,xyz,size,mat):
        obj=UsdGeom.Cube.Define(stage,'/World/SignalStation/'+name)
        obj.CreateSizeAttr(1.)
        obj.AddTranslateOp().Set(Gf.Vec3d(*xyz));obj.AddScaleOp().Set(Gf.Vec3d(*size))
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
    cube('Housing',(.065,.95,.35),(.03,.16,.70),black)
    cube('Lamp',(.083,.95,.65),(.006,.10,.10),lampmat)
    barcode=zxingcpp.create_barcode(practice['barcode'],zxingcpp.BarcodeFormat.Code128)
    raster=np.asarray(zxingcpp.write_barcode_to_image(barcode,scale=1,add_hrt=False))
    row=raster[raster.shape[0]//2];width=.32
    cube('BarcodePanel',(.08,.65,.45),(.01,width,.16),white)
    for i,pixel in enumerate(row):
        if pixel==0:
            cube(f'Bar{i}',(.086,.65-width/2+width*(i+.5)/len(row),.45),(.002,width/len(row),.14),black)
    camera='/World/Robot/Asset/base_link/r07_camera1/r07_camera1_left_optical/Camera'
    product=rep.create.render_product(camera,(1280,960))
    ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach(product)
    palette={'R':(1.,0.,0.),'Y':(1.,.8,0.),'G':(0.,1.,0.),'OFF':(0.,0.,0.)}
    def frame(color):
        lampsh.GetInput('diffuseColor').Set(Gf.Vec3f(*palette[color]))
        lampsh.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*palette[color]))
        rep.orchestrator.step(rt_subframes=2,delta_time=0.)
        return ann.get_data()[:,:,:3].copy()
    for _ in range(3):rgb=frame('R')
    Image.fromarray(rgb).save(output/'station-camera.png')
    codes=zxingcpp.read_barcodes(rgb)
    observed=[c.text for c in codes]
    result['barcode']={'observed':observed,'expected':practice['barcode'],'passed':observed==[practice['barcode']],'evidence':'station-camera.png','dimensions_assumed_m':[.32,.16]}
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    mask=(((hsv[:,:,0]<12)|(hsv[:,:,0]>170))&(hsv[:,:,1]>150)&(hsv[:,:,2]>100)).astype(np.uint8)
    n,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
    roi=None
    if n>1:
        k=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA]))
        if stats[k,cv2.CC_STAT_AREA]>25:
            x,y,w,h,_=stats[k];roi=(int(x),int(y),int(w),int(h))
    result['signal_roi']=roi
    if roi:
        x,y,w,h=roi
        def classify(image):
            crop=cv2.cvtColor(image[y:y+h,x:x+w],cv2.COLOR_RGB2HSV)
            sat=crop[:,:,1]>100;lit=crop[:,:,2]>90
            active=sat&lit
            if active.mean()<.15:return 'OFF'
            hue=crop[:,:,0][active]
            shares={'R':float(((hue<12)|(hue>170)).mean()),'Y':float(((hue>=12)&(hue<40)).mean()),'G':float(((hue>=40)&(hue<90)).mean())}
            best=max(shares,key=shares.get)
            return best if shares[best]>.8 else 'UNKNOWN'
        # Render every color including OFF independently, then the whole 27-second sequence.
        recorder=SignalRecorder(max_gap=.26);observations=[]
        (output/'frames').mkdir(exist_ok=True)
        start_time=float(world.current_time)
        try:
            for tick in range(109):
                if tick:
                    world.play()
                    for _ in range(30):world.step(render=False)
                t=tick/4
                stimulus=practice['sequence'][int(t)//2] if t<24 and t%2<1 else 'OFF'
                rgb=frame(stimulus);detected=classify(rgb)
                Image.fromarray(rgb).save(output/'frames'/f'{tick:04d}.png')
                sample_time=float(world.current_time)-start_time
                observations.append({'time_s':sample_time,'observed':detected})
                try:recorder.observe(sample_time,detected)
                except ValueError:break
                if tick%20==0:print('MISSION_RGB_PROGRESS',tick,flush=True)
        finally:pass
        result['signal']={'complete':recorder.complete,'sequence':recorder.sequence,'error':recorder.error,'passed':recorder.complete and ''.join(recorder.sequence)==practice['sequence'],'observations':observations,'timestamp_basis':'actual World simulation clock, 30 physics steps per 4 Hz camera sample; static robot; render delta_time=0'}
        if result['signal']['passed'] and result['barcode']['passed']:
            result['observed_signal_plan']=plan(recorder.sequence,observed[0],practice['pucks'])
            result['observed_signal_plan']['inventory_source']='synthetic registered inventory, not camera perception'
    else:result['signal']={'passed':False,'error':'No red lamp visible in onboard image'}
    # Corroborate the low-height configuration with actual joint drives.
    from pxr import UsdPhysics, Usd
    world.play()
    from isaacsim.core.prims import SingleArticulation
    robot=SingleArticulation('/World/Robot/Asset',name='audit_readback')
    robot.initialize()
    q=result['reachability']['optimizer_joint_rad']
    for i,angle in enumerate(q,1):
        drive=UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f'/World/Robot/Asset/Joints/arm_joint{i}'),'angular')
        drive.GetTargetPositionAttr().Set(math.degrees(angle))
    for _ in range(600):world.step(render=False)
    low=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render']).ComputeWorldBound(stage.GetPrimAtPath('/World/Robot/Asset/arm_link5')).ComputeAlignedRange().GetMin()[2]
    result['reachability']['physx_low_pose_aabb_min_z_m']=float(low)
    measured=dict(zip(robot.dof_names,[float(v) for v in robot.get_joint_positions()]))
    result['reachability']['physx_joint_readback_rad']={f'arm_joint{i}':measured[f'arm_joint{i}'] for i in range(1,6)}
    result['reachability']['low_pose_targets_reached']=all(abs(measured[f'arm_joint{i}']-angle)<.02 for i,angle in enumerate(q,1))
    for i in range(1,6):
        UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f'/World/Robot/Asset/Joints/arm_joint{i}'),'angular').GetTargetPositionAttr().Set(0.)
    for _ in range(240):world.step(render=False)
    result['remaining']=['camera puck IDs/color/shape','contact grasp with actuated gripper','sensor closed-loop navigation','dice placement and independent reinspection','whole-body return-zone containment']
    result['status']='BLOCKED_FULL_MISSION'
    (output/'audit.json').write_text(json.dumps(result,indent=2))
    ann.detach(product);product.destroy()
    return result
