"""RealSense RGB: YOLO puck identification + CenterPose, with depth surface readout.
Camera optical coordinates: +X right, +Y down, +Z forward, meters.
Depth is a surface measurement, NOT ground truth for the puck geometric center.
"""
import argparse,json,time,os
from pathlib import Path
import cv2,numpy as np,torch
from ultralytics import YOLO
from train_centerpose import load_model,infer
ROOT=Path(__file__).resolve().parents[2]
WINDOW='RealSense | YOLO + CenterPose | Q quit, S snapshot'

def write_json(path,data):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2,allow_nan=False));tmp.replace(path)

def safe_pose(p):
    # Invalid neural/PnP outputs must not become positions in the live API.
    if p.get('translation_valid') and not np.isfinite(p.get('center_camera_m',[])).all():
        p['translation_valid']=False
    return p

class Perception:
    def __init__(self,run,detector_only=False):
        self.detector_only=detector_only
        self.config=json.loads((run/'inference.json').read_text())
        self.yolo=YOLO(str(run/'yolo/weights/best.pt'))
        self.pose=None if detector_only else load_model(run/'centerpose/best.pt')
        self.tags=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    def process(self,image,K,depth_m=None):
        started=time.perf_counter();canvas=cv2.copyMakeBorder(image,0,0,0,370,cv2.BORDER_CONSTANT,value=(25,25,25));rows=[];puck_index=0
        corners,ids,_=self.tags.detectMarkers(image)
        if ids is not None:cv2.aruco.drawDetectedMarkers(canvas,corners,ids)
        detections=self.yolo.predict(image,imgsz=self.config['imgsz'],conf=self.config['detector_confidence'],device=0,verbose=False)[0].boxes
        for b in detections:
            cls=int(b.cls[0]);box=b.xyxy[0].cpu().numpy();name=self.yolo.names[cls]
            row={'class_id':cls,'label':name,'confidence':float(b.conf[0]),'bbox_xyxy':box.tolist()}
            if cls==5:
                rows.append(row);continue
            x1,y1,x2,y2=map(int,box);color=(70,220,70) if cls<4 else (180,180,180)
            cv2.rectangle(canvas,(x1,y1),(x2,y2),color,2)
            text=f'{name} {row["confidence"]:.2f}'
            if cls<4:
                visible=[]
                if ids is not None:
                    for polygon,tag_id in zip(corners,ids.ravel()):
                        pts=polygon.reshape(-1,2)
                        if 0<=int(tag_id)<=11 and (pts[:,0]>=x1).all() and (pts[:,0]<=x2).all() and (pts[:,1]>=y1).all() and (pts[:,1]<=y2).all():visible.append(int(tag_id))
                visible=sorted(set(visible));row['visible_tag_ids']=visible
                panel_y=70+puck_index*85;puck_index+=1
                cv2.putText(canvas,f'{name} {row["confidence"]:.2f}',(image.shape[1]+12,panel_y),cv2.FONT_HERSHEY_SIMPLEX,.48,color,1)
                cv2.putText(canvas,'Tag IDs: '+(','.join(map(str,visible)) or 'not decoded'),(image.shape[1]+12,panel_y+20),cv2.FONT_HERSHEY_SIMPLEX,.43,color,1)
                if visible:text+=' tag:'+','.join(map(str,visible))
                if self.detector_only:
                    cv2.putText(canvas,text,(max(0,x1),max(18,y1-6)),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1,cv2.LINE_AA)
                    rows.append(row);continue
                try:
                    p=safe_pose(infer(self.pose,image,box,K,dimensions_m=(.03,.03,.06),views=self.config['pose_views']))
                    row['pose_confidence']=p['confidence'];row['translation_valid']=p['translation_valid'];row['rotation_candidate_valid']=p['rotation_valid']
                    if p['translation_valid']:
                        xyz=np.asarray(p['center_camera_m']);row['center_camera_m']=xyz.tolist()
                        uv=K@xyz;uv=uv[:2]/uv[2]
                        if np.isfinite(uv).all() and 0<=uv[0]<image.shape[1] and 0<=uv[1]<image.shape[0]:
                            u,v=np.rint(uv).astype(int);u=min(u,image.shape[1]-1);v=min(v,image.shape[0]-1)
                            cv2.drawMarker(canvas,(u,v),(0,255,255),cv2.MARKER_CROSS,16,2)
                            if depth_m is not None:
                                patch=depth_m[max(0,v-2):v+3,max(0,u-2):u+3];valid=patch[np.isfinite(patch)&(patch>0)]
                                if len(valid)>=5:
                                    row['surface_depth_m']=float(np.median(valid))
                                    # A coarse plausibility flag only: surface depth is not center ground truth.
                                    row['depth_consistent']=bool(abs(row['surface_depth_m']-xyz[2])<=.10)
                        row['verification']='DEPTH_CONFLICT' if row.get('depth_consistent') is False else 'UNVERIFIED'
                        label=('CP? ' if row['verification']=='DEPTH_CONFLICT' else 'CP ')+'XYZ mm: '+','.join(f'{value*1000:.0f}' for value in xyz)
                        cv2.putText(canvas,label,(image.shape[1]+12,panel_y+40),cv2.FONT_HERSHEY_SIMPLEX,.43,(0,255,255),1,cv2.LINE_AA)
                        if 'surface_depth_m' in row:
                            cv2.putText(canvas,f'Surface: {row["surface_depth_m"]*1000:.0f} mm'+(' DEPTH CONFLICT' if row['verification']=='DEPTH_CONFLICT' else ''),(image.shape[1]+12,panel_y+60),cv2.FONT_HERSHEY_SIMPLEX,.43,(180,180,180),1)
                        if p['rotation_valid']:row['rotation_candidate_matrix']=p['rotation_matrix']
                    else:text+=' CP invalid'
                except (cv2.error,ValueError,OverflowError) as error:
                    row['translation_valid']=False;row['pose_error']=str(error);text+=' CP error'
            cv2.putText(canvas,text,(max(0,x1),max(18,y1-6)),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1,cv2.LINE_AA);rows.append(row)
        torch.cuda.synchronize();elapsed=time.perf_counter()-started
        cv2.rectangle(canvas,(0,0),(canvas.shape[1],40),(25,25,25),-1)
        cv2.putText(canvas,f'{"YOLO detector only" if self.detector_only else "YOLO + CenterPose"}  {1/max(elapsed,.001):.1f} inference FPS',(8,25),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1)
        return canvas,{'mode':'detector_only' if self.detector_only else 'detector_and_pose','detections':rows,'inference_ms':elapsed*1000,'coordinates':'camera optical, meters; X right Y down Z forward','puck_dimensions_m':[.03,.03,.06],'depth_note':'surface depth is not geometric-center ground truth; rotation is an unvalidated candidate'}

def main():
    global WINDOW
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--detector-only',action='store_true')
    p.add_argument('--run',type=Path,default=ROOT/'runs/perception/train-v11-domain')
    p.add_argument('--output',type=Path,default=ROOT/'runs/perception/realsense-live')
    p.add_argument('--require-usb3',action='store_true');p.add_argument('--fps',type=int,choices=[6,15,30]);p.add_argument('--serial');p.add_argument('--headless',action='store_true');p.add_argument('--max-frames',type=int,default=0)
    p.add_argument('--wait-seconds',type=float,default=120);p.add_argument('--smoke-data',type=Path)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);torch.set_num_threads(4)
    if a.detector_only:WINDOW='RealSense | YOLO detector only | Q quit, S snapshot'
    model=Perception(a.run,detector_only=a.detector_only)
    if a.smoke_data:
        d=json.loads((a.smoke_data/'manifest.json').read_text());sample=next(x for x in d['samples'] if x['split']=='val' and not x['negative']);i=d['camera_intrinsics'];K=np.array([[i['fx'],0,i['cx']],[0,i['fy'],i['cy']],[0,0,1.]])
        canvas,result=model.process(cv2.imread(str(a.smoke_data/sample['image'])),K)
        result['source']='synthetic smoke test, NOT RealSense verification';write_json(a.output/'smoke.json',result);cv2.imwrite(str(a.output/'smoke.jpg'),canvas);print('SMOKE_OK',flush=True);return
    import pyrealsense2 as rs
    context=rs.context();start=time.monotonic();pipeline=None;count=0;last_write=0;total_pucks=0;valid_poses=0
    if not a.headless:cv2.namedWindow(WINDOW,cv2.WINDOW_NORMAL)
    try:
        while True:
            devices=[d for d in context.query_devices() if d.supports(rs.camera_info.serial_number) and (not a.serial or d.get_info(rs.camera_info.serial_number)==a.serial)]
            if a.require_usb3:
                devices=[d for d in devices if d.supports(rs.camera_info.usb_type_descriptor) and d.get_info(rs.camera_info.usb_type_descriptor).startswith('3')]
            if devices:break
            write_json(a.output/'status.json',{'status':'WAITING_FOR_USB3' if a.require_usb3 else 'WAITING_FOR_REALSENSE','elapsed_seconds':round(time.monotonic()-start,1)})
            if not a.headless:
                canvas=np.zeros((480,848,3),np.uint8);cv2.putText(canvas,'Connect RealSense USB 3 camera to this PC',(30,220),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2);cv2.imshow(WINDOW,canvas)
                if cv2.waitKey(250)&255 in (27,ord('q')):return
            else:time.sleep(.25)
            if time.monotonic()-start>a.wait_seconds:raise RuntimeError('RealSense not detected within wait timeout')
        device=devices[0];serial=device.get_info(rs.camera_info.serial_number)
        pipeline=rs.pipeline(context);config=rs.config();config.enable_device(serial)
        usb=device.get_info(rs.camera_info.usb_type_descriptor) if device.supports(rs.camera_info.usb_type_descriptor) else 'unknown'
        fps=a.fps or (15 if usb.startswith('2') else 30)
        config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,fps);config.enable_stream(rs.stream.depth,640,480,rs.format.z16,fps)
        profile=pipeline.start(config);intr=profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        K=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1.]])
        if intr.model in (rs.distortion.none,rs.distortion.brown_conrady) or not any(float(c)!=0 for c in intr.coeffs):
            maps=cv2.initUndistortRectifyMap(K,np.array(intr.coeffs),None,K,(intr.width,intr.height),cv2.CV_32FC1)
        elif intr.model==rs.distortion.inverse_brown_conrady:
            # Use this SDK's distortion implementation instead of treating inverse coefficients as OpenCV coefficients.
            coordinates=np.asarray([rs.rs2_project_point_to_pixel(intr,[(u-intr.ppx)/intr.fx,(v-intr.ppy)/intr.fy,1.])
                for v in range(intr.height) for u in range(intr.width)],dtype=np.float32).reshape(intr.height,intr.width,2)
            maps=(coordinates[:,:,0].copy(),coordinates[:,:,1].copy())
            residuals=[]
            for v in range(0,intr.height,40):
                for u in range(0,intr.width,40):
                    ray=rs.rs2_deproject_pixel_to_point(intr,coordinates[v,u].tolist(),1.)
                    residuals.append(np.linalg.norm((np.array(ray[:2])-[(u-intr.ppx)/intr.fx,(v-intr.ppy)/intr.fy])*[intr.fx,intr.fy]))
            if max(residuals)>.1:raise RuntimeError(f'SDK rectification round-trip exceeds 0.1px: {max(residuals)}')
            write_json(a.output/'rectification-check.json',{'max_roundtrip_px':float(max(residuals)),'samples':len(residuals),'method':'SDK projection/deprojection'})
        else:raise RuntimeError(f'Unsupported RGB distortion model: {intr.model}')
        scale=profile.get_device().first_depth_sensor().get_depth_scale();align=rs.align(rs.stream.color)
        write_json(a.output/'camera.json',{'name':device.get_info(rs.camera_info.name),'serial':serial,'K':K.tolist(),'distortion':str(intr.model),'coefficients':list(intr.coeffs),'rectified':True,'depth_scale':scale,'usb':usb,'requested_fps':fps})
        print('CAMERA_STARTED',serial,flush=True)
        while not a.max_frames or count<a.max_frames:
            frames=align.process(pipeline.wait_for_frames(15000 if count==0 else 5000));rgb=frames.get_color_frame();depth=frames.get_depth_frame()
            if not rgb or not depth:continue
            image=cv2.remap(np.asanyarray(rgb.get_data()),*maps,cv2.INTER_LINEAR)
            depth_m=cv2.remap(np.asanyarray(depth.get_data()).astype(np.float32)*scale,*maps,cv2.INTER_NEAREST)
            canvas,result=model.process(image,K,depth_m);count+=1
            pucks=[x for x in result['detections'] if x['class_id']<4];total_pucks+=len(pucks);valid_poses+=sum(x.get('translation_valid',False) for x in pucks)
            result.update(frame=count,camera_frame_number=rgb.get_frame_number(),camera_timestamp_ms=rgb.get_timestamp(),source='RealSense live',time_unix=time.time())
            if time.monotonic()-last_write>.5:
                cv2.imwrite(str(a.output/'latest-preview.jpg'),canvas);cv2.imwrite(str(a.output/'latest-rgb.png'),image);write_json(a.output/'latest.json',result);write_json(a.output/'status.json',{'status':'STREAMING','frames':count,'puck_detections':total_pucks,'valid_pose_estimates':valid_poses});last_write=time.monotonic()
            if not a.headless:
                cv2.imshow(WINDOW,canvas);key=cv2.waitKey(1)&255
                if key in (27,ord('q')):break
                if key==ord('s'):
                    name=a.output/f'capture-{time.time_ns()}';cv2.imwrite(str(name)+'.jpg',canvas);write_json(name.with_suffix('.json'),result)
                if cv2.getWindowProperty(WINDOW,cv2.WND_PROP_VISIBLE)<1:break
        write_json(a.output/'status.json',{'status':'STOPPED','frames':count,'puck_detections':total_pucks,'valid_pose_estimates':valid_poses})
    except Exception as error:
        write_json(a.output/'status.json',{'status':'ERROR','error':str(error),'frames':count});raise
    finally:
        if pipeline is not None:
            try:pipeline.stop()
            except RuntimeError:pass
        cv2.destroyAllWindows()

if __name__=='__main__':main()
