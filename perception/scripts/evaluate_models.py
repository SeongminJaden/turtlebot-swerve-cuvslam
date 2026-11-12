"""Synthetic evaluation with separate oracle-crop and YOLO-to-pose metrics."""
import argparse,json,time
from pathlib import Path
import cv2,numpy as np,torch
from ultralytics import YOLO
from train_centerpose import load_model,infer
p=argparse.ArgumentParser();p.add_argument('--data',type=Path,required=True);p.add_argument('--run',type=Path,required=True);p.add_argument('--split',choices=['val','test'],default='test');p.add_argument('--imgsz',type=int,default=960);p.add_argument('--confidence',type=float,default=.25);p.add_argument('--pose-views',choices=['single','all7_mean','scale3_median'],default='single');a=p.parse_args()
torch.set_num_threads(4)
m=YOLO(str(a.run/'yolo/weights/best.pt'))
y=m.val(data=str(a.data/'dataset.yaml'),split=a.split,imgsz=a.imgsz,batch=4,device=0,workers=2,project=str(a.run),name=f'{a.split}-yolo',plots=True)
result={'scope':'synthetic frames; separate oracle and YOLO-to-pose metrics, known dimensions; not full mission or real-world validation','inference':{'imgsz':a.imgsz,'pose_views':a.pose_views,'detector_confidence':a.confidence},'yolo':{k:float(v) for k,v in y.results_dict.items()},'yolo_per_class_ap50':{m.names[i]:float(v) for i,v in zip(y.box.ap_class_index,y.box.ap50)}}
model=load_model(a.run/'centerpose/best.pt');manifest=json.loads((a.data/'manifest.json').read_text());intr=manifest['camera_intrinsics'];K=np.array([[intr['fx'],0,intr['cx']],[0,intr['fy'],intr['cy']],[0,0,1.]])
rows=[];end_to_end=[];frame_times=[]
for sample in manifest['samples']:
    if sample['split']!=a.split:continue
    image=cv2.imread(str(a.data/sample['image']))
    torch.cuda.synchronize();frame_start=time.perf_counter()
    detections=m.predict(image,imgsz=a.imgsz,device=0,conf=a.confidence,verbose=False)[0].boxes
    torch.cuda.synchronize();detector_ms=(time.perf_counter()-frame_start)*1000;pose_ms=0
    for pose in sample['poses']:
        if not pose.get('pose_supervision_valid',True):continue
        boxes=[b for b in sample['boxes'] if b['class_id']==pose['class_id']]
        if not boxes:continue
        est=infer(model,image,boxes[0]['xyxy'],K,dimensions_m=pose.get('dimensions_m',[.03,.03,.06]),views=a.pose_views)
        error=float(np.linalg.norm(np.array(est['corners_px'])-np.array(pose['projected_cuboid'])[1:],axis=1).mean())
        row={'image':sample['image'],'class_id':pose['class_id'],'valid':est['translation_valid'],'rotation_candidate_valid':est['rotation_valid'],'ordered_corner_error_px':error}
        if est['translation_valid']:row['translation_error_mm']=float(np.linalg.norm(np.array(est['center_camera_m'])-pose['center_camera_m'])*1000)
        rows.append(row)
        candidates=[b for b in detections if int(b.cls[0])==pose['class_id']]
        e2e={'image':sample['image'],'class_id':pose['class_id'],'detected':bool(candidates),'valid':False}
        if candidates:
            detected=max(candidates,key=lambda b:float(b.conf[0]))
            torch.cuda.synchronize();pose_start=time.perf_counter()
            prediction=infer(model,image,detected.xyxy[0].cpu().numpy(),K,dimensions_m=pose.get('dimensions_m',[.03,.03,.06]),views=a.pose_views)
            torch.cuda.synchronize();pose_ms+=(time.perf_counter()-pose_start)*1000
            e2e['valid']=prediction['translation_valid']
            e2e['rotation_candidate_valid']=prediction['rotation_valid']
            if prediction['translation_valid']:e2e['translation_error_mm']=float(np.linalg.norm(np.array(prediction['center_camera_m'])-pose['center_camera_m'])*1000)
        end_to_end.append(e2e)
    frame_times.append(detector_ms+pose_ms)
valid=[r['translation_error_mm'] for r in rows if r['valid']]
result['evaluation_split']=a.split
result['pose_exclusions']={'deformed_nonrigid_boxes':sum(not p.get('pose_supervision_valid',True) for s in manifest['samples'] if s['split']==a.split for p in s['poses']),'reason':'no rigid-pose ground truth; included in YOLO detection metrics'}
result['latency']={'scope':'detector plus pose for visible target classes; excludes oracle pass and disk reads; concurrent GPU workloads may affect timing','mean_ms_per_frame':float(np.mean(frame_times)),'median_ms_per_frame':float(np.median(frame_times))}
end_errors=[r['translation_error_mm'] for r in end_to_end if r['valid']]
result['end_to_end']={'scope':'highest confidence YOLO box for each class, calibrated RGB, known object dimensions; misses count as failures','objects':len(end_to_end),'valid_objects':len(end_errors),'median_mm_valid_only':float(np.median(end_errors)) if end_errors else None,'p90_mm_valid_only':float(np.percentile(end_errors,90)) if end_errors else None,'all_objects_within_10mm':sum(r.get('translation_error_mm',float('inf'))<=10 for r in end_to_end)/len(end_to_end) if end_to_end else 0,'measurements':end_to_end}
result['centerpose_translation_source']='learned scale-normalized depth' if getattr(model,'metric_depth_head',False) else 'PnP'
result['centerpose_all_objects_within_10mm']=sum(r.get('translation_error_mm',float('inf'))<=10 for r in rows)/len(rows) if rows else 0
result['centerpose_per_class']={str(c):{'count':len([r for r in rows if r['class_id']==c]),'median_mm_valid_only':float(np.median([r['translation_error_mm'] for r in rows if r['class_id']==c and r['valid']]))} for c in range(5)}
result['centerpose']={'test_objects':len(rows),'valid_objects':len(valid),'valid_fraction':len(valid)/len(rows) if rows else 0,'ordered_corner_error_px_mean':float(np.mean([r['ordered_corner_error_px'] for r in rows])) if rows else None,'translation_error_mm_median_valid_only':float(np.median(valid)) if valid else None,'translation_error_mm_p90_valid_only':float(np.percentile(valid,90)) if valid else None,'symmetry_warning':'same object ID on multiple faces can leave pose ambiguities; ordered-corner metric is strict','measurements':rows}
(a.run/f'{a.split}-evaluation.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ('centerpose','end_to_end')},indent=2));print({k:v for k,v in result['centerpose'].items() if k!='measurements'})
