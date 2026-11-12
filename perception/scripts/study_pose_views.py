"""Validation-only comparison of crop views; no test frames or model updates."""
import argparse,json,sys
from pathlib import Path
import cv2,numpy as np,torch
from ultralytics import YOLO
from train_centerpose import load_model,infer_single
p=argparse.ArgumentParser();p.add_argument('--data',type=Path,required=True);p.add_argument('--yolo',type=Path,required=True);p.add_argument('--pose',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--imgsz',type=int,default=640);a=p.parse_args()
torch.set_num_threads(2)
manifest=json.loads((a.data/'manifest.json').read_text());i=manifest['camera_intrinsics'];K=np.array([[i['fx'],0,i['cx']],[0,i['fy'],i['cy']],[0,0,1.]])
model=load_model(a.pose);detector=YOLO(str(a.yolo));rows=[]
views=[(1.5,(0,0)),(1.35,(0,0)),(1.65,(0,0)),(1.5,(-.04,0)),(1.5,(.04,0)),(1.5,(0,-.04)),(1.5,(0,.04))]
for sample in manifest['samples']:
 if sample['split']!='val':continue
 image=cv2.imread(str(a.data/sample['image']));detections=detector.predict(image,imgsz=a.imgsz,conf=.05,verbose=False,device=0)[0].boxes
 for pose in sample['poses']:
  if not any(b['class_id']==pose['class_id'] for b in sample['boxes']):continue
  row={'image':sample['image'],'class_id':pose['class_id'],'truth':pose['center_camera_m'],'views':[]}
  candidates=[b for b in detections if int(b.cls[0])==pose['class_id']]
  if candidates:
   detected=max(candidates,key=lambda b:float(b.conf[0]));row['detector_confidence']=float(detected.conf[0]);box=detected.xyxy[0].cpu().numpy()
   for scale,shift in views:
    est=infer_single(model,image,box,K,dimensions_m=pose.get('dimensions_m',[.03,.03,.06]),crop_scale=scale,shift=shift)
    row['views'].append({'point':est.get('center_camera_m'),'confidence':est['confidence'],'pnp':est.get('pnp_center_camera_m'),'pnp_valid':est.get('pnp_valid',False)})
  rows.append(row)
 print(sample['image'],flush=True)
summary={}
for confidence in (.05,.10,.25):
 for name,indices in [('single',[0]),('scale3',[0,1,2]),('shift5',[0,3,4,5,6]),('all7',list(range(7)))]:
  for reducer in ('mean','median'):
   errors=[];missing=0
   for r in rows:
    if r.get('detector_confidence',0)<confidence:missing+=1;continue
    points=[r['views'][j]['point'] for j in indices if r['views'][j]['confidence']>=.25 and r['views'][j]['point'] is not None]
    if not points:missing+=1;continue
    point=getattr(np,reducer)(points,axis=0);errors.append(float(np.linalg.norm(point-r['truth'])*1000))
   summary[f'{confidence}_{name}_{reducer}']={'median_mm':float(np.median(errors)),'p90_mm':float(np.percentile(errors,90)),'within10_all':sum(e<=10 for e in errors)/len(rows),'missing':missing}
a.output.write_text(json.dumps({'scope':'validation only','views':views,'summary':summary,'rows':rows},indent=2))
print(json.dumps(summary,indent=2),flush=True)
