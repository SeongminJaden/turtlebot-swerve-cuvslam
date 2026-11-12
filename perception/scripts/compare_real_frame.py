"""Paired detector diagnostic on one manually boxed frame; not an accuracy benchmark."""
import argparse,json
from pathlib import Path
import cv2
from ultralytics import YOLO

def iou(a,b):
    w=max(0,min(a[2],b[2])-max(a[0],b[0]));h=max(0,min(a[3],b[3])-max(a[1],b[1]))
    intersection=w*h
    return intersection/max(1e-9,(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection)

def main():
    p=argparse.ArgumentParser();p.add_argument('--reference',type=Path,required=True);p.add_argument('--weights',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    ref=json.loads(a.reference.read_text());image=cv2.imread(str(a.reference.parent/ref['image']))
    result=YOLO(str(a.weights)).predict(image,imgsz=800,conf=.25,device=0,verbose=False)[0]
    detections=[{'class_id':int(b.cls[0]),'confidence':float(b.conf[0]),'xyxy':b.xyxy[0].cpu().tolist()} for b in result.boxes]
    matches=[]
    for gt in ref['annotations']:
        candidates=[(iou(gt['xyxy'],d['xyxy']),d) for d in detections if d['class_id']==gt['class_id']]
        overlap,det=max(candidates,key=lambda x:x[0]) if candidates else (0,None)
        matches.append({'class_id':gt['class_id'],'iou_with_approximate_manual_box':overlap,'matched_at_iou50':overlap>=.5,'detection':det})
    a.output.mkdir(parents=True,exist_ok=True)
    cv2.imwrite(str(a.output/'overlay.jpg'),result.plot())
    (a.output/'diagnostic.json').write_text(json.dumps({'scope':ref['scope'],'weights':str(a.weights),'matches':matches,'detections':detections},indent=2))
    print(json.dumps(matches,indent=2))
if __name__=='__main__':main()
