"""Box recall by deformation on a frozen split, retaining missed detections."""
import argparse,json
from collections import defaultdict
from pathlib import Path
from ultralytics import YOLO
from compare_real_frame import iou

def main():
    p=argparse.ArgumentParser();p.add_argument('--data',type=Path,required=True);p.add_argument('--weights',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--split',choices=['val','test'],default='test');a=p.parse_args()
    manifest=json.loads((a.data/'manifest.json').read_text());model=YOLO(str(a.weights));groups=defaultdict(lambda:dict(objects=0,true_positives=0,false_positives=0));rows=[]
    for s in manifest['samples']:
        if s['split']!=a.split:continue
        gt=[b['xyxy'] for b in s['boxes'] if b['class_id']==4]
        pred=model.predict(str(a.data/s['image']),imgsz=800,conf=.25,device=0,verbose=False)[0]
        detections=sorted([b for b in pred.boxes if int(b.cls[0])==4],key=lambda b:float(b.conf[0]),reverse=True)
        unmatched=list(range(len(gt)));tp=0
        for det in detections:
            pairs=[(iou(det.xyxy[0].cpu().tolist(),gt[j]),j) for j in unmatched]
            overlap,j=max(pairs) if pairs else (0,None)
            if overlap>=.5:tp+=1;unmatched.remove(j)
        mode=s['layout']['box_deformation']['mode'];g=groups[mode];g['objects']+=len(gt);g['true_positives']+=tp;g['false_positives']+=len(detections)-tp
        rows.append(dict(image=s['image'],mode=mode,objects=len(gt),true_positives=tp,false_positives=len(detections)-tp))
    for g in groups.values():
        g['recall']=g['true_positives']/g['objects'] if g['objects'] else None
        n=g['true_positives']+g['false_positives'];g['precision']=g['true_positives']/n if n else None
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(dict(scope='Synthetic box detection at confidence .25 and IoU .50; small per-mode groups, not real-world accuracy',weights=str(a.weights),split=a.split,groups=dict(groups),frames=rows),indent=2))
    print(json.dumps(dict(groups),indent=2))
if __name__=='__main__':main()
