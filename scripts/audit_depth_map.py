"""Offline depth/CAD consistency scoring using truth only for evaluation."""
import argparse,json
from pathlib import Path
import numpy as np
from pose_math import matrix
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args()
m=json.loads(Path('assets/maze/maze.json').read_text());centers=np.array([w['center'] for w in m['panels']]);axes=np.array([[np.cos(w['yaw']),np.sin(w['yaw'])] for w in m['panels']]);length=np.array([w['length'] for w in m['panels']]);posts=np.array(m['posts']);errors=[];frames=[]
for line in (a.directory/'tracking-frames.jsonl').read_text().splitlines():
 r=json.loads(line);pts=np.asarray(r.get('depth_obstacles_base',[])).reshape(-1,2)
 if not len(pts):continue
 # Physics remains nearly planar; score XY wall/post surface proximity.
 t=matrix(r['ground_truth']);pts=pts@t[:2,:2].T+t[:2,3];d=pts[:,None,:]-centers
 along=(d*axes).sum(-1);nearest=centers+np.clip(along,-length/2,length/2)[:,:,None]*axes
 wall=np.linalg.norm(pts[:,None,:]-nearest,axis=-1).min(axis=1)
 post=np.abs(np.linalg.norm(pts[:,None,:]-posts,axis=-1)-.03).min(axis=1)
 e=np.minimum(wall,post);errors.extend(e.tolist());frames.append({'stamp_ns':r['stamp_ns'],'points':len(e),'median_m':float(np.median(e)),'p95_m':float(np.percentile(e,95))})
report={'scope':'offline scoring only; XY CAD wall/post surface residual','points':len(errors),'median_m':float(np.median(errors)) if errors else None,'p95_m':float(np.percentile(errors,95)) if errors else None,'frames':frames}
(a.directory/'depth-map-audit.json').write_text(json.dumps(report,indent=2));print({k:v for k,v in report.items() if k!='frames'})
