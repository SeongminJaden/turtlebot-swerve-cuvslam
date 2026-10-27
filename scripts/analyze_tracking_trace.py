"""Evaluate VO using first-frame rigid alignment; never feeds truth to control."""
import argparse,json
from pathlib import Path
import numpy as np
from pose_math import matrix
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args()
rows=[json.loads(s) for s in (a.directory/'tracking-frames.jsonl').read_text().splitlines()]
errors=[];angles=[];align=None
for r in rows:
 if r.get('base_estimate') is None:continue
 est=matrix(r['base_estimate']);truth=matrix(r['ground_truth'])
 if align is None:align=truth@np.linalg.inv(est)
 prediction=align@est;errors.append(float(np.linalg.norm(prediction[:3,3]-truth[:3,3])))
 rot=truth[:3,:3].T@prediction[:3,:3];angles.append(float(np.rad2deg(np.arccos(np.clip((np.trace(rot)-1)/2,-1,1)))))
report={'frames':len(rows),'accepted':sum(r['tracking_ok'] for r in rows),'position_p95_m':float(np.percentile(errors,95)) if errors else None,'position_max_m':max(errors,default=None),'rotation_p95_deg':float(np.percentile(angles,95)) if angles else None,'scope':'all estimated frames including gate rejections; first-frame alignment only'}
(a.directory/'tracking-evaluation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
