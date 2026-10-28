"""Compare cuVSLAM base pose against PhysX only for evaluation, not policy input."""
import json,argparse
from pathlib import Path
import numpy as np
from pose_math import matrix
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args()
r=[json.loads(l) for l in (a.directory/'steps.jsonl').read_text().splitlines()];valid=[x for x in r if x['base_estimate']]
summary={'frames':len(r),'tracking_success_frames':sum(x['tracking_ok'] for x in r),'forbidden_drive_frames':sum(x['drive_allowed'] and (not x['tracking_ok'] or x['sample_age']>.15) for x in r),'injected_drop_frames':sum(x.get('drop_images',False) for x in r),'drive_during_injected_drop':sum(x['drive_allowed'] and x.get('drop_images',False) for x in r)}
if valid:
 align=matrix(valid[0]['base_pose'])@np.linalg.inv(matrix(valid[0]['base_estimate']));errors=[];angles=[]
 for x in valid:
  est=align@matrix(x['base_estimate']);gt=matrix(x['base_pose']);errors.append(float(np.linalg.norm(est[:3,3]-gt[:3,3])));dr=est[:3,:3].T@gt[:3,:3];angles.append(float(np.arccos(np.clip((np.trace(dr)-1)/2,-1,1))))
 summary['translation_error_m']={k:float(np.percentile(errors,p)) for k,p in [('median',50),('p95',95),('max',100)]};summary['rotation_error_deg']={k:float(np.percentile(angles,p)*180/np.pi) for k,p in [('median',50),('p95',95),('max',100)]};summary['alignment']='single first-pose SE3 transform; no trajectory fitting or scale adjustment'
summary['per_segment']={n:{'frames':sum(x['segment']==n for x in r),'drive_frames':sum(x['segment']==n and x['drive_allowed'] for x in r)} for n in ['front','rear','right','left']}
(a.directory/'metrics.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
