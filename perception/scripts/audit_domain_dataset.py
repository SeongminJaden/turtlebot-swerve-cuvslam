"""Structural audit of camera, lighting, deformation and split coverage (no model scores)."""
import argparse,json,math
from pathlib import Path
from collections import Counter
import numpy as np
p=argparse.ArgumentParser();p.add_argument('data',type=Path);a=p.parse_args();d=json.loads((a.data/'manifest.json').read_text())
def interval(values):return [float(min(values)),float(max(values))]
result={}
for split in ('train','val','test'):
 rows=[r for r in d['samples'] if r['split']==split]
 if not rows:continue
 layouts=[r['layout'] for r in rows];modes=Counter(x['box_deformation']['mode'] for x in layouts)
 centers=[np.r_[x['box_center_xy_m'],0.] for x in layouts];eyes=[np.array(x['camera_eye']) for x in layouts]
 result[split]={'frames':len(rows),'deformation_counts':dict(modes),'camera_distance_to_box_m':interval([np.linalg.norm(e-c) for e,c in zip(eyes,centers)]),'camera_height_m':interval([e[2] for e in eyes]),'camera_roll_deg':interval([x['camera_roll_deg'] for x in layouts]),'dome_kelvin':interval([x['lighting']['dome_kelvin'] for x in layouts]),'sun_kelvin':interval([x['lighting']['sun_kelvin'] for x in layouts]),'dome_intensity_renderer_units':interval([x['lighting']['dome_intensity'] for x in layouts]),'sun_intensity_renderer_units':interval([x['lighting']['sun_intensity'] for x in layouts]),'partially_framed_boxes':sum(any(b['class_id']==4 and (b['xyxy'][0]==0 or b['xyxy'][1]==0 or b['xyxy'][2]==640 or b['xyxy'][3]==640) for b in r['boxes']) for r in rows),'nonrigid_box_pose_exclusions':sum(not p.get('pose_supervision_valid',True) for r in rows for p in r['poses'])}
 assert set(modes)=={'intact','side_dent','corner_dent','top_sag','compressed'},(split,modes)
 assert result[split]['camera_roll_deg'][0]<-10 and result[split]['camera_roll_deg'][1]>10
result['scope']='structural coverage only; renderer intensities are not measured lux; deformed boxes remain detector targets'
(a.data/'domain-audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
