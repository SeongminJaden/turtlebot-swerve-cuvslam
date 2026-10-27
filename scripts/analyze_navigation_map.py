"""Compare saved Nav2 costs with the supplied CAD map, for offline diagnosis."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt,label

p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args()
root=Path(__file__).resolve().parents[1]
c=json.loads((a.directory/'latest-costmap.json').read_text())
cost=np.asarray(c['data']).reshape(c['height'],c['width'])
static=np.flipud(np.asarray(Image.open(root/'assets/maze/localization/maze.pgm')))<100
if cost.shape!=static.shape or not np.allclose(c['origin'],[-.3,-.3]) or not np.isclose(c['resolution'],.01):
    raise ValueError('saved costmap does not match the CAD map grid')
distance=distance_transform_edt(~static)*c['resolution']
route=np.asarray(json.loads((root/'assets/maze/route_clearance.json').read_text())['route'])
xy=np.rint((route-np.asarray(c['origin']))/c['resolution']).astype(int)
def connected(free):
    groups,_=label(free)
    start=groups[xy[0,1],xy[0,0]];end=groups[xy[-1,1],xy[-1,0]]
    return bool(start and start==end)
novel=(cost==100)&~static
report={'costmap_stamp_ns':c['stamp_ns'],'frame_id':c['frame_id'],
        'static_connected_radius_0_315':connected(distance>.315),
        'combined_connected':connected((cost>=0)&(cost<99)),
        'extra_lethal_cells':int(novel.sum()),
        'extra_lethal_distance_from_cad_m_quantiles':dict(zip(['min','median','p90','max'],np.quantile(distance[novel],[0,.5,.9,1]).tolist())) if novel.any() else {},
        'scope':'offline grid comparison; distances are discrepancies, not measured sensor accuracy'}
(a.directory/'map-diagnostics.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
