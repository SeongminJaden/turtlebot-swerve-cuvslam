"""A* with an axis-aligned swerve body footprint; fixed base yaw is deliberate.
Clearance includes the measured USD collision envelope plus a provisional margin.
Original CAD map and its historical route remain unchanged.
"""
import heapq,json,math,argparse
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--half-width',type=float,default=.245);p.add_argument('--start',nargs=2,type=float);p.add_argument('--goal',nargs=2,type=float);p.add_argument('--output',type=Path,default=Path('assets/maze/route_footprint.json'));a=p.parse_args()
root=Path(__file__).resolve().parents[1];m=json.loads((root/'assets/maze/maze.json').read_text());res=.025;half=a.half_width
if a.start:m['start']=a.start
if a.goal:m['goal']=a.goal
xx,yy=np.meshgrid(np.arange(0,4.41,res),np.arange(0,3.21,res));free=np.ones(xx.shape,dtype=bool)
for x,y in m['posts']:
 dx=np.maximum(np.abs(xx-x)-half,0);dy=np.maximum(np.abs(yy-y)-half,0);free&=dx*dx+dy*dy>(.03+.005)**2
for w in m['panels']:
 # Supplied maze has orthogonal panels. Fail rather than silently approximate diagonals.
 assert abs(math.sin(2*w['yaw']))<1e-6
 hx=abs(math.cos(w['yaw']))*w['length']/2+abs(math.sin(w['yaw']))*w['thickness']/2
 hy=abs(math.sin(w['yaw']))*w['length']/2+abs(math.cos(w['yaw']))*w['thickness']/2
 free&=~((np.abs(xx-w['center'][0])<hx+half+.005)&(np.abs(yy-w['center'][1])<hy+half+.005))
start=tuple(np.rint(np.asarray(m['start'])/res).astype(int));goal=tuple(np.rint(np.asarray(m['goal'])/res).astype(int))
assert free[start[1],start[0]] and free[goal[1],goal[0]]
# Clearance preference steers routes away from inflated boundaries.
from scipy.ndimage import distance_transform_edt
clearance=distance_transform_edt(free)*res
queue=[(0.,start)];cost={start:0.};parent={start:None}
while queue:
 _,u=heapq.heappop(queue)
 if u==goal:break
 for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
  v=(u[0]+dx,u[1]+dy)
  if not (0<=v[0]<free.shape[1] and 0<=v[1]<free.shape[0] and free[v[1],v[0]]):continue
  if dx and dy and not(free[u[1],v[0]] and free[v[1],u[0]]):continue
  step=math.hypot(dx,dy)*res*(1.+.03/max(clearance[v[1],v[0]],.01));new=cost[u]+step
  if new<cost.get(v,float('inf')):
   cost[v]=new;parent[v]=u;heapq.heappush(queue,(new+res*math.hypot(v[0]-goal[0],v[1]-goal[1]),v))
if goal not in parent:raise RuntimeError('No safe footprint route')
route=[];u=goal
while u is not None:route.append(np.asarray(u)*res);u=parent[u]
route=np.asarray(route[::-1]);arc=np.r_[0,np.linalg.norm(np.diff(route,axis=0),axis=1).cumsum()]
samples=np.arange(0,arc[-1],.05);out=np.stack([np.interp(samples,arc,route[:,k]) for k in range(2)],axis=1);out=np.vstack([out,route[-1]])
report={'route':out.tolist(),'route_length_m':float(arc[-1]),'footprint_half_width_m':half,'extra_obstacle_margin_m':.005,'base_yaw_assumption_rad':0.,'grid_resolution_m':res,'sample_spacing_m':.05,'scope':'first closed maze; supplied CAD walls and posts'}
(root/a.output).write_text(json.dumps(report,indent=2));print({k:v for k,v in report.items() if k!='route'})
