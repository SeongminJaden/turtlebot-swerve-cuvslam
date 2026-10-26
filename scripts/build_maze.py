"""Build metric USD collision geometry from the supplied assembly layout.
CAD floor X/Z becomes simulation X/Y; CAD height Y becomes simulation Z.
Panels are exact nominal boxes; posts/brackets use conservative D60 cylinders.
"""
import hashlib
import json
import math
from pathlib import Path
from collections import deque
ROOT = Path(__file__).resolve().parents[1]
source = ROOT/'assets/maze/source/twin-90-B-A1-R3-layout.json'
data = json.loads(source.read_text())
nodes = {n['id']: [v/1000 for v in n['xz_mm']] for n in data['nodes']}
parts = ['#usda 1.0\n(defaultPrim = "Maze"\n metersPerUnit = 1\n upAxis = "Z")\ndef Xform "Maze" {']
walls = []
for p in data['panels']:
    a, b = nodes[p['a']], nodes[p['b']]
    x,y = (a[0]+b[0])/2, (a[1]+b[1])/2
    angle = math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))
    length=p['length_mm']/1000
    walls.append({'id':p['id'],'center':[x,y],'length':length,'yaw':math.radians(angle),'thickness':.003})
    parts.append(f'''def Cube "{p['id']}" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {{
 double size = 1
 double3 xformOp:translate = ({x}, {y}, 0.2)
 float xformOp:rotateZ = {angle}
 double3 xformOp:scale = ({length}, 0.003, 0.38)
 uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ", "xformOp:scale"]
 color3f[] primvars:displayColor = [(0.32, 0.46, 0.62)]
}}''')
for name,(x,y) in nodes.items():
    parts.append(f'''def Cylinder "Post_{name}" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {{
 double radius = 0.03
 double height = 0.39
 uniform token axis = "Z"
 double3 xformOp:translate = ({x}, {y}, 0.195)
 uniform token[] xformOpOrder = ["xformOp:translate"]
 color3f[] primvars:displayColor = [(0.16, 0.18, 0.2)]
}}''')
parts.append('}')
(ROOT/'assets/maze/maze.usda').write_text('\n'.join(parts))
def clearance(x,y):
    d=min(math.hypot(x-a,y-b)-.03 for a,b in nodes.values())
    for w in walls:
        dx,dy=x-w['center'][0],y-w['center'][1]
        c,s=math.cos(w['yaw']),math.sin(w['yaw'])
        u,v=abs(c*dx+s*dy),abs(-s*dx+c*dy)
        d=min(d,math.hypot(max(u-w['length']/2,0),max(v-.0015,0)))
    return d
resolution=.05
free={(i,j) for i in range(1,89) for j in range(1,63) if clearance(i*resolution,j*resolution)>.30}
def bfs(start):
    previous={start:None}; queue=deque([start]); last=start
    while queue:
        last=queue.popleft()
        for delta in [(1,0),(-1,0),(0,1),(0,-1)]:
            p=(last[0]+delta[0],last[1]+delta[1])
            if p in free and p not in previous:previous[p]=last;queue.append(p)
    return last,previous
start,_=bfs(min(free));goal,previous=bfs(start)
route=[];p=goal
while p is not None:route.append([p[0]*resolution,p[1]*resolution]);p=previous[p]
route.reverse()
report={'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(), 'units':'metres',
        'axis_mapping':'CAD X,Z floor -> Sim X,Y; height -> Z','panels':walls,'posts':list(nodes.values()),
        'simplification':'Nominal panels; conservative D60 posts replace fasteners/mount detail',
        'start':route[0],'goal':route[-1],'route':route,'clearance_radius':.30,
        'route_length_m':(len(route)-1)*resolution,'unit':'first closed maze only; second unit remains in USD'}
(ROOT/'assets/maze/maze.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:report[k] for k in ['start','goal','route_length_m','simplification']},indent=2))
print('WALLS',len(walls),'POSTS',len(nodes),'REACHABLE_SAMPLES',len(previous))
