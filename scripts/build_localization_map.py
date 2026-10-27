"""Rasterize the supplied CAD maze for Nav2 map localization (no robot truth)."""
import json,math
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];m=json.loads((root/'assets/maze/maze.json').read_text());posts=np.array(m['posts']);resolution=.01
origin=np.floor((posts.min(0)-.3)/resolution)*resolution;size=np.ceil((posts.max(0)+.3-origin)/resolution).astype(int)
x=origin[0]+(np.arange(size[0])+.5)*resolution;y=origin[1]+(np.arange(size[1])+.5)*resolution;xx,yy=np.meshgrid(x,y);occupied=np.zeros(xx.shape,dtype=bool)
for px,py in posts:occupied|=(xx-px)**2+(yy-py)**2<(.03+resolution/2)**2
for w in m['panels']:
 c,s=np.cos(w['yaw']),np.sin(w['yaw']);dx,dy=xx-w['center'][0],yy-w['center'][1];u=c*dx+s*dy;v=-s*dx+c*dy
 occupied|=(np.abs(u)<w['length']/2+resolution/2)&(np.abs(v)<w['thickness']/2+resolution/2)
image=np.where(occupied,0,254).astype('uint8');folder=root/'assets/maze/localization';folder.mkdir(exist_ok=True)
(folder/'maze.pgm').write_bytes(f'P5\n{size[0]} {size[1]}\n255\n'.encode()+np.flipud(image).tobytes())
(folder/'maze.yaml').write_text(f'image: maze.pgm\nmode: trinary\nresolution: {resolution}\norigin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
print(folder/'maze.yaml',size.tolist())
