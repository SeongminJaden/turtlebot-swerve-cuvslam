"""Procedural carton proxy: folds, cardboard fibers and approximate side printing.
Not an exact scan of Bambu packaging. Returns one 3x2 face texture atlas.
"""
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont

def create_carton_atlas(path,seed=713):
    rng=np.random.default_rng(seed);size=384
    noise=rng.normal(0,2.5,(size*2,size*3,1))
    rgb=np.clip(np.array([180,151,111])[None,None,:]+noise,0,255).astype('uint8')
    im=Image.fromarray(rgb);pen=ImageDraw.Draw(im)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',26)
    small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    for face in range(6):
        x,y=(face%3)*size,(face//3)*size
        pen.rectangle((x+3,y+3,x+size-4,y+size-4),outline=(120,98,68),width=2)
        pen.line((x+12,y+12,x+size-12,y+12),fill=(209,179,138),width=3)
        if face<4:
            pen.text((x+30,y+145),'Bambu Lab',font=font,fill=(47,49,40))
            pen.text((x+30,y+185),'FILAMENT  |  1 kg',font=small,fill=(55,53,40))
            pen.rectangle((x+28,y+230,x+135,y+280),outline=(75,66,46),width=2)
            pen.text((x+36,y+245),'PLA BASIC',font=small,fill=(70,60,42))
        else:
            pen.line((x+20,y+size-25,x+size-20,y+size-25),fill=(105,85,58),width=2)
            pen.line((x+25,y+20,x+25,y+size-20),fill=(136,110,75),width=2)
    Path(path).parent.mkdir(parents=True,exist_ok=True);im.save(path)

def cube_mesh_data():
    faces=[[(.5,-.5,-.5),(.5,.5,-.5),(.5,.5,.5),(.5,-.5,.5)],
           [(-.5,.5,-.5),(-.5,-.5,-.5),(-.5,-.5,.5),(-.5,.5,.5)],
           [(.5,.5,-.5),(-.5,.5,-.5),(-.5,.5,.5),(.5,.5,.5)],
           [(-.5,-.5,-.5),(.5,-.5,-.5),(.5,-.5,.5),(-.5,-.5,.5)],
           [(-.5,-.5,.5),(.5,-.5,.5),(.5,.5,.5),(-.5,.5,.5)],
           [(-.5,.5,-.5),(.5,.5,-.5),(.5,-.5,-.5),(-.5,-.5,-.5)]]
    uv=[]
    for i in range(6):
        x,y=i%3,i//3
        uv.extend([((x+.005)/3,(y+.005)/2),((x+.995)/3,(y+.005)/2),
                   ((x+.995)/3,(y+.995)/2),((x+.005)/3,(y+.995)/2)])
    return [p for f in faces for p in f],uv
