"""Isaac Sim 6.1 stereo rendering smoke test; native USD + Replicator API."""
from pathlib import Path
import json,argparse
from importlib.metadata import version
assert version('isaacsim')=='6.1.0.0', 'This runner requires Isaac Sim 6.1.0.0'
parser=argparse.ArgumentParser()
parser.add_argument('--headless',action='store_true')
args=parser.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':args.headless,'width':640,'height':480})
import omni.usd
import omni.replicator.core as rep
import numpy as np
from PIL import Image
from pxr import UsdGeom,UsdLux,UsdPhysics,Gf
ROOT=Path(__file__).resolve().parents[1]
(ROOT/'checks').mkdir(exist_ok=True)
try:
    omni.usd.get_context().open_stage(str(ROOT/'assets/m38_stereo.usda'))
    for _ in range(10): app.update()
    stage=omni.usd.get_context().get_stage()
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(120)
    # A render-only test must not let the robot free-fall between frames.
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
    UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1800.)
    for i in range(35):
        cube=UsdGeom.Cube.Define(stage,f'/World/Marker{i}')
        cube.CreateSizeAttr(.12+(i%3)*.04)
        cube.AddTranslateOp().Set(Gf.Vec3d(1.+(i%3)*.3,(i%7-3)*.25,.15+(i//7)*.23))
        cube.CreateDisplayColorAttr([Gf.Vec3f((i%3)/3.,(i%5)/5.,(i%7)/7.)])
    annotators=[]
    for side in ['left','right']:
        product=rep.create.render_product(f'/World/Robot/base_link/sim_{side}_optical',(640,480))
        ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach(product)
        annotators.append(ann)
    for _ in range(20): rep.orchestrator.step(rt_subframes=1)
    results=[]
    for side,ann in zip(['left','right'],annotators):
        pixels=ann.get_data()
        assert pixels.shape[:2]==(480,640),pixels.shape
        assert float(np.std(pixels[:,:,:3]))>1.
        Image.fromarray(pixels).save(ROOT/f'checks/sim61_{side}.png')
        results.append(dict(side=side,shape=list(pixels.shape)))
    (ROOT/'checks/sim61.json').write_text(json.dumps(dict(status='PASS_STEREO_RENDER',version=version('isaacsim'),cameras=results,scope='render only; no dynamics or ROS tracking'),indent=2))
except Exception:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
