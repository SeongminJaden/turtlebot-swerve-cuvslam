"""Render the actual USD model from an orbiting camera. No physics claim."""
from pathlib import Path
from math import sin, cos, pi
import argparse
import datetime
import json
from PIL import Image
Image.init()
parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, default=Path("evidence")/datetime.date.today().isoformat())
args = parser.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'width': 800, 'height': 600})
import omni.usd
import omni.replicator.core as rep
from pxr import UsdGeom, UsdLux, UsdPhysics, Gf
from PIL import Image
ROOT = Path(__file__).resolve().parents[1]
OUT = args.output.resolve()
OUT.mkdir(parents=True, exist_ok=True)
(OUT/'orbit-frames').mkdir(exist_ok=True)
try:
    omni.usd.get_context().open_stage(str(ROOT/'assets/m38_stereo.usda'))
    for _ in range(10): app.update()
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(120)
    UsdLux.DomeLight.Define(stage, '/World/CaptureLight').CreateIntensityAttr(650.)
    bbox = UsdGeom.BBoxCache(0, ['default', 'render']).ComputeWorldBound(stage.GetPrimAtPath('/World/Robot')).ComputeAlignedRange()
    center = bbox.GetMidpoint()
    radius = max(bbox.GetSize()) * 1.8
    print('MODEL_BOUNDS', bbox, 'CENTER', center, flush=True)
    camera = UsdGeom.Camera.Define(stage, '/World/CaptureCamera')
    camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 100))
    camera.CreateFocalLengthAttr(28)
    transform = camera.AddTransformOp()
    product = rep.create.render_product(str(camera.GetPath()), (800, 600))
    ann = rep.AnnotatorRegistry.get_annotator('rgb')
    ann.attach(product)
    frames = []
    for i in range(36):
        angle = 2*pi*i/36 + .7
        eye = center + Gf.Vec3d(radius*cos(angle), radius*sin(angle), radius*.45)
        transform.Set(Gf.Matrix4d().SetLookAt(eye, center, Gf.Vec3d(0, 0, 1)).GetInverse())
        for _ in range(3): rep.orchestrator.step(rt_subframes=1)
        pixels = ann.get_data()
        for retry in range(6):
            if hasattr(pixels, 'shape') and pixels.shape[:2] == (600, 800):
                break
            rep.orchestrator.step(rt_subframes=1)
            pixels = ann.get_data()
        if not hasattr(pixels, 'shape') or pixels.shape[:2] != (600, 800):
            raise RuntimeError(f'Missing render data for frame {i}')
        frame = Image.fromarray(pixels.copy()).convert('RGB')
        frame.save(OUT/'orbit-frames'/f'{i:03d}.png')
        if i == 0: frame.save(OUT/'model-overview.png')
        frames.append(frame)
    frames[0].save(OUT/'model-orbit.gif', save_all=True, append_images=frames[1:], duration=120, loop=0)
    (OUT/'model-capture.json').write_text(json.dumps({'scope':'Actual USD render, orbiting camera only; not driving or physics validation', 'frames':len(frames), 'resolution':[800,600]},indent=2))
except Exception:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
