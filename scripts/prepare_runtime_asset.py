"""Explicit floating base for Isaac Lab; original CAD USD is preserved."""
from pathlib import Path
from pxr import Usd, UsdPhysics
ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'assets/m38_runtime.usda'
stage=Usd.Stage.CreateNew(str(out))
robot=stage.DefinePrim('/Robot','Xform')
robot.GetReferences().AddReference('./m38_arm_description/usd/m38_arm.usdc')
stage.SetDefaultPrim(robot)
stage.SetMetadata('metersPerUnit',1.0);stage.SetMetadata('upAxis','Z')
robot.RemoveAPI(UsdPhysics.ArticulationRootAPI)
base=stage.GetPrimAtPath('/Robot/base_link')
UsdPhysics.ArticulationRootAPI.Apply(base)
stage.GetRootLayer().Save()
print(out)
