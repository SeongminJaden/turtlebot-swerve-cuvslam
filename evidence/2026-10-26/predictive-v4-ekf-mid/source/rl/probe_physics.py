import argparse
from pathlib import Path
import json
from isaaclab.app import add_launcher_args, launch_simulation
parser=argparse.ArgumentParser();add_launcher_args(parser);args=parser.parse_args()
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab_physx.physics import PhysxCfg
ROOT=Path(__file__).resolve().parents[1]
cfg=sim_utils.SimulationCfg(dt=1/120,device='cuda:0',physics=PhysxCfg())
with launch_simulation(cfg,args):
    try:
        sim=sim_utils.SimulationContext(cfg)
        ground=sim_utils.GroundPlaneCfg();ground.func('/World/Ground',ground)
        robot=Articulation(ArticulationCfg(prim_path='/World/Robot',
            spawn=sim_utils.UsdFileCfg(usd_path=str(ROOT/'assets/m38_runtime.usda')),
            init_state=ArticulationCfg.InitialStateCfg(pos=(0,0,.05)),
            actuators={'steer':ImplicitActuatorCfg(joint_names_expr=['.*_steer_joint'],joint_effort_limit=1.,stiffness=30.,damping=3.),
                       'wheel':ImplicitActuatorCfg(joint_names_expr=['.*_wheel_joint'],joint_effort_limit=1.,stiffness=0.,damping=1.),
                       'arm':ImplicitActuatorCfg(joint_names_expr=['arm_joint.*'],joint_effort_limit=2.,stiffness=50.,damping=5.)}))
        sim.reset()
        print('JOINT_NAMES',robot.joint_names,flush=True)
        print('INITIAL_POSE',robot.data.root_pose_w.torch.tolist(),flush=True)
        for i in range(240):
            robot.write_data_to_sim();sim.step();robot.update(1/120)
            if i%60==0:print('STEP',i,'POSE',robot.data.root_pose_w.torch.tolist(),flush=True)
        print('PHYSICS_PROBE_COMPLETE',flush=True)
    except BaseException:
        import traceback
        traceback.print_exc()
        raise
