import argparse,traceback
from isaaclab.app import add_launcher_args,launch_simulation
p=argparse.ArgumentParser();add_launcher_args(p);args=p.parse_args()
from maze_env import MazeEnvCfg,MazeEnv
import torch
cfg=MazeEnvCfg();cfg.scene.num_envs=1
with launch_simulation(cfg.sim,args):
    try:
        env=MazeEnv(cfg);obs,_=env.reset()
        print('OBS',obs['policy'].shape,flush=True)
        for i in range(150):
            action=(env.goal-env.xy())*3
            obs,reward,term,trunc,_=env.step(action)
            if i%25==0:print('MOTION',i,env.xy().tolist(),reward.tolist(),flush=True)
        print('ENV_SMOKE_COMPLETE',flush=True)
        env.close()
    except BaseException:traceback.print_exc();raise
