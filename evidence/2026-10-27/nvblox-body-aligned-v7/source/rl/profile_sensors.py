import argparse,cProfile,pstats
from isaaclab.app import add_launcher_args,launch_simulation
p=argparse.ArgumentParser();add_launcher_args(p);a=p.parse_args()
import torch
from maze_env import MazeEnvCfg
from predictive_env import configure
from geared_env import GearedMaze
from robot_sensors import configure_sensors,PhysicsSensorStream
cfg=configure_sensors(configure(MazeEnvCfg()));cfg.sim.device=a.device or 'cuda:0';cfg.scene.num_envs=1;cfg.sim.dt=.005;cfg.decimation=13;cfg.sim.render_interval=13;cfg.episode_length_s=100
with launch_simulation(cfg.sim,a):
 e=GearedMaze(cfg);e.reset();e.evaluation=True;s=PhysicsSensorStream(e,udp_port=18769)
 prof=cProfile.Profile();prof.enable()
 for i in range(20):e.step(torch.zeros((1,2),device=e.device))
 prof.disable();prof.dump_stats('runs/sensor-profile.prof')
 with open('runs/sensor-profile.txt','w') as out:pstats.Stats(prof,stream=out).sort_stats('cumtime').print_stats(45)
 e.close()
