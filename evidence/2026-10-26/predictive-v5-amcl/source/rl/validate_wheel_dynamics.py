"""A/B actual PhysX straight-motion audit of the wheel inertia proxy."""
import argparse,json
from pathlib import Path
from isaaclab.app import add_launcher_args,launch_simulation
p=argparse.ArgumentParser();p.add_argument('--balanced',action='store_true');p.add_argument('--output',type=Path,required=True);add_launcher_args(p);a=p.parse_args()
import torch,numpy as np
from maze_env import MazeEnvCfg,ROOT
from predictive_env import configure
from geared_env import GearedMaze
from robot_sensors import configure_sensors,RobotSensors
cfg=configure_sensors(configure(MazeEnvCfg()),contacts=True);cfg.sim.device='cpu';cfg.sim.dt=.005;cfg.decimation=10;cfg.sim.render_interval=10;cfg.scene.num_envs=1;cfg.episode_length_s=30
if a.balanced:cfg.scene.robot.spawn.usd_path=str(ROOT/'assets/m38_balanced_wheels.usda')
rows=[]
with launch_simulation(cfg.sim,a):
 e=GearedMaze(cfg);e.reset();e.evaluation=True;ids=e.robot._ALL_INDICES;q=e.robot.data.default_joint_pos.torch.clone();q[:,e.steer]=torch.tensor([np.pi/2,np.pi/2,0.,0.]);q[:,e.arm_ids[3]]=-.3;e.robot.write_joint_position_to_sim_index(position=q,env_ids=ids);e.arm_command[:]=q[:,e.arm_ids];e.steering_target[:]=q[:,e.steer]
 for _ in range(30):e.step(torch.zeros((1,2),device=e.device))
 sensors=RobotSensors(e)
 for i in range(80):
  t=i*e.step_dt;command=min(.95,.6*t) if t<2. else max(0.,.95-.6*(t-2.))
  e.velocity[:]=torch.tensor([command,0.]);e.drive=True;e.step(torch.zeros((1,2),device=e.device))
  sensor=sensors.sample(1_000_000_000+round((i+1)*e.step_dt*1e9));v=e.robot.data.root_vel_w.torch[0].numpy();wheel=e.robot.data.joint_vel.torch[0,e.wheel].numpy()
  contact=e.scene['contacts'];forces=contact.data.net_normal_forces_w.torch[0].norm(dim=-1).numpy();nonwheel=[(n,float(forces[k])) for k,n in enumerate(contact.body_names) if not n.endswith('_wheel')]
  rows.append({'contact_body':max(nonwheel,key=lambda x:x[1])[0],'contact_force':max(x[1] for x in nonwheel),'t':t,'command':command,'speed':float(np.linalg.norm(v[:2])),'yaw_rate':float(v[5]),'max_wheel_rad_s':float(np.abs(wheel).max()),'fusion_error':sensor['fusion_error']})
 a.output.mkdir(parents=True,exist_ok=True);report={'balanced':a.balanced,'max_speed':max(r['speed'] for r in rows),'max_wheel_rad_s':max(r['max_wheel_rad_s'] for r in rows),'max_yaw_rate':max(abs(r['yaw_rate']) for r in rows),'fusion_rejections':sum(r['fusion_error'] is not None for r in rows),'rows':rows};(a.output/'report.json').write_text(json.dumps(report,indent=2));print('DYNAMICS',json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True);e.close()
