"""Joint acceleration + anticipatory arm PPO on actual Isaac Lab PhysX.
Training perception is a disclosed analytic wall-ray/FOV proxy. Deployment must
replace it with timestamped stereo/depth data; cuVSLAM is separately validated.
"""
import math
import torch
from isaaclab.utils import configclass
from maze_env import MazeEnvCfg,ROOT,MAP
from geared_env import GearedMaze,R
@configclass
class PredictiveEnvCfg(MazeEnvCfg):
 action_space=2
 observation_space=24
 episode_length_s=22.

def configure(cfg):
 cfg.scene.robot.spawn.usd_path=str(ROOT/'assets/m38_active_camera_demo.usda');cfg.scene.robot.actuators['arm'].joint_velocity_limit=.5;cfg.scene.robot.actuators['wheel'].joint_velocity_limit=.95/R;cfg.scene.robot.actuators['wheel'].joint_effort_limit=.25;return cfg

class PredictiveEnv(GearedMaze):
 def __init__(self,cfg):
  super().__init__(cfg);n=self.num_envs;d=self.device
  self.j=int(self.arm_ids[0]);self.index=torch.zeros(n,dtype=torch.long,device=d);self.end=torch.zeros_like(self.index);self.start=torch.zeros_like(self.index);self.speed_command=torch.zeros(n,device=d);self.known_clearance=torch.zeros(n,device=d);self.view_age=torch.zeros(n,device=d);self.cam_target=torch.zeros(n,device=d);self.previous_xy=torch.zeros((n,2),device=d);self.previous_error=torch.zeros(n,device=d);self.previous_actions=torch.zeros((n,2),device=d);self.actions=torch.zeros_like(self.previous_actions);self.success=torch.zeros(n,dtype=torch.bool,device=d);self.success_count=0;self.episode_count=0;self.collision_count=0;self.dwell=torch.zeros(n,device=d);self.evaluation=False;self.clearance_heading=torch.zeros(n,device=d)
  centers=torch.tensor([w['center'] for w in MAP['panels']],device=d);axis=torch.tensor([[math.cos(w['yaw']),math.sin(w['yaw'])] for w in MAP['panels']],device=d);length=torch.tensor([w['length'] for w in MAP['panels']],device=d);self.wall_a=centers-axis*length[:,None]/2;self.wall_b=centers+axis*length[:,None]/2
 def geometry(self,update_sensor=True):
  xy=self.xy();offset=torch.arange(-2,16,device=self.device);cand=(self.index[:,None]+offset).clamp(0,len(self.route)-1);cand=torch.minimum(cand,self.end[:,None]);points=self.route[cand];dist=(points-xy[:,None,:]).norm(dim=-1);self.index=cand.gather(1,dist.argmin(1)[:,None])[:,0]
  nav_idx=torch.minimum(self.index+4,self.end);delta=self.route[nav_idx]-xy;unit=delta/(delta.norm(dim=-1,keepdim=True)+1e-6);heading=torch.atan2(unit[:,1],unit[:,0]);speed=self.robot.data.root_vel_w.torch[:,:2].norm(dim=-1)
  # Anticipation covers camera slew time before the upcoming route bend.
  look_steps=(6+speed*24).long().clamp(6,24);future_idx=torch.minimum(self.index+look_steps,self.end);future=self.route[future_idx]-xy;future_heading=torch.atan2(future[:,1],future[:,0]);raw=future_heading-self.yaw()
  candidates=(raw[:,None]+torch.tensor([-2*math.pi,0.,2*math.pi],device=self.device)).clamp(-3.05,3.05);angular=torch.atan2(torch.sin(candidates-raw[:,None]),torch.cos(candidates-raw[:,None])).abs();best=angular.min(1).values;cost=(candidates-self.cam_target[:,None]).abs()+100*(angular>best[:,None]+math.radians(10));self.cam_target=candidates.gather(1,cost.argmin(1)[:,None])[:,0]
  q=self.robot.data.joint_pos.torch[:,self.j];view_error=torch.atan2(torch.sin(heading-self.yaw()-q),torch.cos(heading-self.yaw()-q));visible=view_error.abs()<math.radians(35)
  # Ray / finite-wall intersections in planar CAD coordinates.
  ray=unit[:,None,:];edge=(self.wall_b-self.wall_a)[None,:,:];w=self.wall_a[None,:,:]-xy[:,None,:];cross=lambda a,b:a[...,0]*b[...,1]-a[...,1]*b[...,0]
  den=cross(ray,edge);den=torch.where(den.abs()<1e-8,torch.full_like(den,1e-8),den);t=cross(w,edge)/den;u=cross(w,ray)/den;hit=(t>0)&(u>=0)&(u<=1);clearance=torch.where(hit,t,torch.full_like(t,3.)).min(1).values.clamp(0,3.)
  traveled=(xy-self.previous_xy).norm(dim=-1)
  if update_sensor:
   # A range measured in one direction is not free space in another.
   same_direction=torch.cos(heading-self.clearance_heading)>math.cos(math.radians(5))
   retained=torch.where(same_direction,(self.known_clearance-traveled).clamp(min=0),0.)
   self.known_clearance=torch.where(visible,clearance,retained);self.view_age=torch.where(visible,0.,self.view_age+self.step_dt)
   self.clearance_heading=torch.where(visible,heading,self.clearance_heading)
  proj=((xy[:,None,:]-self.wall_a[None,:,:])*edge).sum(-1)/(edge.square().sum(-1)+1e-8);nearest=self.wall_a[None,:,:]+proj.clamp(0,1)[:,:,None]*edge;wall_distance=(xy[:,None,:]-nearest).norm(dim=-1).min(1).values
  self.g={'xy':xy,'unit':unit,'heading':heading,'future_heading':future_heading,'speed':speed,'visible':visible,'clearance':clearance,'wall_distance':wall_distance,'goal_distance':(self.route[self.end]-xy).norm(dim=-1),'cross_track':dist.min(1).values}
  return self.g
 def _pre_physics_step(self,actions):
  self.previous_actions=self.actions.clone();self.actions=actions.clamp(-1,1);g=self.g;self.speed_command=(self.speed_command+.6*self.actions[:,0]*self.step_dt).clamp(0,.95)
  # Stop only for an unobserved braking envelope, not merely arm movement.
  safe=torch.sqrt((2*.6*(self.known_clearance-.18).clamp(min=0)));safe=torch.where(self.view_age>1.5,torch.zeros_like(safe),safe);speed=torch.minimum(self.speed_command,safe);self.speed_command=speed.clone()
  self.velocity=g['unit']*speed[:,None];self.drive=True;self.arm_command[:,0]=(self.robot.data.joint_pos.torch[:,self.j]+.5*self.actions[:,1]*self.step_dt).clamp(-math.pi,math.pi);super()._pre_physics_step(actions)
 def _get_observations(self):
  # geometry() is called once before each action. No hidden repeated age updates.
  if not hasattr(self,'g'):self.geometry()
  g=self.g;q=self.robot.data.joint_pos.torch[:,self.j];qd=self.robot.data.joint_vel.torch[:,self.j];yaw=self.yaw();remaining=1-self.episode_length_buf/self.max_episode_length;curve=torch.atan2(torch.sin(g['future_heading']-g['heading']),torch.cos(g['future_heading']-g['heading']))
  obs=torch.stack([self.cam_target/math.pi,q/math.pi,qd/.5,(self.cam_target-q)/math.pi,self.actions[:,1],remaining,g['speed']/.95,self.speed_command/.95,g['goal_distance']/2,torch.sin(g['future_heading']-yaw),torch.cos(g['future_heading']-yaw),torch.sin(g['heading']-yaw),torch.cos(g['heading']-yaw),self.known_clearance/3,self.view_age/2,g['visible'].float(),curve/math.pi,(self.index-self.start).float()/(self.end-self.start).clamp(min=1),self.actions[:,0],g['cross_track']],-1)
  # Body attitude and angular velocity allow anticipatory braking for stability.
  attitude=self.robot.data.root_pose_w.torch[:,3:5]/.3;rates=self.robot.data.root_vel_w.torch[:,3:5]/2
  return {'policy':torch.cat([obs,attitude,rates],-1).clamp(-5,5)}
 def _get_dones(self):
  self.geometry();xy=self.xy();p=self.robot.data.root_pose_w.torch;dist=(self.route[self.end]-xy).norm(dim=-1);v=self.robot.data.root_vel_w.torch[:,:2].norm(dim=-1);self.dwell=torch.where((dist<.10)&(v<.10),self.dwell+self.step_dt,0.);self.success=self.dwell>=.3
  self.bad=(p[:,2]<-.08)|(p[:,2]>.25)|(p[:,3:5].norm(dim=1)>.3)|(~torch.isfinite(p).all(dim=1))|(self.g['wall_distance']<.14)|(self.g['cross_track']>.4)
  timeout=self.episode_length_buf>=self.max_episode_length-1
  return (torch.zeros_like(self.bad) if self.evaluation else self.bad|self.success),timeout
 def _get_rewards(self):
  xy=self.xy();progress=((xy-self.previous_xy)*self.g['unit']).sum(dim=-1).clamp(-.1,.1);e=(self.cam_target-self.robot.data.joint_pos.torch[:,self.j]).abs();reward=8*progress+.25*(self.previous_error-e)-.08*self.step_dt-.005*self.actions.square().sum(-1)*self.step_dt-.01*(self.actions[:,0]-self.previous_actions[:,0]).square()+5*self.success.float()-5*self.bad.float();reward-=2*(self.g['speed'].square()/1.2+.18-self.known_clearance).clamp(min=0)*self.step_dt
  reward-=2*(self.robot.data.root_pose_w.torch[:,3:5].square().sum(-1)/.09)*self.step_dt
  self.previous_error=e.detach().clone();self.previous_xy=xy.detach().clone();return reward
 def _reset_idx(self,ids):
  if ids is None:ids=self.robot._ALL_INDICES
  # Preserve the generic PhysX reset and replace its episode goal/distribution.
  super()._reset_idx(ids)
  if not hasattr(self,'index'):return
  self.success_count+=int(self.success[ids].sum());self.episode_count+=len(ids);self.collision_count+=int(self.bad[ids].sum()) if hasattr(self,'bad') else 0
  n=len(ids);idx=torch.randint(0,len(self.route)-35,(n,),device=self.device);self.index[ids]=idx;self.start[ids]=idx;self.end[ids]=(idx+torch.randint(18,35,(n,),device=self.device)).clamp(max=len(self.route)-1)
  pose=self.robot.data.default_root_pose.torch[ids].clone();pose[:,:2]=self.route[idx]+self.scene.env_origins[ids,:2];pose[:,2]=.015;pose[:,3:]=torch.tensor([0.,0.,0.,1.],device=self.device);self.robot.write_root_pose_to_sim_index(root_pose=pose,env_ids=ids);self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros((n,6),device=self.device),env_ids=ids)
  heading=torch.atan2(self.route[idx+4,1]-self.route[idx,1],self.route[idx+4,0]-self.route[idx,0]);q=self.robot.data.default_joint_pos.torch[ids].clone();q[:,self.j]=(heading+(torch.rand(n,device=self.device)-.5)*1.4).clamp(-2.8,2.8);self.robot.write_joint_position_to_sim_index(position=q,env_ids=ids);self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q),env_ids=ids)
  self.arm_command[ids]=q[:,self.arm_ids];self.steering_target[ids]=q[:,self.steer];self.cam_target[ids]=heading.clamp(-3.05,3.05);self.clearance_heading[ids]=heading;self.speed_command[ids]=0;self.known_clearance[ids]=0;self.view_age[ids]=0;self.previous_xy[ids]=self.route[idx];self.previous_error[ids]=(self.cam_target[ids]-q[:,self.j]).abs();self.actions[ids]=0;self.previous_actions[ids]=0;self.dwell[ids]=0;self.success[ids]=False;self.robot.reset(ids)
  self.geometry(update_sensor=False)
