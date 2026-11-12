"""v4 curriculum: bounded preview, no speed windup, measured IMU observations.
Navigation/clearance training remains an analytic proxy with sensor noise;
actual stereo + cuVSLAM are evaluated separately in the asynchronous runtime.
"""
import math,torch
from predictive_env import PredictiveEnv as PreviousEnv,PredictiveEnvCfg,configure as previous_configure
from robot_sensors import configure_sensors

def configure(cfg):
 cfg=configure_sensors(previous_configure(cfg));cfg.episode_length_s=55.
 return cfg

class PredictiveEnv(PreviousEnv):
 def __init__(self,cfg):
  super().__init__(cfg)
  self.filtered_up=torch.zeros((self.num_envs,3),device=self.device);self.filtered_up[:,2]=1.
  self.last_omega=torch.zeros_like(self.filtered_up);self.filtered_alpha=torch.zeros_like(self.filtered_up)
 def geometry(self,update_sensor=True):
  g=super().geometry(update_sensor)
  turn=torch.atan2(torch.sin(g['future_heading']-g['heading']),torch.cos(g['future_heading']-g['heading'])).clamp(-math.radians(20),math.radians(20))
  g['future_heading']=g['heading']+turn
  raw=g['future_heading']-self.yaw()
  cand=(raw[:,None]+torch.tensor([-2*math.pi,0.,2*math.pi],device=self.device)).clamp(-3.05,3.05)
  err=torch.atan2(torch.sin(cand-raw[:,None]),torch.cos(cand-raw[:,None])).abs();cost=(cand-self.cam_target[:,None]).abs()+100*(err>err.min(1).values[:,None]+.02)
  self.cam_target=cand.gather(1,cost.argmin(1)[:,None])[:,0]
  return g
 def _get_observations(self):
  obs=super()._get_observations()['policy']
  if not hasattr(self,'filtered_up'):return {'policy':obs}
  front=self.scene['opencr_1'].data;rear=self.scene['opencr_2'].data
  flip=torch.tensor([-1.,-1.,1.],device=self.device)
  omega=(front.ang_vel_b.torch+rear.ang_vel_b.torch*flip)/2
  force=(front.lin_acc_b.torch+rear.lin_acc_b.torch*flip)/2
  alpha=(omega-self.last_omega)/self.step_dt;mix=self.step_dt/(.03+self.step_dt)
  self.filtered_alpha=(1-mix)*self.filtered_alpha+mix*alpha;self.last_omega=omega.clone()
  r=torch.tensor([0.,0.,.16597],device=self.device).expand_as(omega)
  force=force-torch.cross(self.filtered_alpha,r,dim=-1)-torch.cross(omega,torch.cross(omega,r,dim=-1),dim=-1)
  up=self.filtered_up-torch.cross(omega,self.filtered_up,dim=-1)*self.step_dt
  magnitude=force.norm(dim=-1,keepdim=True);blend=self.step_dt/(2.+self.step_dt)
  up=torch.where((magnitude-9.80665).abs()<.5,(1-blend)*up+blend*force/magnitude.clamp(min=.1),up)
  self.filtered_up=up/up.norm(dim=-1,keepdim=True).clamp(min=.1)
  roll=torch.atan2(up[:,1],up[:,2]);pitch=torch.atan2(-up[:,0],up[:,1:3].norm(dim=-1))
  obs[:,20]=torch.sin(roll/2)*torch.cos(pitch/2)/.3;obs[:,21]=torch.cos(roll/2)*torch.sin(pitch/2)/.3;obs[:,22:24]=omega[:,:2]/2
  # Include resolution/noise, not simulator-exact arm angle/rate observations.
  q=self.robot.data.joint_pos.torch[:,self.j];quant=torch.round(q/(2*math.pi)*4096)/4096*(2*math.pi)
  obs[:,1]=quant/math.pi;obs[:,3]=(self.cam_target-quant)/math.pi
  obs[:,2]+=torch.randn_like(q)*.025;obs[:,20:24]+=torch.randn_like(obs[:,20:24])*.005
  obs[:,5]=1.-obs[:,17] # route remaining, independent of episode timeout
  return {'policy':obs.clamp(-5,5)}
 def _get_rewards(self):
  reward=super()._get_rewards()
  error=(self.cam_target-self.robot.data.joint_pos.torch[:,self.j]).abs()
  reward-=.3*error*self.step_dt
  reward-=.15*self.g['cross_track']*self.step_dt
  return reward
 def _reset_idx(self,ids):
  super()._reset_idx(ids)
  if not hasattr(self,'index'):return
  if ids is None:ids=self.robot._ALL_INDICES
  # Half of episodes cover long segments, including the full route start.
  n=len(ids);long=torch.rand(n,device=self.device)<.5
  self.end[ids]=torch.where(long,torch.full_like(self.end[ids],len(self.route)-1),self.end[ids])
  q=self.robot.data.joint_pos.torch[ids].clone();q[:,int(self.arm_ids[3])]=-.30
  self.robot.write_joint_position_to_sim_index(position=q,env_ids=ids);self.arm_command[ids]=q[:,self.arm_ids]
  if hasattr(self,'filtered_up'):
   self.filtered_up[ids]=torch.tensor([0.,0.,1.],device=self.device);self.last_omega[ids]=0.;self.filtered_alpha[ids]=0.
