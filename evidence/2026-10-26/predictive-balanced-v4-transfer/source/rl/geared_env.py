"""Wheel-side actuator limits and swerve allocation with an arm camera."""
import math
import torch
from maze_env import MazeEnv
R=.03068719889482
class GearedMaze(MazeEnv):
 def __init__(self,cfg, wheel_speed_limit=.95/R):
  super().__init__(cfg);self.wheel_speed_limit=wheel_speed_limit;self.arm_ids=torch.tensor([self.robot.joint_names.index('arm_joint'+str(i)) for i in range(1,6)],device=self.device);self.arm_command=torch.zeros((self.num_envs,5),device=self.device);self.drive=False;self.velocity=torch.zeros((self.num_envs,2),device=self.device)
 def _pre_physics_step(self,actions):
  # Base allocator expects .18 m/s scale; retain geometry, replace scale via
  # unclamped equivalent computation below (wheel-side quantities throughout).
  y=self.yaw() if not hasattr(self,'measured_yaw') else self.measured_yaw;c,s=y.cos(),y.sin();wx,wy=self.velocity[:,0],self.velocity[:,1]
  vx,vy=c*wx+s*wy,-s*wx+c*wy;wz=(-2*y).clamp(-.5,.5) if self.drive else torch.zeros_like(y)
  mx=torch.tensor([.1,.1,-.1,-.1],device=self.device);my=torch.tensor([.1,-.1,.1,-.1],device=self.device)
  u=vx[:,None]-wz[:,None]*my;v=vy[:,None]+wz[:,None]*mx;beta=torch.tensor([-math.pi/2,-math.pi/2,0.,math.pi],device=self.device)
  raw=(torch.atan2(v,u)-beta+math.pi)%(2*math.pi)-math.pi;sign=torch.where(raw.abs()>math.pi/2,-1.,1.);angle=torch.where(raw>math.pi/2,raw-math.pi,raw);angle=torch.where(angle< -math.pi/2,angle+math.pi,angle)
  hold=((raw.abs()-math.pi/2).abs()<.06)&(self.steering_target.abs()>.7);held=self.steering_target.sign()*(math.pi/2);angle=torch.where(hold,held,angle);sign=torch.where(hold,torch.cos(raw-held).sign(),sign)
  self.steering_target+=(angle-self.steering_target).clamp(-self.step_dt,self.step_dt)
  steer_position=self.robot.data.joint_pos.torch[:,self.steer] if not hasattr(self,'measured_steer') else self.measured_steer
  error=angle-steer_position;speed=torch.sqrt(u*u+v*v)/R*sign
  self.wheel_target=(speed*torch.cos(error).clamp(min=0)).clamp(-self.wheel_speed_limit,self.wheel_speed_limit) if self.drive else torch.zeros_like(speed)
 def _apply_action(self):
  super()._apply_action();self.robot.actuators.target_command.set_position_index(value=self.arm_command,joint_ids=self.arm_ids)

