"""Wheel-side actuator limits and swerve allocation with an arm camera."""
import math
import torch
from maze_env import MazeEnv
R=.03068719889482
class GearedMaze(MazeEnv):
 def __init__(self,cfg, wheel_speed_limit=.95/R):
  super().__init__(cfg);self.wheel_speed_limit=wheel_speed_limit;self.arm_ids=torch.tensor([self.robot.joint_names.index('arm_joint'+str(i)) for i in range(1,6)],device=self.device);self.arm_command=torch.zeros((self.num_envs,5),device=self.device);self.drive=False;self.velocity=torch.zeros((self.num_envs,2),device=self.device)
 def _pre_physics_step(self,actions):
  guarded=getattr(self,'synchronize_steering',False)
  if guarded and not self.drive:
   # A camera/IMU stop must not simultaneously steer every module back to zero.
   delta=self.wheel_acceleration_limit*self.step_dt/R
   self.wheel_target+=(-self.wheel_target).clamp(-delta,delta)
   return
  # Base allocator expects .18 m/s scale; retain geometry, replace scale via
  # unclamped equivalent computation below (wheel-side quantities throughout).
  y=self.yaw() if not hasattr(self,'measured_yaw') else self.measured_yaw;c,s=y.cos(),y.sin();wx,wy=self.velocity[:,0],self.velocity[:,1]
  vx,vy=c*wx+s*wy,-s*wx+c*wy;yaw_error=torch.atan2(torch.sin(y-getattr(self,'hold_yaw',0.)),torch.cos(y-getattr(self,'hold_yaw',0.)));wz=(-2*yaw_error).clamp(-.5,.5) if self.drive else torch.zeros_like(y)
  mx=torch.tensor([.1,.1,-.1,-.1],device=self.device);my=torch.tensor([.1,-.1,.1,-.1],device=self.device)
  u=vx[:,None]-wz[:,None]*my;v=vy[:,None]+wz[:,None]*mx;beta=torch.tensor([-math.pi/2,-math.pi/2,0.,math.pi],device=self.device)
  raw=(torch.atan2(v,u)-beta+math.pi)%(2*math.pi)-math.pi;sign=torch.where(raw.abs()>math.pi/2,-1.,1.);angle=torch.where(raw>math.pi/2,raw-math.pi,raw);angle=torch.where(angle< -math.pi/2,angle+math.pi,angle)
  hold=((raw.abs()-math.pi/2).abs()<.06)&(self.steering_target.abs()>.7);held=self.steering_target.sign()*(math.pi/2);angle=torch.where(hold,held,angle);sign=torch.where(hold,torch.cos(raw-held).sign(),sign)
  steer_position=self.robot.data.joint_pos.torch[:,self.steer] if not hasattr(self,'measured_steer') else self.measured_steer
  if guarded:
   if not hasattr(self,'steering_phase'):
    self.steering_phase=torch.zeros(self.num_envs,dtype=torch.long,device=self.device);self.reorient_angle=angle.clone()
   wheel_velocity=self.robot.data.joint_vel.torch[:,self.wheel] if not hasattr(self,'measured_wheel') else self.measured_wheel
   trigger=(self.steering_phase==0)&((angle-steer_position).abs().amax(1)>.35)
   self.reorient_angle[trigger]=angle[trigger];self.steering_phase[trigger]=1
   stopped=(self.steering_phase==1)&(wheel_velocity.abs().amax(1)<.5);self.steering_phase[stopped]=2
   angle=torch.where((self.steering_phase>0)[:,None],self.reorient_angle,angle)
   advance=(angle-self.steering_target).clamp(-self.step_dt,self.step_dt)
   self.steering_target+=torch.where((self.steering_phase==1)[:,None],torch.zeros_like(advance),advance)
   aligned=(self.steering_phase==2)&((angle-steer_position).abs().amax(1)<.08);self.steering_phase[aligned]=0
  else:self.steering_target+=(angle-self.steering_target).clamp(-self.step_dt,self.step_dt)
  error=angle-steer_position;speed=torch.sqrt(u*u+v*v)/R*sign
  desired=(speed*torch.cos(error).clamp(min=0)).clamp(-self.wheel_speed_limit,self.wheel_speed_limit) if self.drive else torch.zeros_like(speed)
  if guarded:
   desired=speed*torch.cos(error).clamp(min=0).amin(1,keepdim=True)
   desired=desired.clamp(-self.wheel_speed_limit,self.wheel_speed_limit)
   desired=torch.where(((self.steering_phase==0)&~aligned)[:,None],desired,torch.zeros_like(desired))
  # Wheel-side slew guard also applies to perception-loss braking. Instant
  # velocity-servo stops pitch the chassis; this is an actuator constraint,
  # not replacement of the policy's acceleration action.
  if hasattr(self,'wheel_acceleration_limit'):
   delta=self.wheel_acceleration_limit*self.step_dt/R
   self.wheel_target+=(desired-self.wheel_target).clamp(-delta,delta)
  else:self.wheel_target=desired
 def _apply_action(self):
  super()._apply_action();self.robot.actuators.target_command.set_position_index(value=self.arm_command,joint_ids=self.arm_ids)

