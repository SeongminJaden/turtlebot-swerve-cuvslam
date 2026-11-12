"""Real-time paced, asynchronous full-maze camera-policy integration.
Camera/VO never block physics. Pose age gates motion. Navigation uses VO pose
registered to the known initial spawn, with GT retained only for evaluation.
"""
import argparse,json,math,time,signal,sys
from pathlib import Path
import numpy as np
from isaaclab.app import add_launcher_args,launch_simulation
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=240);p.add_argument('--max-speed',type=float,default=.95);p.add_argument('--cruise',type=float,default=.45);p.add_argument('--motor-torque',type=float,default=1.);p.add_argument('--efficiency',type=float,default=1.);p.add_argument('--gt-navigation',action='store_true');add_launcher_args(p);a=p.parse_args()
import torch
from maze_env import MazeEnv,MazeEnvCfg,ROOT
from camera_policy import CameraPolicy
from active_view import ViewGate,ViewGateConfig
from camera_heading import target_for_heading
sys.path.insert(0,str(ROOT/'scripts'));from pose_math import matrix,quaternion
R=.03068719889482;GEAR=4.;wheel_omega=a.max_speed/R;motor_omega=wheel_omega/GEAR;wheel_torque=a.motor_torque*a.efficiency/GEAR
class GearedMaze(MazeEnv):
 def __init__(self,cfg):
  super().__init__(cfg);self.arm_ids=torch.tensor([self.robot.joint_names.index('arm_joint'+str(i)) for i in range(1,6)],device=self.device);self.arm_command=torch.zeros((1,5),device=self.device);self.drive=False;self.velocity=torch.zeros(2,device=self.device)
 def _pre_physics_step(self,actions):
  # Base allocator expects .18 m/s scale; retain geometry, replace scale via
  # unclamped equivalent computation below (wheel-side quantities throughout).
  y=self.yaw();c,s=y.cos(),y.sin();wx,wy=self.velocity
  vx,vy=c*wx+s*wy,-s*wx+c*wy;wz=(-2*y).clamp(-.5,.5) if self.drive else torch.zeros_like(y)
  mx=torch.tensor([.1,.1,-.1,-.1],device=self.device);my=torch.tensor([.1,-.1,.1,-.1],device=self.device)
  u=vx[:,None]-wz[:,None]*my;v=vy[:,None]+wz[:,None]*mx;beta=torch.tensor([-math.pi/2,-math.pi/2,0.,math.pi],device=self.device)
  raw=(torch.atan2(v,u)-beta+math.pi)%(2*math.pi)-math.pi;sign=torch.where(raw.abs()>math.pi/2,-1.,1.);angle=torch.where(raw>math.pi/2,raw-math.pi,raw);angle=torch.where(angle< -math.pi/2,angle+math.pi,angle)
  hold=((raw.abs()-math.pi/2).abs()<.06)&(self.steering_target.abs()>.7);held=self.steering_target.sign()*(math.pi/2);angle=torch.where(hold,held,angle);sign=torch.where(hold,torch.cos(raw-held).sign(),sign)
  self.steering_target+=(angle-self.steering_target).clamp(-self.step_dt,self.step_dt)
  error=angle-self.robot.data.joint_pos.torch[:,self.steer];speed=torch.sqrt(u*u+v*v)/R*sign
  self.wheel_target=(speed*torch.cos(error).clamp(min=0)).clamp(-wheel_omega,wheel_omega) if self.drive else torch.zeros_like(speed)
 def _apply_action(self):
  super()._apply_action();self.robot.actuators.target_command.set_position_index(value=self.arm_command,joint_ids=self.arm_ids)

cfg=MazeEnvCfg();cfg.scene.num_envs=1;cfg.episode_length_s=a.seconds+10;cfg.scene.robot.spawn.usd_path=str(ROOT/'assets/m38_active_camera_demo.usda');cfg.scene.robot.actuators['arm'].joint_velocity_limit=.5;cfg.scene.robot.actuators['wheel'].joint_velocity_limit=wheel_omega;cfg.scene.robot.actuators['wheel'].joint_effort_limit=wheel_torque
# Main corridor vertices simplified from the supplied grid route; each is
# reached at rest before a different camera view is requested.
waypoints=np.array([[1.85,2.80],[1.85,2.20],[2.15,2.00],[3.30,2.00],[3.30,.65],[3.10,.45],[.35,.45],[.35,.35]])
a.output.mkdir(parents=True,exist_ok=True);stop_requested=False
signal.signal(signal.SIGINT,lambda *_:globals().__setitem__('stop_requested',True));signal.signal(signal.SIGTERM,lambda *_:globals().__setitem__('stop_requested',True))
rows=[];bodies=[];joints=[];trajectory=[];segment=0;hold=None;prev_action=0.;previous_target=0.;align=None;last_source=-1;frames=0;last_tracking={};nav_xy=None;nav_yaw=0.;reason='timeout';segment_start=0.
with launch_simulation(cfg.sim,a):
 env=GearedMaze(cfg);env.reset();env.evaluation=True;ids=env.robot._ALL_INDICES
 pose=env.robot.data.default_root_pose.torch.clone();pose[0,:3]=torch.tensor([.35,2.85,.015],device=env.device);pose[0,3:]=torch.tensor([0.,0.,0.,1.],device=env.device);env.robot.write_root_pose_to_sim_index(root_pose=pose,env_ids=ids);env.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1,6),device=env.device),env_ids=ids)
 model=CameraPolicy().to(env.device);ck=torch.load(ROOT/'runs/camera_ppo_v1/policy.pt',map_location=env.device,weights_only=True);model.load_state_dict(ck['model']);model.eval();j=int(env.arm_ids[0]);gate=ViewGate(ViewGateConfig(max_sample_age_s=.25));begin=time.monotonic();log=open(a.output/'steps.jsonl','w');heading=math.atan2(waypoints[0,1]-2.85,waypoints[0,0]-.35)
 for step in range(int(a.seconds/env.step_dt)):
  t=step*env.step_dt;stamp_ns=1000000000+round(t*1e9);base=env.robot.data.root_pose_w.torch[0].cpu().numpy().copy();bp=env.robot.data.body_pose_w.torch[0].cpu().numpy().copy()
  state={'names':env.robot.body_names,'bodies':bp.tolist(),'base_pose':base.tolist(),'stamp_ns':stamp_ns};tmp=a.output/'state.tmp';tmp.write_text(json.dumps(state));tmp.replace(a.output/'latest-state.json')
  try:last_tracking=json.loads((a.output/'latest-tracking.json').read_text())
  except (FileNotFoundError,json.JSONDecodeError):pass
  source=last_tracking.get('source_stamp_ns',-1);age=(stamp_ns-source)/1e9 if source>=0 else 1e6;ok=bool(last_tracking.get('tracking_ok',False));new_frame=source!=last_source
  if new_frame and source>=0:
   last_source=source;frames+=1
   if ok and last_tracking.get('base_estimate') is not None:
    est=matrix(last_tracking['base_estimate'])
    if align is None:
     # Initial known spawn is a localization prior, not continuous GT feedback.
     initial=matrix([.35,2.85,float(base[2]),0,0,0,1]);align=initial@np.linalg.inv(est)
    world=align@est;nav_xy=world[:2,3];qworld=quaternion(world[:3,:3]);nav_yaw=math.atan2(2*(qworld[3]*qworld[2]+qworld[0]*qworld[1]),1-2*(qworld[1]**2+qworld[2]**2))
  if a.gt_navigation:nav_xy=base[:2];nav_yaw=float(env.yaw()[0])
  q=float(env.robot.data.joint_pos.torch[0,j]);qd=float(env.robot.data.joint_vel.torch[0,j]);vel=env.robot.data.root_vel_w.torch[0];target=target_for_heading(heading,nav_yaw,previous_target);previous_target=target;err=math.atan2(math.sin(heading-nav_yaw-q),math.cos(heading-nav_yaw-q))
  # Dwell accrues only on distinct tracker samples. Re-reading one success
  # frame is not a new verification event; DRIVE still checks age every tick.
  allow=gate.step(dt=env.step_dt,desired_heading=heading,alignment_error=err,tracking_ok=ok and nav_xy is not None,sample_age=age,reachable=True,base_speed=float(vel[:2].norm()),base_yaw_rate=float(vel[5]),arm_speed=qd)
  if gate.state=='VERIFY' and not new_frame:gate.good_time=max(0.,gate.good_time-env.step_dt)
  env.drive=allow
  if allow:
   if hold is None:hold=q
   env.arm_command[0,0]=hold;action=0.
  elif gate.state!='STOP':
   hold=None;obs=torch.tensor([[target/math.pi,q/math.pi,qd/.5,(target-q)/math.pi,prev_action,max(0.,1-(t-segment_start)/18.)]],device=env.device)
   with torch.no_grad():action=float(model(obs)[0].mean.clamp(-1,1)[0,0])
   env.arm_command[0,0]=max(-math.pi,min(math.pi,q+.5*action*env.step_dt))
  else:hold=None;action=0.;env.arm_command[0,0]=q
  prev_action=action
  desired=np.zeros(2);distance=float('inf')
  if nav_xy is not None:
   delta=waypoints[segment]-nav_xy;distance=float(np.linalg.norm(delta));speed=min(a.max_speed,a.cruise,math.sqrt(max(0.,2*.45*max(0.,distance-.035))),2*distance)
   if allow and distance>1e-6:desired=delta/distance*speed
  # Acceleration limit and immediate commanded braking when the gate closes.
  old=env.velocity.cpu().numpy();dv=desired-old;length=np.linalg.norm(dv)
  if allow and length>.4*env.step_dt:desired=old+dv/length*.4*env.step_dt
  if not allow:desired[:]=0
  env.velocity=torch.tensor(desired,device=env.device,dtype=torch.float32)
  row={'step':step,'sim_s':t,'wall_s':time.monotonic()-begin,'segment':segment,'state':gate.state,'drive_allowed':allow,'tracking_ok':ok,'sample_age':age,'new_image':new_frame,'base_pose':base.tolist(),'navigation_xy':None if nav_xy is None else nav_xy.tolist(),'base_estimate':last_tracking.get('base_estimate'),'arm_yaw':q,'target_yaw':target,'base_speed':float(vel[:2].norm()),'command_speed':float(np.linalg.norm(desired)),'wheel_speed_max':float(env.robot.data.joint_vel.torch[0,env.wheel].abs().max()),'pose_quality':last_tracking.get('pose_quality'),'distance':distance};rows.append(row);log.write(json.dumps(row)+'\n');log.flush();bodies.append(bp);joints.append(env.robot.data.joint_pos.torch[0].cpu().numpy().copy());trajectory.append([t,*base,segment,float(allow),err])
  env.step(torch.zeros((1,2),device=env.device))
  if bool(env.evaluation_failure[0]):reason='physics_failure';break
  if distance<.055:
   segment+=1;gate.reset();hold=None;prev_action=0.;segment_start=t
   if segment==len(waypoints):reason='goal_reached';break
   delta=waypoints[segment]-nav_xy;heading=math.atan2(delta[1],delta[0])
  if step%75==0:print('ASYNC',round(t,1),'segment',segment,'state',gate.state,'age',round(age,3),'speed',round(row['base_speed'],3),'xy',base[:2],'quality',row['pose_quality'],flush=True)
  if stop_requested:reason='interrupted';break
  time.sleep(max(0.,begin+(step+1)*env.step_dt-time.monotonic()))
 log.close();elapsed=time.monotonic()-begin
 report={'result':reason,'completed_segments':segment,'total_segments':len(waypoints),'sim_s':len(rows)*env.step_dt,'wall_s':elapsed,'real_time_factor':len(rows)*env.step_dt/elapsed,'camera_frames':frames,'max_measured_base_speed_m_s':max(x['base_speed'] for x in rows),'max_measured_wheel_rad_s':max(x['wheel_speed_max'] for x in rows),'wheel_speed_limit_rad_s':wheel_omega,'motor_speed_limit_rad_s':motor_omega,'post_gear_speed_limit_m_s':a.max_speed,'gear_speed_multiplier':GEAR,'motor_torque_assumption_Nm':a.motor_torque,'efficiency_assumption':a.efficiency,'wheel_torque_limit_Nm':wheel_torque,'navigation_pose':'simulator GT debug' if a.gt_navigation else 'actual cuVSLAM + same-image arm FK + known start prior','forbidden_drive_frames':sum(x['drive_allowed'] and (not x['tracking_ok'] or x['sample_age']>.25) for x in rows),'final_gt_goal_distance_m':float(np.linalg.norm(base[:2]-waypoints[-1]))}
 (a.output/'report.json').write_text(json.dumps(report,indent=2));np.savez_compressed(a.output/'poses.npz',bodies=np.array(bodies),joints=np.array(joints),trajectory=np.array(trajectory),body_names=np.array(env.robot.body_names));(a.output/'finished').write_text(reason);print('ASYNC_COMPLETE',json.dumps(report),flush=True);env.close()
