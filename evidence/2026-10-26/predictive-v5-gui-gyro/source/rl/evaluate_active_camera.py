"""Closed-loop Lab physics + live Kit stereo + actual cuVSLAM status.

Learned yaw alignment controls the arm. Base waypoints use an explicitly
labelled proportional controller and simulator pose, not a navigation PPO.
"""
import argparse,json,socket,math,time,hashlib,signal
from pathlib import Path
import numpy as np
from isaaclab.app import add_launcher_args,launch_simulation
p=argparse.ArgumentParser();p.add_argument('--checkpoint',type=Path,default=Path('runs/camera_ppo_v1/policy.pt'));p.add_argument('--output',type=Path,required=True);p.add_argument('--steps',type=int,default=2400);p.add_argument('--drop-step',type=int,default=-100);add_launcher_args(p);a=p.parse_args()
import torch
from maze_env import MazeEnv,MazeEnvCfg,ROOT
from camera_policy import CameraPolicy
from active_view import ViewGate
from camera_heading import target_for_heading

class ActiveMazeEnv(MazeEnv):
 def __init__(self,cfg):
  super().__init__(cfg);self.arm_ids=torch.tensor([self.robot.joint_names.index('arm_joint'+str(i)) for i in range(1,6)],device=self.device);self.arm_command=torch.zeros((1,5),device=self.device);self.drive=False
 def _pre_physics_step(self,actions):
  super()._pre_physics_step(actions)
  if not self.drive:self.wheel_target[:]=0
 def _apply_action(self):
  super()._apply_action();self.robot.actuators.target_command.set_position_index(value=self.arm_command,joint_ids=self.arm_ids)

cfg=MazeEnvCfg();cfg.scene.num_envs=1;cfg.episode_length_s=10000.;cfg.scene.robot.spawn.usd_path=str(ROOT/'assets/m38_active_camera_demo.usda');cfg.scene.robot.actuators['arm'].joint_velocity_limit=.5
a.output.mkdir(parents=True,exist_ok=True);rows=[];bodies=[];joints=[];trajectory=[];begin=time.monotonic()
stop_requested=False
def stop(*_):
 global stop_requested
 stop_requested=True
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
with launch_simulation(cfg.sim,a):
 env=ActiveMazeEnv(cfg);env.reset();env.evaluation=True;ids=env.robot._ALL_INDICES
 pose=env.robot.data.default_root_pose.torch.clone();pose[0,:3]=torch.tensor([.35,2.85,.015],device=env.device);pose[0,3:]=torch.tensor([0.,0.,0.,1.],device=env.device)
 env.robot.write_root_pose_to_sim_index(root_pose=pose,env_ids=ids);env.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1,6),device=env.device),env_ids=ids)
 model=CameraPolicy().to(env.device);ck=torch.load(a.checkpoint,map_location=env.device,weights_only=True);model.load_state_dict(ck['model']);model.eval()
 sock=socket.create_connection(('127.0.0.1',18766));reader=sock.makefile('r');gate=ViewGate();j=int(env.arm_ids[0]);last_action=0.;segment=0;segment_start=0.;hold=None;previous_target=0.
 # Short manoeuvres inside the same supplied maze cell; return after each axis.
 waypoints=[('front',(.50,2.85),0.),('rear',(.35,2.85),3.05),('right',(.35,2.65),-math.pi/2),('left',(.35,2.85),math.pi/2)]
 reason='step_limit'
 log=open(a.output/'steps.jsonl','w')
 for step in range(a.steps):
  t=step*env.step_dt;stamp_ns=1000000000+round(t*1e9);base=env.robot.data.root_pose_w.torch[0].cpu().numpy().copy();bp=env.robot.data.body_pose_w.torch[0].cpu().numpy().copy()
  drop_images=a.drop_step<=step<a.drop_step+3
  req={'names':env.robot.body_names,'bodies':bp.tolist(),'base_pose':base.tolist(),'stamp_ns':stamp_ns,'drop_images':drop_images};sock.sendall((json.dumps(req)+'\n').encode());tracking=json.loads(reader.readline())
  name,goal,heading=waypoints[segment];yaw=float(env.yaw()[0]);target=target_for_heading(heading,yaw,previous_target);previous_target=target;q=float(env.robot.data.joint_pos.torch[0,j]);qd=float(env.robot.data.joint_vel.torch[0,j]);vel=env.robot.data.root_vel_w.torch[0];err=math.atan2(math.sin(heading-yaw-q),math.cos(heading-yaw-q))
  age=0. if tracking['stamp_ns']==stamp_ns else 1e6
  allow=gate.step(dt=env.step_dt,desired_heading=heading,alignment_error=err,tracking_ok=tracking['tracking_ok'],sample_age=age,reachable=True,base_speed=float(vel[:2].norm()),base_yaw_rate=float(vel[5]),arm_speed=qd)
  env.drive=allow
  if allow:
   if hold is None:hold=q
   env.arm_command[0,0]=hold;action=0.
  elif gate.state!='STOP':
   hold=None;obs=torch.tensor([[target/math.pi,q/math.pi,qd/.5,(target-q)/math.pi,last_action,max(0.,1-(t-segment_start)/18.)]],device=env.device)
   with torch.no_grad():action=float(model(obs)[0].mean.clamp(-1,1)[0,0])
   env.arm_command[0,0]=max(-math.pi,min(math.pi,q+.5*action*env.step_dt))
  else:hold=None;action=0.;env.arm_command[0,0]=q
  last_action=action;env.goal[0]=torch.tensor(goal,device=env.device);base_action=((env.goal-env.xy())*4).clamp(-.5,.5) if allow else torch.zeros((1,2),device=env.device)
  row={'step':step,'sim_s':t,'segment':name,'state':gate.state,'drive_allowed':allow,'tracking_ok':tracking['tracking_ok'],'vo_state':tracking['vo_state'],'base_pose':base.tolist(),'rig_pose':tracking['rig_pose'],'base_estimate':tracking['base_estimate'],'target_yaw':target,'arm_yaw':q,'arm_speed':qd,'alignment_error':err,'base_speed':float(vel[:2].norm()),'sample_age':age,'action':action,'drop_images':drop_images};rows.append(row);log.write(json.dumps(row)+'\n');log.flush();bodies.append(bp);joints.append(env.robot.data.joint_pos.torch[0].cpu().numpy().copy());trajectory.append([t,*base,segment,float(allow),err])
  if step%30==0:print('STEP',step,name,gate.state,'VO',tracking['vo_state'],'error',round(err,3),'xy',base[:2],flush=True)
  env.step(base_action)
  if stop_requested:reason='interrupted_with_evidence_saved';break
  if bool(env.evaluation_failure[0]):reason='physics_failure';break
  if allow and float((env.goal-env.xy()).norm())<.045:
   segment+=1;gate.reset();hold=None;segment_start=t;last_action=0.
   if segment==len(waypoints):reason='all_manoeuvres_completed';break
 log.close();sock.close()
 report={'result':reason,'completed_segments':segment,'total_segments':len(waypoints),'steps':len(rows),'sim_s':len(rows)*env.step_dt,'wall_s':time.monotonic()-begin,'tracking_success_frames':sum(x['tracking_ok'] for x in rows),'drive_frames':sum(x['drive_allowed'] for x in rows),'checkpoint_sha256':hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),'scope':'Live synchronous co-simulation: PhysX + stereo render + actual cuVSLAM status; yaw PPO, base proportional controller using simulator pose; not Nav2 or visual-quality RL.','forbidden_drive_frames':sum(x['drive_allowed'] and (not x['tracking_ok'] or x['sample_age']>.15) for x in rows)}
 (a.output/'report.json').write_text(json.dumps(report,indent=2));np.savez_compressed(a.output/'poses.npz',bodies=np.array(bodies),joints=np.array(joints),trajectory=np.array(trajectory),body_names=np.array(env.robot.body_names));print('ACTIVE_CAMERA_COMPLETE',json.dumps(report),flush=True);env.close()
