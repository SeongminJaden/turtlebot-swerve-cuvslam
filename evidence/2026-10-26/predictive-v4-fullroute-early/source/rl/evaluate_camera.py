"""Held-out deterministic PhysX evaluation; no camera tracking proxy claim."""
import argparse,json,hashlib,traceback
from pathlib import Path
import numpy as np
from isaaclab.app import add_launcher_args,launch_simulation
p=argparse.ArgumentParser();p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True);add_launcher_args(p);args=p.parse_args()
import torch
from camera_env import CameraEnv,CameraEnvCfg
from camera_policy import CameraPolicy
cfg=CameraEnvCfg();cfg.scene.num_envs=16;cfg.episode_length_s=1000.
rng=np.random.default_rng(12345)
cases=[]
for direction,a in [('front',0.),('left',np.pi/2),('right',-np.pi/2),('rear_positive',3.05),('rear_negative',-3.05)]:
 for q in np.linspace(-2.8,2.8,12):cases.append(dict(direction=direction,target=a,initial=float(q)))
for _ in range(68):cases.append(dict(direction='random',target=float(rng.uniform(-3.,3.)),initial=float(rng.uniform(-2.8,2.8)) ))
args.output.mkdir(parents=True,exist_ok=True);records=[];saved_bodies=[];saved_joints=[];saved_traj=[]
with launch_simulation(cfg.sim,args):
 env=CameraEnv(cfg);env.reset();env.evaluation=True
 model=CameraPolicy().to(env.device);ck=torch.load(args.checkpoint,map_location=env.device,weights_only=True);model.load_state_dict(ck['model']);model.eval()
 for start in range(0,len(cases),16):
  batch=cases[start:start+16];env.reset();ids=env.robot._ALL_INDICES
  q=env.robot.data.default_joint_pos.torch.clone();q[:,env.j]=torch.tensor([x['initial'] for x in batch],device=env.device)
  env.robot.write_joint_position_to_sim_index(position=q,env_ids=ids);env.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q),env_ids=ids)
  env.command=q.clone();env.target=torch.tensor([x['target'] for x in batch],device=env.device);env.previous_error=(env.target-q[:,env.j]).abs();env.dwell[:]=0;env.actions[:]=0;env.robot.reset(ids);env.scene.update(env.step_dt)
  first=np.full(16,np.nan);maxspeed=np.zeros(16);finite=True
  for step in range(270):
   obs=env._get_observations()['policy'];obs[:,-1]=max(0.,1-step/270)
   with torch.no_grad():action=model(obs)[0].mean
   env.step(action)
   ok=env.success.cpu().numpy();first[np.isnan(first)&ok]=(step+1)*env.step_dt
   maxspeed=np.maximum(maxspeed,env.robot.data.joint_vel.torch[:,env.j].abs().cpu().numpy());finite &= bool(torch.isfinite(env.robot.data.joint_pos.torch).all())
   if start==0:
    # First trial starts at -2.8 rad and points forward; actual body poses.
    saved_bodies.append(env.robot.data.body_pose_w.torch[0].cpu().numpy().copy());saved_joints.append(env.robot.data.joint_pos.torch[0].cpu().numpy().copy())
    saved_traj.append([step*env.step_dt,*env.robot.data.root_pose_w.torch[0].cpu().numpy(),float(env.target[0]),float(action[0,0]),float(env.error()[0])])
  errors=env.error().abs().cpu().numpy()
  for i,c in enumerate(batch):records.append(c|{'success':bool(np.isfinite(first[i])),'settle_s':None if np.isnan(first[i]) else float(first[i]),'final_error_rad':float(errors[i]),'max_yaw_speed_rad_s':float(maxspeed[i]),'finite':finite})
  print('BATCH',start,'success',sum(x['success'] for x in records),'of',len(records),flush=True)
 summary={d:{'successes':sum(x['success'] for x in records if x['direction']==d),'trials':sum(x['direction']==d for x in records)} for d in sorted({x['direction'] for x in records})}
 report={'checkpoint_sha256':hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),'iteration':ck['iteration'],'physics':'Isaac Sim PhysX via Isaac Lab','evaluation_seed':12345,'success_definition':'abs yaw error <0.12rad and yaw speed <0.07rad/s continuously for0.5s within18s','scope':'fixed-base arm_joint1 alignment. No live cuVSLAM or navigation validation.','results':records,'by_direction':summary,'successes':sum(x['success'] for x in records),'trials':len(records)}
 (args.output/'report.json').write_text(json.dumps(report,indent=2));np.savez_compressed(args.output/'poses.npz',bodies=np.array(saved_bodies),joints=np.array(saved_joints),body_names=np.array(env.robot.body_names),trajectory=np.array(saved_traj))
 print('CAMERA_EVAL_COMPLETE',json.dumps(summary),flush=True);env.close()
