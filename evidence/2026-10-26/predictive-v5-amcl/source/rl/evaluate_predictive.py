"""Deterministic full-route PhysX evaluation of learned acceleration + camera yaw.
Uses analytic camera-FOV range proxy and simulator localization; NOT live cuVSLAM.
"""
import argparse,json,hashlib,time
from pathlib import Path
import numpy as np
from isaaclab.app import add_launcher_args,launch_simulation
p=argparse.ArgumentParser();p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=180.);add_launcher_args(p);args=p.parse_args()
import torch
from predictive_env import PredictiveEnv,PredictiveEnvCfg,configure
from predictive_policy import PredictivePolicy
cfg=configure(PredictiveEnvCfg());cfg.scene.num_envs=1;cfg.episode_length_s=args.seconds+10
args.output.mkdir(parents=True,exist_ok=True)
with launch_simulation(cfg.sim,args):
 env=PredictiveEnv(cfg);env.reset();env.evaluation=True;ids=env.robot._ALL_INDICES
 model=PredictivePolicy().to(env.device);ck=torch.load(args.checkpoint,map_location=env.device,weights_only=True);model.load_state_dict(ck['model']);model.eval()
 pose=env.robot.data.default_root_pose.torch.clone();pose[:,:2]=env.route[0]+env.scene.env_origins[:,:2];pose[:,2]=.015;pose[:,3:]=torch.tensor([0.,0.,0.,1.],device=env.device)
 env.robot.write_root_pose_to_sim_index(root_pose=pose,env_ids=ids);env.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1,6),device=env.device),env_ids=ids)
 q=env.robot.data.default_joint_pos.torch.clone();q[:,env.j]=0.;env.robot.write_joint_position_to_sim_index(position=q,env_ids=ids);env.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q),env_ids=ids)
 env.index[:]=0;env.start[:]=0;env.end[:]=len(env.route)-1;env.speed_command[:]=0;env.known_clearance[:]=0;env.view_age[:]=0;env.clearance_heading[:]=0;env.cam_target[:]=0;env.previous_xy[:]=env.route[0];env.previous_error[:]=0;env.actions[:]=0;env.previous_actions[:]=0;env.dwell[:]=0;env.arm_command[:]=q[:,env.arm_ids];env.steering_target[:]=q[:,env.steer];env.robot.reset(ids);env.scene.update(env.step_dt);env.geometry(update_sensor=False)
 rows=[];bodies=[];joints=[];begin=time.monotonic();outcome='timeout'
 for step in range(int(args.seconds/env.step_dt)):
  with torch.no_grad():action=model(env._get_observations()['policy'])[0].mean
  env.step(action);g=env.g
  row={'time_s':(step+1)*env.step_dt,'xy':env.xy()[0].cpu().tolist(),'route_index':int(env.index[0]),'speed_m_s':float(g['speed'][0]),'command_m_s':float(env.speed_command[0]),'arm_yaw_rad':float(env.robot.data.joint_pos.torch[0,env.j]),'arm_speed_rad_s':float(env.robot.data.joint_vel.torch[0,env.j]),'acceleration_action':float(action[0,0]),'arm_action':float(action[0,1]),'known_clearance_m':float(env.known_clearance[0]),'view_age_s':float(env.view_age[0]),'visible':bool(g['visible'][0]),'cross_track_m':float(g['cross_track'][0]),'wall_clearance_m':float(g['wall_distance'][0]),'wheel_max_rad_s':float(env.robot.data.joint_vel.torch[0,env.wheel].abs().max()),'bad':bool(env.bad[0])};rows.append(row)
  if step%3==0:bodies.append(env.robot.data.body_pose_w.torch[0].cpu().numpy().copy());joints.append(env.robot.data.joint_pos.torch[0].cpu().numpy().copy())
  if step%150==0:print(json.dumps(row),flush=True)
  if bool(env.bad[0]):outcome='collision_or_departure';break
  if bool(env.success[0]):outcome='success';break
 report={'outcome':outcome,'scope':'full first-maze route; PhysX with simulator localization and analytic FOV range proxy, not live cuVSLAM/depth','checkpoint_sha256':hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),'iteration':ck['iteration'],'sim_seconds':rows[-1]['time_s'],'wall_seconds':time.monotonic()-begin,'route_index':int(env.index[0]),'route_last_index':len(env.route)-1,'goal_distance_m':float(g['goal_distance'][0]),'max_speed_m_s':max(r['speed_m_s'] for r in rows),'max_wheel_rad_s':max(r['wheel_max_rad_s'] for r in rows),'moving_fraction':sum(r['speed_m_s']>.03 for r in rows)/len(rows),'simultaneous_arm_and_base_fraction':sum(r['speed_m_s']>.03 and abs(r['arm_speed_rad_s'])>.03 for r in rows)/len(rows),'rows':rows}
 (args.output/'report.json').write_text(json.dumps(report,indent=2));np.savez_compressed(args.output/'poses.npz',bodies=np.array(bodies),joints=np.array(joints),body_names=np.array(env.robot.body_names));print('EVALUATION_COMPLETE',json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True);env.close()
