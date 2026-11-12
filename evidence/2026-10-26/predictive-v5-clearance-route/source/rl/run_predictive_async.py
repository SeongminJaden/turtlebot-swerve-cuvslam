"""Asynchronous learned acceleration + moving arm camera with live VO/depth.
No ground-truth wall query or continuous ground-truth localization in control.
"""
import argparse,json,math,time,sys,hashlib,os
from pathlib import Path
import numpy as np
from isaaclab.app import add_launcher_args,launch_simulation
p=argparse.ArgumentParser();p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=180);p.add_argument('--route',type=Path,default=Path(os.getenv('ROUTE_FILE','assets/maze/route_clearance.json')));p.add_argument('--robot-asset',type=Path,default=Path('assets/m38_balanced_wheels.usda'));p.add_argument('--use-amcl',action='store_true');p.add_argument('--use-ekf',action='store_true');p.add_argument('--physics-hz',type=int,default=200);add_launcher_args(p);p.set_defaults(device='cpu');a=p.parse_args()
import torch
torch.set_num_threads(1)
from maze_env import MazeEnvCfg,ROOT,MAP
from geared_env import GearedMaze,R
from predictive_env import configure
from predictive_policy import PredictivePolicy
from robot_sensors import configure_sensors,PhysicsSensorStream
from camera_heading import target_for_heading
from turtlebot_swerve.path_speed import PathSpeedEnvelope
sys.path.insert(0,str(ROOT/'scripts'))
from snapshot_runtime import snapshot
snapshot(a.output)
from pose_math import matrix,quaternion
from depth_clearance import corridor_clearance
cfg=configure_sensors(configure(MazeEnvCfg()),contacts=True);cfg.scene.robot.spawn.usd_path=str(a.robot_asset if a.robot_asset.is_absolute() else ROOT/a.robot_asset);cfg.sim.device=a.device or 'cpu';cfg.sim.dt=1/a.physics_hz;cfg.decimation=round(a.physics_hz/15);cfg.sim.render_interval=cfg.decimation;cfg.scene.num_envs=1;cfg.episode_length_s=a.seconds+10
route=np.asarray(json.loads((a.route if a.route.is_absolute() else ROOT/a.route).read_text())['route']);a.output.mkdir(parents=True,exist_ok=True)
path_speed=PathSpeedEnvelope(route)
wrap=lambda x:math.atan2(math.sin(x),math.cos(x))
with launch_simulation(cfg.sim,a):
 env=GearedMaze(cfg,wheel_speed_limit=.92/R);env.wheel_acceleration_limit=.6;env.reset();env.evaluation=True;ids=env.robot._ALL_INDICES
 pose=env.robot.data.default_root_pose.torch.clone();pose[0,:3]=torch.tensor([*route[0],.015],device=env.device);pose[0,3:]=torch.tensor([0,0,0,1.],device=env.device);env.robot.write_root_pose_to_sim_index(root_pose=pose,env_ids=ids);env.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1,6),device=env.device),env_ids=ids)
 q0=env.robot.data.default_joint_pos.torch.clone();q0[0,int(env.arm_ids[2])]=-.20;q0[0,int(env.arm_ids[3])]=-.30;q0[0,int(env.arm_ids[0])]=float(np.arctan2(*(route[4]-route[0])[::-1]));env.robot.write_joint_position_to_sim_index(position=q0,env_ids=ids);env.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q0),env_ids=ids);env.arm_command[:]=q0[:,env.arm_ids];env.robot.reset(ids);env.scene.update(env.step_dt)
 # Let the initial contact and arm servo settle before the first stereo exposure.
 for _ in range(30):env.step(torch.zeros((1,2),device=env.device))
 sensors=PhysicsSensorStream(env)
 model=PredictivePolicy().to(env.device);ck=torch.load(a.checkpoint,map_location=env.device,weights_only=True);model.load_state_dict(ck['model']);model.eval();j=int(env.arm_ids[0])
 bootstrap=[];align=None;nav=None;nav_yaw=0.;nav_speed=0.;prior_nav=None;prior_stamp=None;source=-1;seen_vo=-1;last={};index=0;command=0.;known=0.;view_age=0.;known_heading=0.;prev_target=0.;prev_action=np.zeros(2);rows=[];bodies=[];joints=[];reason='timeout';begin=time.monotonic();dwell=0;fault_since=None
 for step in range(int(a.seconds/env.step_dt)):
  stamp=1_000_000_000+round(step*env.step_dt*1e9);base=env.robot.data.root_pose_w.torch[0].cpu().numpy().copy();bp=env.robot.data.body_pose_w.torch[0].cpu().numpy().copy()
  sensor_packet=sensors.latest;sensor_tmp=a.output/'sensors.tmp';sensor_tmp.write_text(json.dumps(sensor_packet));sensor_tmp.replace(a.output/'latest-sensors.json')
  state={'stamp_ns':stamp,'base_pose':base.tolist(),'bodies':bp.tolist(),'names':env.robot.body_names,'encoder_rig_matrix':sensor_packet['encoder_rig_matrix'],'encoder_left_camera_matrix':sensor_packet['encoder_left_camera_matrix'],'drop_images':float(os.getenv('FAULT_CAMERA_START','-1'))>=0 and float(os.getenv('FAULT_CAMERA_START','-1'))<=step*env.step_dt<float(os.getenv('FAULT_CAMERA_START','-1'))+float(os.getenv('FAULT_CAMERA_DURATION','1'))};tmp=a.output/'state.tmp';tmp.write_text(json.dumps(state));tmp.replace(a.output/'latest-state.json')
  try:last=json.loads((a.output/'latest-tracking.json').read_text())
  except (FileNotFoundError,json.JSONDecodeError):pass
  age=(stamp-last.get('source_stamp_ns',-1))/1e9;fresh=0<=age<=.3 and last.get('tracking_ok',False);new=last.get('source_stamp_ns',-1)!=seen_vo;seen_vo=last.get('source_stamp_ns',-1)
  navigation=None;map_alignment=None
  if a.use_amcl:
   try:
    transform=json.loads((a.output/'latest-map-transform.json').read_text());amcl=json.loads((a.output/'latest-amcl.json').read_text())
    if -.25<=(stamp-transform['stamp_ns'])/1e9<1. and max(amcl['covariance'][0],amcl['covariance'][7])<.01:map_alignment=np.asarray(transform['matrix'])
   except (FileNotFoundError,json.JSONDecodeError):pass
  if a.use_ekf:
   try:
    filtered=json.loads((a.output/'latest-filtered.json').read_text())
    if 0<=(stamp-filtered['stamp_ns'])/1e9<=.1:navigation=(filtered['pose'],filtered['stamp_ns'])
   except (FileNotFoundError,json.JSONDecodeError):pass
  elif last.get('tracking_ok') and last.get('base_estimate'):navigation=(last['base_estimate'],last['source_stamp_ns'])
  if a.use_amcl and map_alignment is None:navigation=None
  if navigation is not None and navigation[1]>source:
   est=matrix(navigation[0]);source=navigation[1]
   if a.use_amcl and map_alignment is not None:align=map_alignment
   if align is None and new and fresh:
    bootstrap.append(est);bootstrap=bootstrap[-10:]
    spread=max(np.linalg.norm(x[:3,3]-est[:3,3]) for x in bootstrap)
    rotation_spread=max(np.arccos(np.clip((np.trace(x[:3,:3].T@est[:3,:3])-1)/2,-1,1)) for x in bootstrap)
    if len(bootstrap)==10 and spread<.005 and rotation_spread<math.radians(.5):
     align=matrix([*route[0],.015,0,0,0,1])@np.linalg.inv(est)
     print('VO_BOOTSTRAP_STABLE',source,flush=True)
   if align is not None:
    world=align@est;nav=world[:2,3];nav_yaw=math.atan2(world[1,0],world[0,0])
    if a.use_ekf:nav_speed=float(np.linalg.norm(filtered['body_velocity'][:2]))
    elif prior_nav is not None and source>prior_stamp:nav_speed=.7*nav_speed+.3*float(np.linalg.norm(nav-prior_nav)/((source-prior_stamp)/1e9))
    prior_nav=nav.copy();prior_stamp=source
  measured=None
  try:measured=json.loads((a.output/'latest-observations.json').read_text())
  except (FileNotFoundError,json.JSONDecodeError):pass
  sensor_age=float('inf') if measured is None else (stamp-measured['stamp_ns'])/1e9
  sensor_ok=measured is not None and 0<=sensor_age<=.1 and abs(measured['imu_stamp_ns']-measured['joint_stamp_ns'])<=20_000_000 and measured['tilt_xyzw'] is not None
  policy_packet={'encoders':sensor_packet['encoders'],'fused':None,'tilt_xyzw':None}
  if sensor_ok:policy_packet={'encoders':measured['encoders'],'fused':{'gyro':measured['gyro']},'tilt_xyzw':measured['tilt_xyzw']}
  q=policy_packet['encoders']['position'][j];qd=policy_packet['encoders']['velocity'][j];action=np.zeros(2);speed=0.;heading=0.;cross=0.;distance=float('inf');visible=False
  if nav is not None and policy_packet['tilt_xyzw'] is not None and policy_packet['fused'] is not None:
   candidates=np.arange(max(0,index-2),min(len(route),index+16));distances=np.linalg.norm(route[candidates]-nav,axis=1);index=int(candidates[np.argmin(distances)]);cross=float(distances.min());delta=route[min(index+4,len(route)-1)]-nav;unit=delta/max(np.linalg.norm(delta),1e-6);heading=math.atan2(unit[1],unit[0]);future=route[min(index+int(np.clip(6+nav_speed*24,6,24)),len(route)-1)]-nav;future_heading=math.atan2(future[1],future[0]);future_heading=heading+float(np.clip(wrap(future_heading-heading),-math.radians(20),math.radians(20))) if ck.get('environment_version') in ('v4','v5') else future_heading;target=target_for_heading(future_heading,nav_yaw,prev_target);prev_target=target;distance=float(np.linalg.norm(route[-1]-nav))
   known=max(0.,known-nav_speed*env.step_dt);view_age+=env.step_dt
   if abs(wrap(heading-known_heading))>math.radians(5):known=0.
   # Latest depth points and camera optical orientation belong to the source pose.
   # Reproject through the VO+FK pose; world GT is never used here.
   if fresh and last.get('rig_pose') and last.get('depth_forward_valid_fraction',0)>.9:
    forward=last.get('camera_forward_base',[1.,0.,0.]);source_camera_heading=nav_yaw+math.atan2(forward[1],forward[0])
    visible=abs(wrap(heading-source_camera_heading))<math.radians(30)
    if visible and new:
     pts=np.asarray(last.get('depth_obstacles_base',[])).reshape(-1,2)
     known=max(0.,corridor_clearance(pts,heading-nav_yaw)-nav_speed*age);known_heading=heading;view_age=age
   # Complementary tilt and gyro from the two offset PhysX IMUs.
   obs=np.array([target/math.pi,q/math.pi,qd/.5,(target-q)/math.pi,prev_action[1],(1-index/(len(route)-1)) if ck.get('environment_version') in ('v4','v5') else max(0.,1-step*env.step_dt/(a.seconds+10)),nav_speed/.95,command/.95,distance/2,math.sin(future_heading-nav_yaw),math.cos(future_heading-nav_yaw),math.sin(heading-nav_yaw),math.cos(heading-nav_yaw),known/3,view_age/2,float(visible),wrap(future_heading-heading)/math.pi,index/(len(route)-1),prev_action[0],cross,*np.asarray(policy_packet['tilt_xyzw'][:2])/.3,*np.asarray(policy_packet['fused']['gyro'][:2])/2],np.float32)
   with torch.no_grad():action=model(torch.tensor(obs.clip(-5,5)[None],device=env.device))[0].mean.clamp(-1,1)[0].cpu().numpy()
   command=float(np.clip(command+.6*action[0]*env.step_dt,0,.95));cap=math.sqrt(1.2*max(0.,known-.32));route_cap=path_speed.at(index,nav_speed)
   if index>=len(route)-3:route_cap=min(.15,2*distance) if distance>.04 else 0.
   speed=min(command,cap,route_cap) if fresh and view_age<=1.5 else 0.;command=speed
   env.velocity=torch.tensor([unit*speed],device=env.device,dtype=torch.float32)
  else:env.velocity.zero_();command=0.
  env.measured_yaw=torch.tensor([nav_yaw],device=env.device);env.measured_steer=torch.tensor([[policy_packet['encoders']['position'][int(k)] for k in env.steer]],device=env.device)
  env.drive=bool(sensor_ok and fresh);env.arm_command[0,0]=float(np.clip(q+.5*action[1]*env.step_dt,-math.pi,math.pi));prev_action=action
  env.step(torch.zeros((1,2),device=env.device));actual_speed=float(env.robot.data.root_vel_w.torch[0,:2].norm())
  row={'sim_s':(step+1)*env.step_dt,'route_index':index,'bootstrap_ready':align is not None,'base_xy':base[:2].tolist(),'base_quaternion':base[3:].tolist(),'nav_yaw':nav_yaw,'nav_speed_m_s':nav_speed,'nav_xy':None if nav is None else nav.tolist(),'tracking_fresh':bool(fresh),'image_age_s':age,'sensor_age_s':sensor_age if math.isfinite(sensor_age) else None,'sensor_ok':bool(sensor_ok),'navigation_age_s':(stamp-source)/1e9 if source>=0 else None,'speed_m_s':actual_speed,'actual_wheel_speed_max_rad_s':float(env.robot.data.joint_vel.torch[0,env.wheel].abs().max()),'actual_yaw_rate_rad_s':float(env.robot.data.root_vel_w.torch[0,5]),'command_m_s':speed,'path_speed_cap_m_s':path_speed.at(index,nav_speed),'arm_speed_rad_s':qd,'actions':action.tolist(),'known_clearance_m':known,'view_age_s':view_age,'cross_track_m':cross};rows.append(row)
  if step%3==0:bodies.append(bp);joints.append(env.robot.data.joint_pos.torch[0].cpu().numpy().copy())
  dwell=dwell+env.step_dt if distance<.04 and actual_speed<.10 else 0.
  if last.get('pose_quality') in ('pose_jump_or_gap','invalid_pose','invalid_quaternion'):
   if fault_since is None:fault_since=step*env.step_dt
   if step*env.step_dt-fault_since>2.:reason='tracking_quality_fault';break
  if sensors.peak_contact_force>.5:reason='evaluation_contact_failure';break
  if dwell>=.3:reason='success';break
  if bool(env.evaluation_failure[0]) or cross>.4:reason='physics_or_path_failure';break
  if step%75==0:print('PREDICTIVE_ASYNC',json.dumps(row),flush=True)
  time.sleep(max(0.,begin+(step+1)*env.step_dt-time.monotonic()))
 elapsed=time.monotonic()-begin;report={'outcome':reason,'checkpoint_sha256':hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),'iteration':ck['iteration'],'robot_asset':cfg.scene.robot.spawn.usd_path,'navigation_estimator':'AMCL + robot_localization' if a.use_amcl else 'robot_localization' if a.use_ekf else 'cuVSLAM_pose','physics_hz':a.physics_hz,'sensor_packets_sent':sensors.sent,'sensor_send_drops':sensors.dropped,'sim_seconds':len(rows)*env.step_dt,'wall_seconds':elapsed,'real_time_factor':len(rows)*env.step_dt/elapsed,'route_index':index,'route_last_index':len(route)-1,'post_gear_speed_limit_m_s':.95,'wheel_command_linear_limit_m_s':.92,'wheel_acceleration_guard_m_s2':.6,'path_steering_rate_limit_rad_s':.6,'path_braking_m_s2':.4,'gear_speed_multiplier':4.,'control_sensor_change':'v5 PPO + ROS quantized encoders/dual IMU; deployment posture and guards separately validated','cuvslam_tracking_mode':int(os.getenv('CUVSLAM_TRACKING_MODE','0')),'motor_torque_assumption_Nm':1.,'max_speed_m_s':max(r['speed_m_s'] for r in rows),'physics_peak_speed_m_s':sensors.peak_speed,'physics_peak_wheel_speed_rad_s':sensors.peak_wheel_speed,'peak_nonwheel_contact_force_N':sensors.peak_contact_force,'nonwheel_contact_samples':sensors.contact_samples,'peak_contact_body':sensors.peak_contact_body,'first_contact_stamp_ns':sensors.first_contact_stamp_ns,'actual_goal_error_m':float(np.linalg.norm(env.robot.data.root_pose_w.torch[0,:2].cpu().numpy()-route[-1])),'max_actual_wheel_speed_rad_s':max(r['actual_wheel_speed_max_rad_s'] for r in rows),'fault_imu2_start_s':os.getenv('FAULT_IMU2_START'),'fault_camera_start_s':os.getenv('FAULT_CAMERA_START'),'simultaneous_motion_fraction':sum(r['speed_m_s']>.03 and abs(r['arm_speed_rad_s'])>.03 for r in rows)/len(rows),'rows':rows}
 report['validated_success']=bool(reason=='success' and report['actual_goal_error_m']<.10 and sensors.peak_speed<=.951 and sensors.peak_contact_force<=.5)
 (a.output/'report.json').write_text(json.dumps(report,indent=2));np.savez_compressed(a.output/'poses.npz',bodies=np.array(bodies),joints=np.array(joints),body_names=np.array(env.robot.body_names));(a.output/'finished').write_text(reason);print('PREDICTIVE_ASYNC_COMPLETE',json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True);env.close()
