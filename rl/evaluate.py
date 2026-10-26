"""Evaluate a saved PPO policy on the supplied maze without pose teleportation."""
import argparse,json,traceback,hashlib
from pathlib import Path
import numpy as np
from isaaclab.app import add_launcher_args,launch_simulation
p=argparse.ArgumentParser();p.add_argument('--checkpoint',type=Path,default=Path('runs/maze_ppo/policy.pt'));p.add_argument('--output',type=Path,default=Path('evidence/2026-10-26/maze-eval'));p.add_argument('--steps',type=int,default=2400);p.add_argument('--video',action='store_true');p.add_argument('--baseline',action='store_true');add_launcher_args(p);args=p.parse_args()
import torch
from maze_env import MazeEnv,MazeEnvCfg,MAP
from policy import Policy
cfg=MazeEnvCfg();cfg.scene.num_envs=1;cfg.episode_length_s=10000.
if args.video:
    from isaaclab_visualizers.kit import KitVisualizerCfg
    from isaaclab.envs.utils.video_recorder_cfg import VideoRecorderCfg
    cfg.sim.visualizer_cfgs=[KitVisualizerCfg(headless=True,eye=(2.2,1.55,7.2),lookat=(2.2,1.55,0.),window_width=960,window_height=720)]
    cfg.video_recorders=[VideoRecorderCfg(source='visualizer:kit',output_dir=str(args.output),video_length=args.steps,frame_stride=3,fps=5,output_filename_prefix='isaacsim_maze')]
completed=False
with launch_simulation(cfg.sim,args):
    try:
        env=MazeEnv(cfg);env.reset();env.evaluation=True
        model=Policy().to(env.device)
        checkpoint=torch.load(args.checkpoint,map_location=env.device,weights_only=True);model.load_state_dict(checkpoint['model']);model.eval()
        ids=env.robot._ALL_INDICES
        pose=env.robot.data.default_root_pose.torch.clone();pose[0,:2]=env.route[0]+env.scene.env_origins[0,:2];pose[0,2]=.015;pose[0,3:]=torch.tensor([0.,0.,0.,1.],device=env.device)
        env.robot.write_root_pose_to_sim_index(root_pose=pose,env_ids=ids)
        env.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1,6),device=env.device),env_ids=ids)
        env.robot.reset(ids);env.scene.update(env.step_dt)
        records=[];bodies=[];joints=[];waypoint=8;reason='step_limit';closest=0
        for step in range(args.steps):
            xy=env.xy()[0]
            # Bounded local search cannot jump across a wall to a distant route segment.
            end=min(closest+15,len(env.route));closest+=int((env.route[closest:end]-xy).norm(dim=1).argmin())
            waypoint=min(closest+6,len(env.route)-1)
            env.goal[0]=env.route[waypoint]
            obs=env._get_observations()['policy']
            with torch.no_grad():
                action=(env.goal-env.xy())*4 if args.baseline else model(obs)[0].mean
            ob,reward,terminated,truncated,_=env.step(action)
            pose=env.robot.data.root_pose_w.torch[0].cpu().numpy().copy()
            records.append([step*env.step_dt,*pose,waypoint,*action[0].cpu().numpy()])
            bodies.append(env.robot.data.body_pose_w.torch[0].cpu().numpy().copy())
            joints.append(env.robot.data.joint_pos.torch[0].cpu().numpy().copy())
            if step%150==0:print('EVAL',step,'waypoint',waypoint,'pose',pose[:3].tolist(),flush=True)
            if bool(env.evaluation_failure[0]) or bool(terminated[0]) or bool(truncated[0]):reason='physics_failure_or_timeout';break
            if step>300 and np.linalg.norm(data_xy:=np.array(records[-1][1:3])-np.array(records[-301][1:3]))<.025:
                reason='stalled';break
            if waypoint==len(env.route)-1 and float((env.xy()[0]-env.route[-1]).norm())<.12:reason='goal_reached';break
        args.output.mkdir(parents=True,exist_ok=True)
        data=np.asarray(records)
        np.savetxt(args.output/'trajectory.csv',data,delimiter=',',header='time,x,y,z,qx,qy,qz,qw,waypoint,action_x,action_y',comments='')
        np.savez_compressed(args.output/'poses.npz',bodies=np.array(bodies),joints=np.array(joints),body_names=np.array(env.robot.body_names),trajectory=data)
        report={'result':reason,'success':reason=='goal_reached','steps':len(data),'simulated_s':len(data)*env.step_dt,'route_points':len(env.route),'last_waypoint':waypoint,'checkpoint_iteration':None if args.baseline else checkpoint['iteration'],'checkpoint_sha256':None if args.baseline else hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),'policy':'proportional_debug_baseline' if args.baseline else 'PPO deterministic mean','pose_source':'simulator ground truth; not cuVSLAM','physics':'Isaac Sim PhysX via Isaac Lab','final_goal_error_m':float((env.xy()[0]-env.route[-1]).norm()),'body_names':env.robot.body_names,'max_abs_height_m':float(np.abs(data[:,3]).max()),'allocator':'steering end-stop branch hold 0.06 rad','environment_sha256':hashlib.sha256(Path(__file__).with_name('maze_env.py').read_bytes()).hexdigest(),'maze_sha256':hashlib.sha256((Path(__file__).resolve().parents[1]/'assets/maze/maze.usda').read_bytes()).hexdigest()}
        (args.output/'report.json').write_text(json.dumps(report,indent=2));print('EVALUATION_COMPLETE',json.dumps(report),flush=True);completed=True;env.close()
    except BaseException:traceback.print_exc();raise

if not completed:raise SystemExit(1)
