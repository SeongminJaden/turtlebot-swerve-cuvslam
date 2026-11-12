"""Actual PhysX sensor smoke test; ground truth is used only for scoring."""
import argparse,json
from pathlib import Path
from isaaclab.app import add_launcher_args,launch_simulation
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);add_launcher_args(p);a=p.parse_args()
import torch,numpy as np,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from pose_math import matrix
from maze_env import MazeEnvCfg
from predictive_env import configure
from geared_env import GearedMaze
from robot_sensors import configure_sensors,RobotSensors
cfg=configure_sensors(configure(MazeEnvCfg()));cfg.scene.num_envs=1;cfg.episode_length_s=60
rows=[]
with launch_simulation(cfg.sim,a):
    env=GearedMaze(cfg);env.reset();env.evaluation=True;sensors=RobotSensors(env)
    for step in range(150):
        env.drive=step>=45
        env.velocity[:]=torch.tensor([0.,-.12 if step>=45 else 0.],device=env.device)
        env.step(torch.zeros((1,2),device=env.device))
        record=sensors.sample(1_000_000_000+round((step+1)*env.step_dt*1e9))
        base=matrix(env.robot.data.root_pose_w.torch[0].cpu().numpy());tip=matrix(env.robot.data.body_pose_w.torch[0,env.robot.body_names.index('arm_link5')].cpu().numpy());mount=np.eye(4);mount[:3,3]=[.04742,0,.059]
        truth=np.linalg.inv(base)@tip@mount
        record['fk_translation_error_m']=float(np.linalg.norm(truth[:3,3]-np.asarray(record['encoder_rig_matrix'])[:3,3]))
        record['evaluation_speed']=float(env.robot.data.root_vel_w.torch[0,:2].norm())
        rows.append(record)
    a.output.mkdir(parents=True,exist_ok=True)
    (a.output/'samples.json').write_text(json.dumps(rows))
    stationary=[r for r in rows[15:40] if r['fused']]
    report={'physics_simulated_seconds':150*cfg.sim.dt*cfg.decimation,'sample_rate_hz':1/(cfg.sim.dt*cfg.decimation),
     'max_fk_translation_error_m':max(r['fk_translation_error_m'] for r in rows),
     'joint_count':len(rows[-1]['encoders']['names']),'valid_fusion_count':sum(r['fused'] is not None for r in rows),
     'fusion_errors':sorted(set(r['fusion_error'] for r in rows if r['fusion_error'])),
     'rest_accel_mean':np.mean([r['fused']['accel'] for r in stationary],axis=0).tolist(),
     'encoder_changed':rows[-1]['encoders']['motor_counts']!=rows[40]['encoders']['motor_counts'],
     'max_actual_speed':max(r['evaluation_speed'] for r in rows),
     'scope':'PhysX sensor smoke test, not full maze or hardware validation'}
    (a.output/'report.json').write_text(json.dumps(report,indent=2));print('SENSOR_REPORT',json.dumps(report),flush=True)
    assert report['max_fk_translation_error_m']<.005
    assert report['joint_count']==13 and report['encoder_changed'] and report['valid_fusion_count']>100
    assert abs(report['rest_accel_mean'][2]-9.81)<.3
    env.close()
