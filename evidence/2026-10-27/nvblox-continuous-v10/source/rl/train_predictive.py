"""Small reproducible PPO trainer on actual Isaac Lab / PhysX maze transitions."""
import argparse,json,time,traceback,sys,hashlib,subprocess
from pathlib import Path
from isaaclab.app import add_launcher_args,launch_simulation
p=argparse.ArgumentParser();p.add_argument('--environment',choices=['v3','v4','v5'],default='v3');p.add_argument('--iterations',type=int,default=100);p.add_argument('--num-envs',type=int,default=8);p.add_argument('--resume',type=Path);p.add_argument('--warmstart',type=Path);p.add_argument('--seed',type=int,default=7);p.add_argument('--output',type=Path,default=Path('runs/camera_ppo'));add_launcher_args(p);args=p.parse_args()
import torch
from torch import nn
from torch.distributions import Normal
if args.environment=='v5':
 from predictive_env_v5 import PredictiveEnv as MazeEnv,PredictiveEnvCfg as MazeEnvCfg,configure
elif args.environment=='v4':
 from predictive_env_v4 import PredictiveEnv as MazeEnv,PredictiveEnvCfg as MazeEnvCfg,configure
else:
 from predictive_env import PredictiveEnv as MazeEnv,PredictiveEnvCfg as MazeEnvCfg,configure
from predictive_policy import PredictivePolicy as Policy
cfg=configure(MazeEnvCfg());cfg.scene.num_envs=args.num_envs;cfg.seed=args.seed
torch.manual_seed(args.seed)
args.output.mkdir(parents=True,exist_ok=True)
source_dir=Path(__file__).resolve().parent
metadata={'argv':sys.argv,'seed':args.seed,'num_envs':args.num_envs,'rollout_steps':128,
          'physics':'Isaac Sim PhysX','physics_dt':cfg.sim.dt,'decimation':cfg.decimation,
          'source_hashes':{},'git_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=source_dir,text=True).strip()}
for name in ('predictive_env.py','predictive_env_v4.py','predictive_env_v5.py','robot_sensors.py','predictive_policy.py','train_predictive.py','geared_env.py'):
    content=(source_dir/name).read_bytes();metadata['source_hashes'][name]=hashlib.sha256(content).hexdigest()
    (args.output/(name+'.snapshot')).write_bytes(content)
(args.output/'metadata.json').write_text(json.dumps(metadata,indent=2))
completed=False
with launch_simulation(cfg.sim,args):
    try:
        env=MazeEnv(cfg);obs,_=env.reset();obs=obs['policy']
        model=Policy().to(env.device);model.initialize_camera(torch.load(source_dir.parent/'runs/camera_ppo_v1/policy.pt',map_location=env.device,weights_only=True)['model']);optimizer=torch.optim.Adam(model.parameters(),lr=3e-4)
        if args.warmstart:
            prior=torch.load(args.warmstart,map_location=env.device,weights_only=True)['model'];state=model.state_dict()
            for key,value in prior.items():
                if key in ('actor.0.weight','critic.0.weight') and value.shape[1]<state[key].shape[1]:
                    state[key].zero_();state[key][:,:value.shape[1]]=value
                else:state[key]=value
            model.load_state_dict(state)
        start_iteration=0
        if args.resume:
            saved=torch.load(args.resume,map_location=env.device,weights_only=True);model.load_state_dict(saved['model']);start_iteration=saved['iteration']
            if 'optimizer' in saved:optimizer.load_state_dict(saved['optimizer'])
        args.output.mkdir(parents=True,exist_ok=True)
        history=[];begin=time.monotonic();horizon=128
        for iteration in range(start_iteration,start_iteration+args.iterations):
            oo=[];aa=[];ll=[];vv=[];rr=[];dd=[]
            for _ in range(horizon):
                with torch.no_grad():
                    distribution,value=model(obs);act=distribution.sample();logp=distribution.log_prob(act).sum(-1)
                oo.append(obs.clone());aa.append(act);ll.append(logp);vv.append(value)
                ob,reward,terminated,truncated,_=env.step(act)
                rr.append(reward);dd.append((terminated|truncated).float());obs=ob['policy']
            with torch.no_grad():_,nextvalue=model(obs)
            advantage=torch.zeros_like(torch.stack(rr));gae=torch.zeros(args.num_envs,device=env.device)
            for t in reversed(range(horizon)):
                nv=nextvalue if t==horizon-1 else vv[t+1]
                delta=rr[t]+.99*nv*(1-dd[t])-vv[t]
                gae=delta+.99*.95*(1-dd[t])*gae;advantage[t]=gae
            returns=advantage+torch.stack(vv)
            batchobs=torch.stack(oo).flatten(0,1);batchact=torch.stack(aa).flatten(0,1);oldlog=torch.stack(ll).flatten()
            adv=advantage.flatten();adv=(adv-adv.mean())/(adv.std()+1e-8);ret=returns.flatten()
            for epoch in range(4):
                for idx in torch.randperm(len(adv),device=env.device).split(256):
                    dist,val=model(batchobs[idx]);lp=dist.log_prob(batchact[idx]).sum(-1)
                    ratio=(lp-oldlog[idx]).exp()
                    actor=-torch.minimum(ratio*adv[idx],ratio.clamp(.8,1.2)*adv[idx]).mean()
                    loss=actor+.5*((val-ret[idx])**2).mean()-.005*dist.entropy().sum(-1).mean()
                    optimizer.zero_grad();loss.backward();nn.utils.clip_grad_norm_(model.parameters(),1.);optimizer.step()
            row={'iteration':iteration+1,'transitions':(iteration+1)*horizon*args.num_envs,
                 'mean_reward':torch.stack(rr).mean().item(),'mean_action_std':model.logstd.exp().mean().item(),
                 'success_count':env.success_count,'collision_count':env.collision_count,'episode_count':env.episode_count,'elapsed_s':time.monotonic()-begin}
            history.append(row)
            if iteration%5==0 or iteration==start_iteration+args.iterations-1:
                print(json.dumps(row),flush=True)
                torch.save({'model':model.state_dict(),'optimizer':optimizer.state_dict(),'iteration':iteration+1,'seed':args.seed,'environment_version':args.environment,
                            'observation':'24-dimensional path preview, velocity, camera state, visible clearance, age, body tilt and angular rates',
                            'action':'acceleration 0.6m/s2 and arm yaw velocity 0.5rad/s',
                            'scope':'joint acceleration and anticipatory camera yaw PhysX policy; analytic wall-ray/FOV training proxy, not image encoder','reset_distribution':'random 0.9-1.7m maze route segments, near-path initial yaw camera'},args.output/'policy.pt.tmp')
                (args.output/'policy.pt.tmp').replace(args.output/'policy.pt')
                (args.output/'training.json').write_text(json.dumps(history,indent=2))
        print('TRAINING_COMPLETE',flush=True);completed=True;env.close()
    except BaseException:traceback.print_exc();raise

if not completed:raise SystemExit(1)
