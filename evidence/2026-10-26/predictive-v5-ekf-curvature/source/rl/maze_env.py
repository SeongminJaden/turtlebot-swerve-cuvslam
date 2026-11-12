"""Isaac Lab / Isaac Sim PhysX navigation task using the user's maze.
Policy selects planar motion; an analytic wheel allocator handles motor geometry.
Observations currently use simulator pose (privileged training/evaluation input).
"""
from pathlib import Path
import json
import math
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab_physx.physics import PhysxCfg
from isaaclab.utils import configclass
ROOT=Path(__file__).resolve().parents[1]
MAP=json.loads((ROOT/'assets/maze/maze.json').read_text())

@configclass
class MazeSceneCfg(InteractiveSceneCfg):
    light=AssetBaseCfg(prim_path='/World/Light',spawn=sim_utils.DomeLightCfg(intensity=1800.))
    ground=AssetBaseCfg(prim_path='/World/Ground',spawn=sim_utils.GroundPlaneCfg())
    maze=AssetBaseCfg(prim_path='{ENV_REGEX_NS}/Maze',spawn=sim_utils.UsdFileCfg(usd_path=str(ROOT/'assets/maze/maze.usda')))
    robot=ArticulationCfg(prim_path='{ENV_REGEX_NS}/Robot',
        spawn=sim_utils.UsdFileCfg(usd_path=str(ROOT/'assets/m38_runtime.usda')),
        init_state=ArticulationCfg.InitialStateCfg(pos=(.35,2.85,.015)),
        actuators={
            'steer':ImplicitActuatorCfg(joint_names_expr=['.*_steer_joint'],joint_effort_limit=1.,stiffness=30.,damping=3.),
            'wheel':ImplicitActuatorCfg(joint_names_expr=['.*_wheel_joint'],joint_effort_limit=1.,stiffness=0.,damping=1.),
            'arm':ImplicitActuatorCfg(joint_names_expr=['arm_joint.*'],joint_effort_limit=2.,stiffness=150.,damping=8.)})

@configclass
class MazeEnvCfg(DirectRLEnvCfg):
    decimation=4
    episode_length_s=8.
    action_space=2
    observation_space=12
    state_space=0
    sim=sim_utils.SimulationCfg(dt=1/60,render_interval=4,device='cuda:0',physics=PhysxCfg())
    scene=MazeSceneCfg(num_envs=8,env_spacing=9.,replicate_physics=True,clone_in_fabric=False)

class MazeEnv(DirectRLEnv):
    def __init__(self,cfg,**kwargs):
        super().__init__(cfg,**kwargs)
        self.robot=self.scene['robot']
        self.steer=torch.tensor([self.robot.joint_names.index(n+'_steer_joint') for n in ['lh1','rh1','lh2','rh2']],device=self.device)
        self.wheel=torch.tensor([self.robot.joint_names.index(n+'_wheel_joint') for n in ['lh1','rh1','lh2','rh2']],device=self.device)
        self.route=torch.tensor(MAP['route'],device=self.device)
        self.goal=torch.zeros((self.num_envs,2),device=self.device)
        self.previous_distance=torch.zeros(self.num_envs,device=self.device)
        self.actions=torch.zeros((self.num_envs,2),device=self.device)
        self.steering_target=torch.zeros((self.num_envs,4),device=self.device)
        self.wheel_target=torch.zeros_like(self.steering_target)
        self.evaluation=False
        self.successes=0
        self.episodes=0
    def xy(self):
        return self.robot.data.root_pose_w.torch[:,:2]-self.scene.env_origins[:,:2]
    def yaw(self):
        q=self.robot.data.root_pose_w.torch[:,3:7]
        return torch.atan2(2*(q[:,3]*q[:,2]+q[:,0]*q[:,1]),1-2*(q[:,1]**2+q[:,2]**2))
    def _pre_physics_step(self,actions):
        self.actions=actions.clamp(-1,1)
        y=self.yaw();c,s=y.cos(),y.sin()
        wx,wy=.18*self.actions[:,0],.18*self.actions[:,1]
        vx,vy=c*wx+s*wy,-s*wx+c*wy
        wz=(-2*y).clamp(-.5,.5)
        mx=torch.tensor([.1,.1,-.1,-.1],device=self.device)
        my=torch.tensor([.1,-.1,.1,-.1],device=self.device)
        u=vx[:,None]-wz[:,None]*my;v=vy[:,None]+wz[:,None]*mx
        beta=torch.tensor([-math.pi/2,-math.pi/2,0.,math.pi],device=self.device)
        angle=(torch.atan2(v,u)-beta+math.pi)%(2*math.pi)-math.pi
        raw_angle=angle.clone()
        sign=torch.where(angle.abs()>math.pi/2,-1.,1.)
        angle=torch.where(angle>math.pi/2,angle-math.pi,angle)
        angle=torch.where(angle< -math.pi/2,angle+math.pi,angle)
        # Hold the existing branch within 0.06 rad of a steering end stop.
        # Otherwise tiny yaw/XY noise repeatedly requests +90/-90 degree flips.
        hold=((raw_angle.abs()-math.pi/2).abs()<.06)&(self.steering_target.abs()>.7)
        held_angle=self.steering_target.sign()*(math.pi/2)
        angle=torch.where(hold,held_angle,angle)
        sign=torch.where(hold,torch.cos(raw_angle-held_angle).sign(),sign)
        speed=torch.sqrt(u*u+v*v)/.03068719889482*sign
        delta=(angle-self.steering_target).clamp(-self.step_dt,self.step_dt)
        self.steering_target+=delta
        error=angle-self.robot.data.joint_pos.torch[:,self.steer]
        self.wheel_target=speed*torch.cos(error).clamp(min=0)
    def _apply_action(self):
        self.robot.actuators.target_command.set_position_index(value=self.steering_target,joint_ids=self.steer)
        self.robot.actuators.target_command.set_velocity_index(value=self.wheel_target,joint_ids=self.wheel)
    def _get_observations(self):
        delta=(self.goal-self.xy()).clamp(-1,1)
        vel=self.robot.data.root_vel_w.torch
        yaw=self.yaw()
        obs=torch.cat([delta,vel[:,:2],yaw.sin()[:,None],yaw.cos()[:,None],
                       self.robot.data.joint_pos.torch[:,self.steer]/(math.pi/2),self.actions],dim=1)
        return {'policy':obs}
    def _get_rewards(self):
        delta=self.goal-self.xy();dist=delta.norm(dim=1)
        progress=(self.previous_distance-dist)/self.step_dt
        reward=4*progress-.02-.01*(self.actions**2).sum(dim=1)
        reward+=2*(dist<.12).float()
        self.previous_distance=dist.detach().clone()
        return reward
    def _get_dones(self):
        p=self.robot.data.root_pose_w.torch
        bad=(p[:,2]<-.08)|(p[:,2]>.25)|(p[:,3:5].norm(dim=1)>.3)|(~torch.isfinite(p).all(dim=1))
        reached=(self.goal-self.xy()).norm(dim=1)<.12
        timeout=self.episode_length_buf>=self.max_episode_length-1
        if self.evaluation:
            self.evaluation_failure=bad.clone()
            return torch.zeros_like(bad),timeout
        return bad|reached,timeout
    def _reset_idx(self,env_ids):
        if env_ids is None:env_ids=self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        n=len(env_ids)
        idx=torch.randint(12,len(self.route)-13,(n,),device=self.device)
        pose=self.robot.data.default_root_pose.torch[env_ids].clone()
        pose[:,:2]=self.route[idx]+self.scene.env_origins[env_ids,:2]
        pose[:,:2]+=(torch.rand((n,2),device=self.device)-.5)*.10
        pose[:,2]=.015
        pose[:,3:]=torch.tensor([0.,0.,0.,1.],device=self.device)
        self.robot.write_root_pose_to_sim_index(root_pose=pose,env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros((n,6),device=self.device),env_ids=env_ids)
        pos=self.robot.data.default_joint_pos.torch[env_ids].clone()
        self.robot.write_joint_position_to_sim_index(position=pos,env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(pos),env_ids=env_ids)
        direction=torch.where(torch.rand(n,device=self.device)<.5, -12, 12)
        target_idx=(idx+direction).clamp(0,len(self.route)-1)
        self.goal[env_ids]=self.route[target_idx]
        self.previous_distance[env_ids]=(self.goal[env_ids]-pose[:,:2]+self.scene.env_origins[env_ids,:2]).norm(dim=1)
        self.steering_target[env_ids]=pos[:,self.steer]
        self.actions[env_ids]=0
        self.robot.reset(env_ids)
