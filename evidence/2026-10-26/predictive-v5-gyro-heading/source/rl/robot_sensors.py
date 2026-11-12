"""PhysX IMUs and quantized joint encoders, independent of camera rendering.
Encoder resolution is a configurable simulation assumption, not hardware calibration.
"""
import sys
from collections import deque
from pathlib import Path
import numpy as np
from isaaclab.sensors import ImuCfg,ContactSensorCfg
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from turtlebot_swerve.imu_tilt import ImuTilt
from turtlebot_swerve.arm_kinematics import ArmKinematics
from turtlebot_swerve.dual_imu import DualImuFusion, Sample
POSITIONS=((.0570002786,-.0000186031,.16597),(-.0570002786,.0000186031,.16597))
ROTATIONS=((0.,0.,0.,1.),(0.,0.,1.,0.))

def configure_sensors(cfg,contacts=False):
    if contacts:
        cfg.scene.robot.spawn.activate_contact_sensors=True
        cfg.scene.contacts=ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*",update_period=0.,history_length=1)
    for i in range(2):
        setattr(cfg.scene,f'opencr_{i+1}',ImuCfg(
            prim_path='{ENV_REGEX_NS}/Robot/base_link',update_period=0.,
            offset=ImuCfg.OffsetCfg(pos=POSITIONS[i],rot=ROTATIONS[i])))
    return cfg

class RobotSensors:
    def __init__(self,env,counts_per_revolution=4096):
        self.env=env;self.cpr=counts_per_revolution
        self.fk=ArmKinematics(Path(__file__).resolve().parents[1]/"assets/m38_arm_description/urdf/m38_arm.urdf")
        self.fusion=DualImuFusion(POSITIONS,ROTATIONS)
        self.previous=None;self.encoder_history=deque();self.tilt=ImuTilt()
        self.ratio=np.array([4. if n.endswith('_wheel_joint') else 1. for n in env.robot.joint_names])
    def sample(self,stamp_ns):
        e=self.env;t=stamp_ns/1e9;raw=[];samples=[]
        # ADC resolution/range matches official OpenCR MPU9250/ICM20648 setup.
        for i in range(2):
            d=e.scene[f'opencr_{i+1}'].data
            w=d.ang_vel_b.torch[0].cpu().numpy();f=d.lin_acc_b.torch[0].cpu().numpy()
            scales=np.array([2000/32768*np.pi/180,9.80665/16384])
            adc=np.rint(np.array([w,f])/scales[:,None]);saturated=bool((np.abs(adc)>32767).any())
            quantized=np.clip(adc,-32768,32767)*scales[:,None]
            w,f=quantized
            raw.append(dict(frame_id=('opencr_front_imu','opencr_rear_imu')[i],gyro=w.tolist(),accel=f.tolist(),saturated=saturated))
            samples.append(Sample(t,w,f,np.eye(3)*scales[0]**2/12,np.eye(3)*scales[1]**2/12))
        fused=None;error=None
        if any(v['saturated'] for v in raw):error='IMU ADC saturated';self.fusion.previous=None
        else:
            try:fused=self.fusion.update(samples)
            except ValueError as exc:error=str(exc)
        attitude=None if fused is None else self.tilt.update(t,fused['gyro'],fused['accel'])
        q=e.robot.data.joint_pos.torch[0].cpu().numpy()
        counts=np.rint(q/self.ratio/(2*np.pi)*self.cpr).astype(np.int64)
        measured=counts/self.cpr*(2*np.pi)*self.ratio
        valid_velocity=self.previous is not None
        velocity=np.zeros_like(measured)
        if self.previous is not None and t<=self.previous[0]:raise ValueError('Encoder time must increase')
        self.encoder_history.append((t,measured.copy()))
        while len(self.encoder_history)>2 and self.encoder_history[1][0]<=t-.03:self.encoder_history.popleft()
        if valid_velocity:
            pt,pq=self.encoder_history[0];velocity=(measured-pq)/(t-pt)
        self.previous=(t,measured.copy())
        return dict(stamp_ns=stamp_ns,imu=raw,fusion_error=error,
                    tilt_xyzw=None if attitude is None else attitude.tolist(),
                    encoder_rig_matrix=self.fk.rig(e.robot.joint_names,measured).tolist(),
                    encoder_left_camera_matrix=self.fk.left_camera(e.robot.joint_names,measured).tolist(),
                    fused=None if fused is None else {k:np.asarray(v).tolist() for k,v in fused.items()},
                    encoders=dict(names=e.robot.joint_names,position=measured.tolist(),velocity=velocity.tolist(),
                                  motor_counts=counts.tolist(),counts_per_revolution=self.cpr,
                                  wheel_speed_multiplier=4.,velocity_valid=valid_velocity))

class PhysicsSensorStream:
    """Sample after each actual physics step; timestamp by simulation step counter.
    UDP transport is nonblocking and may drop packets; consumers must reject gaps.
    """
    def __init__(self,env,udp_port=18768):
        import socket
        self.env=env;self.sensor=RobotSensors(env);self.start=env._sim_step_counter
        self.previous=self.start;self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.socket.setblocking(False);self.port=udp_port;self.sent=0;self.dropped=0
        self.peak_speed=0.;self.peak_wheel_speed=0.;self.peak_contact_force=0.;self.contact_samples=0;self.peak_contact_body=None;self.first_contact_stamp_ns=None
        self.latest=self.sensor.sample(1_000_000_000);original=env.scene.update
        def update(dt):
            original(dt)
            counter=env._sim_step_counter
            if counter<=self.previous:return
            self.previous=counter
            self.latest=self.sensor.sample(1_000_000_000+round((counter-self.start)*env.physics_dt*1e9))
            # Privileged scoring only: never included in the published sensor packet.
            self.peak_speed=max(self.peak_speed,float(env.robot.data.root_vel_w.torch[0,:2].norm()))
            self.peak_wheel_speed=max(self.peak_wheel_speed,float(env.robot.data.joint_vel.torch[0,env.wheel].abs().max()))
            if 'contacts' in env.scene.sensors:
                contact=env.scene['contacts'];indices=[i for i,n in enumerate(contact.body_names) if not n.endswith('_wheel')]
                forces=contact.data.net_normal_forces_w.torch[0,indices].norm(dim=-1);which=int(forces.argmax());force=float(forces[which])
                if force>self.peak_contact_force:self.peak_contact_body=contact.body_names[indices[which]]
                if force>.5 and self.first_contact_stamp_ns is None:self.first_contact_stamp_ns=self.latest['stamp_ns']
                self.peak_contact_force=max(self.peak_contact_force,force);self.contact_samples+=int(force>.5)
            self.latest['sequence']=self.sent+self.dropped
            import json
            try:self.socket.sendto(json.dumps(self.latest).encode(),('127.0.0.1',self.port));self.sent+=1
            except BlockingIOError:self.dropped+=1
        env.scene.update=update
