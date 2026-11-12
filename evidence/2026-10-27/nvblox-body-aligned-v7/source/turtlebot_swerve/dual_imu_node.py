"""Timestamp-interpolated dual OpenCR IMU ROS 2 adapter."""
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from .dual_imu import DualImuFusion,Sample,interpolate


class DualImuNode(Node):
    def __init__(self, **node_options):
        super().__init__('dual_imu_fusion', **node_options)
        def param(n,v):return self.declare_parameter(n,v).value
        if not param('geometry_verified',False):
            raise ValueError('Set measured mounting positions/axes and geometry_verified; simulation profile is separate.')
        self.frame=param('output_frame','base_link');self.frames=[param('imu1.frame','opencr_front_imu'),param('imu2.frame','opencr_rear_imu')]
        positions=[param(f'imu{i}.position_m',[float('nan')]*3) for i in (1,2)]
        rotations=[param(f'imu{i}.quaternion_xyzw',[float('nan')]*4) for i in (1,2)]
        gb=[param(f'imu{i}.gyro_bias_rad_s',[0.,0.,0.]) for i in (1,2)]
        ab=[param(f'imu{i}.accel_bias_m_s2',[0.,0.,0.]) for i in (1,2)]
        self.offset=[param(f'imu{i}.time_offset_s',0.) for i in (1,2)]
        self.gvar=param('unknown_gyro_variance',.0004);self.avar=param('unknown_accel_variance',.04)
        self.max_age=param('max_age_s',.1);self.max_gap=param('interpolation_max_gap_s',.03)
        self.fusion=DualImuFusion(positions,rotations,gb,ab,alpha_tau=param('alpha_filter_tau_s',.03),max_gap=param('derivative_max_gap_s',.1),gyro_difference=param('gyro_difference_limit_rad_s',.35),accel_difference=param('accel_difference_limit_m_s2',2.))
        self.buffers=[deque(maxlen=100),deque(maxlen=100)];self.last_stamp=-1.;self.last_received=None
        self.output=self.create_publisher(Imu,param('output_topic','/imu/base/data_raw'),10)
        self.corrected=[self.create_publisher(Imu,f'/imu/base/corrected_{i}',10) for i in (1,2)]
        self.diag=self.create_publisher(DiagnosticArray,'/imu/diagnostics',10)
        self.subs=[self.create_subscription(Imu,param(f'imu{i+1}.topic',f'/opencr_{i+1}/imu'),lambda m,j=i:self.receive(m,j),qos_profile_sensor_data) for i in range(2)]
        self.create_timer(.2,self.watchdog)

    def status(self,level,text,values=None):
        s=DiagnosticStatus();s.name='dual_imu_fusion';s.hardware_id='two_rigid_body_IMUs';s.level=bytes([level]) if isinstance(s.level,bytes) else level;s.message=text
        s.values=[KeyValue(key=k,value=str(v)) for k,v in (values or {}).items()]
        d=DiagnosticArray();d.header.stamp=self.get_clock().now().to_msg();d.status=[s];self.diag.publish(d)

    def covariance(self,values,fallback):
        if values[0]<0:raise ValueError('Required gyro/acceleration measurement unavailable')
        a=np.asarray(values).reshape(3,3)
        if not np.isfinite(a).all():raise ValueError('Invalid covariance')
        if np.all(a==0):return np.eye(3)*fallback
        if not np.allclose(a,a.T,atol=1e-9) or np.linalg.eigvalsh(a).min()<-1e-10:raise ValueError('Covariance must be positive semidefinite')
        return a

    def receive(self,msg,i):
        try:
            if msg.header.frame_id!=self.frames[i]:raise ValueError('Unexpected IMU frame; mounting transform would be wrong')
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9+self.offset[i]
            now=self.get_clock().now().nanoseconds*1e-9
            if stamp<=0 or now-stamp>self.max_age or stamp-now>.01:raise ValueError('Stale/future IMU timestamp or mismatched clock')
            if self.buffers[i] and stamp<=self.buffers[i][-1].time:raise ValueError('Duplicate/out-of-order IMU sample')
            v=lambda a:np.array([a.x,a.y,a.z],dtype=float)
            self.buffers[i].append(Sample(stamp,v(msg.angular_velocity),v(msg.linear_acceleration),self.covariance(msg.angular_velocity_covariance,self.gvar),self.covariance(msg.linear_acceleration_covariance,self.avar)))
            if not all(self.buffers):return
            # Publish at IMU1 measurement times, not twice per interpolated pair.
            eligible=[s.time for s in self.buffers[0] if self.last_stamp<s.time<=self.buffers[1][-1].time]
            if not eligible:return
            target=eligible[-1]
            if now-target>self.max_age:raise ValueError('Pair is stale')
            samples=[interpolate(b,target,self.max_gap) for b in self.buffers]
            result=self.fusion.update(samples);self.last_stamp=target
            if result is None:self.status(1,'Collecting angular acceleration history');return
            out=Imu();ns=round(target*1e9);out.header.stamp.sec=ns//1_000_000_000;out.header.stamp.nanosec=ns%1_000_000_000;out.header.frame_id=self.frame
            # Independent AHRS yaw references cannot be averaged as if common.
            out.orientation.w=1.;out.orientation_covariance[0]=-1.
            for name,values in [('angular_velocity',result['gyro']),('linear_acceleration',result['accel'])]:
                dst=getattr(out,name);dst.x,dst.y,dst.z=map(float,values)
            out.angular_velocity_covariance=result['gyro_cov'].flatten().tolist();out.linear_acceleration_covariance=result['accel_cov'].flatten().tolist();self.output.publish(out)
            for j,pub in enumerate(self.corrected):
                corrected=Imu();corrected.header=out.header;corrected.orientation.w=1.;corrected.orientation_covariance[0]=-1.;corrected.angular_velocity.x,corrected.angular_velocity.y,corrected.angular_velocity.z=map(float,result['corrected_gyro'][j]);corrected.angular_velocity_covariance=result['rotated_gyro_cov'][j].flatten().tolist()
                corrected.linear_acceleration.x,corrected.linear_acceleration.y,corrected.linear_acceleration.z=map(float,result['corrected'][j]);corrected.linear_acceleration_covariance=result['corrected_accel_cov'][j].flatten().tolist();pub.publish(corrected)
            self.last_received=now
            self.status(0,'Synchronized and lever-arm corrected mean',{'gyro_difference_rad_s':result['gyro_difference'],'accel_difference_m_s2':result['accel_difference'],'alpha_rad_s2':result['alpha'].tolist(),'pair_age_s':now-target,'gravity':'included (specific force)','orientation':'not fused; independent AHRS frames'})
        except ValueError as e:self.status(1,str(e))

    def watchdog(self):
        now=self.get_clock().now().nanoseconds*1e-9
        if self.last_received is None or now-self.last_received>self.max_age:self.status(2,'No fresh valid dual-IMU output; consumer must stop using old data')


def main(args=None):
    rclpy.init(args=args);node=None
    try:node=DualImuNode();rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
