"""Actual ROS 2 + PTY USB transport integration; no physical sensor claim."""
import os,pty,threading,struct,zlib,time,json,sys
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from turtlebot_swerve.opencr_imu_driver import OpenCrImuDriver
from turtlebot_swerve.dual_imu_node import DualImuNode
from turtlebot_swerve.dual_imu import rotation
rs=np.array([[.057,0,.166],[-.057,0,.166]]);qs=[[0.,0.,0.,1.],[0.,0.,1.,0.]];omega=np.array([0.,1.,0.]);force=np.array([.2,-.1,9.80665]);stop=threading.Event();fault=threading.Event();threads=[];masters=[];ports=[]
def emulate(fd,i):
 R=rotation(qs[i]);g=R.T@omega;f=R.T@(force+np.cross(omega,np.cross(omega,rs[i])));seq=0
 while not stop.is_set():
  try:cmd=os.read(fd,1)
  except OSError:time.sleep(.001);continue
  if cmd==b'R':
   seq+=1;us=(time.monotonic_ns()//1000)&0xffffffff
   raw=struct.pack('<4sIII10fI',b'IMU1',seq,us,us,*g,*f,1.,0.,0.,0.,1)
   crc=zlib.crc32(raw)^(1 if fault.is_set() and i==1 else 0)
   os.write(fd,raw+struct.pack('<I',crc))
for i in range(2):
 m,s=pty.openpty();masters.append(m);ports.append(os.ttyname(s));t=threading.Thread(target=emulate,args=(m,i),daemon=True);t.start();threads.append(t)
rclpy.init();params=lambda d:[Parameter(k,value=v) for k,v in d.items()]
drivers=[OpenCrImuDriver(namespace=f'opencr_{i+1}',parameter_overrides=params({'port':ports[i],'frame_id':f'opencr_{["front","rear"][i]}_imu','topic':f'/opencr_{i+1}/imu','poll_hz':200.,'max_rtt_s':.02})) for i in range(2)]
fusion=DualImuNode(parameter_overrides=params({'geometry_verified':True,'imu1.position_m':rs[0].tolist(),'imu2.position_m':rs[1].tolist(),'imu1.quaternion_xyzw':qs[0],'imu2.quaternion_xyzw':qs[1],'alpha_filter_tau_s':0.}))
monitor=Node('dual_imu_test_monitor');rows=[];diagnostics=[]
monitor.create_subscription(Imu,'/imu/base/data_raw',lambda m:rows.append({'force':[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],'gyro':[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],'frame':m.header.frame_id,'orientation_cov0':m.orientation_covariance[0]}),10)
monitor.create_subscription(DiagnosticArray,'/imu/diagnostics',lambda m:diagnostics.extend([{'level':int.from_bytes(s.level,'little') if isinstance(s.level,bytes) else s.level,'message':s.message} for s in m.status]),10)
executor=SingleThreadedExecutor()
for n in [*drivers,fusion,monitor]:executor.add_node(n)
try:
 until=time.monotonic()+2.
 while time.monotonic()<until:executor.spin_once(timeout_sec=.01)
 assert len(rows)>=30,len(rows)
 errors=np.linalg.norm(np.asarray([r['force'] for r in rows])-force,axis=1)
 assert max(errors)<1e-4,max(errors)
 assert all(r['frame']=='base_link' and r['orientation_cov0']==-1 for r in rows)
 fault.set();until=time.monotonic()+.3
 while time.monotonic()<until:executor.spin_once(timeout_sec=.01)
 before=len(rows);until=time.monotonic()+.3
 while time.monotonic()<until:executor.spin_once(timeout_sec=.01)
 assert len(rows)==before,'Fusion reused stale second IMU'
 assert any(x['level']==2 for x in diagnostics),'No stale data diagnostic'
 report={'scope':'ROS2 + two PTY serial OpenCR emulators; not physical OpenCR or Isaac sensor','poll_hz':200,'valid_messages':before,'max_force_error_m_s2':float(max(errors)),'expected_force':force.tolist(),'uncorrected_force_z_m_s2':float(force[2]-.166),'corrupt_second_imu_stops_output':True,'stale_diagnostic':True}
 out=Path('evidence/2026-10-26/dual-imu-audit/ros-transport-200hz-test.json');out.write_text(json.dumps(report,indent=2));print(json.dumps(report))
finally:
 stop.set()
 for n in [*drivers,fusion,monitor]:n.destroy_node()
 executor.shutdown();rclpy.shutdown()
