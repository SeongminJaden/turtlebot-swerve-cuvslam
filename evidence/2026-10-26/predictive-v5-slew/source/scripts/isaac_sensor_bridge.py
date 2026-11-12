"""Publish physics-stamped sensors without waiting for the camera renderer.
One latest-state slot intentionally drops superseded samples, never invents them.
"""
import argparse,json,time,sys,socket,os
import numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from turtlebot_swerve.encoder_odometry import body_twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu,JointState
from std_msgs.msg import Int64MultiArray
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
p=argparse.ArgumentParser();p.add_argument('--directory',type=Path,required=True);p.add_argument('--udp-port',type=int);p.add_argument('--fault-imu2-start-s',type=float,default=float(os.getenv('FAULT_IMU2_START','-1')));p.add_argument('--fault-imu2-duration-s',type=float,default=float(os.getenv('FAULT_IMU2_DURATION','1')));a=p.parse_args()
rclpy.init();node=Node('isaac_robot_sensor_bridge')
imu=[node.create_publisher(Imu,f'/opencr_{i+1}/imu',10) for i in range(2)]
joints=node.create_publisher(JointState,'/joint_states',10)
counts=node.create_publisher(Int64MultiArray,'/encoders/motor_counts',10)
clock=node.create_publisher(Clock,'/clock',10)
wheel_odom=node.create_publisher(Odometry,'/wheel/odometry',10)
sock=None
if a.udp_port:
 sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',a.udp_port));sock.settimeout(.02)
previous=-1;xy=np.zeros(2);yaw=0.
try:
 while rclpy.ok():
  try:
   data=json.loads(sock.recv(65535)) if sock else json.loads((a.directory/'latest-sensors.json').read_text())
  except (FileNotFoundError,json.JSONDecodeError,socket.timeout):time.sleep(.002);continue
  ns=data['stamp_ns']
  if ns<=previous:time.sleep(.002);continue
  dt=(ns-previous)/1e9 if previous>=0 else 0.;previous=ns;stamp=Time(sec=ns//10**9,nanosec=ns%10**9);clock.publish(Clock(clock=stamp))
  for sensor_index,(pub,v) in enumerate(zip(imu,data['imu'])):
   sim_s=(ns-1_000_000_000)/1e9
   if sensor_index==1 and a.fault_imu2_start_s>=0 and a.fault_imu2_start_s<=sim_s<a.fault_imu2_start_s+a.fault_imu2_duration_s:continue
   if v['saturated']:continue
   m=Imu();m.header.stamp=stamp;m.header.frame_id=v['frame_id'];m.orientation.w=1.;m.orientation_covariance[0]=-1.
   m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=v['gyro']
   m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=v['accel'];pub.publish(m)
  e=data['encoders'];m=JointState();m.header.stamp=stamp;m.header.frame_id='base_link';m.name=e['names'];m.position=e['position']
  if e['velocity_valid']:m.velocity=e['velocity']
  if e['velocity_valid'] and 0<dt<.2:
   twist,residual=body_twist(e['names'],e['position'],e['velocity'])
   midpoint=yaw+.5*twist[2]*dt;c,s=np.cos(midpoint),np.sin(midpoint)
   xy+=np.array([[c,-s],[s,c]])@twist[:2]*dt;yaw+=twist[2]*dt
   od=Odometry();od.header.stamp=stamp;od.header.frame_id='wheel_odom';od.child_frame_id='base_link'
   od.pose.pose.position.x,od.pose.pose.position.y=xy.tolist();od.pose.pose.orientation.z=float(np.sin(yaw/2));od.pose.pose.orientation.w=float(np.cos(yaw/2))
   od.twist.twist.linear.x,od.twist.twist.linear.y,od.twist.twist.angular.z=twist.tolist()
   # Conservative simulation placeholder, not a calibrated slip uncertainty model.
   for k in (0,7,35):od.twist.covariance[k]=.0004+residual**2
   for k in (14,21,28):od.twist.covariance[k]=1e6
   for k in (0,7,14,21,28,35):od.pose.covariance[k]=1e6
   wheel_odom.publish(od)
  joints.publish(m);counts.publish(Int64MultiArray(data=e['motor_counts']))
  rclpy.spin_once(node,timeout_sec=0)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:
 node.destroy_node()
 if sock:sock.close()
 if rclpy.ok():rclpy.shutdown()
