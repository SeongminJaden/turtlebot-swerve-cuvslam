"""Synthetic ROS integration check for the official robot_localization node."""
import json,math,os,subprocess,time
from pathlib import Path
import rclpy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
root=Path(__file__).resolve().parents[1];binary=root/'build/robot-localization-runtime/prefix/opt/ros/lyrical/lib/robot_localization/ekf_node'
rclpy.init();n=rclpy.create_node('ekf_integration_test');output=[]
clock=n.create_publisher(Clock,'/clock',10);wheel=n.create_publisher(Odometry,'/wheel/odometry',10);imu=n.create_publisher(Imu,'/imu/base/data_raw',10);vo=n.create_publisher(PoseWithCovarianceStamped,'/active_camera/base_pose_covariance',10)
n.create_subscription(Odometry,'/odometry/filtered',lambda m:output.append(m),100)
log=(root/'runs/ekf-integration-node.log').open('w');process=subprocess.Popen([str(binary),'--ros-args','-r','__node:=ekf_filter_node','--params-file',str(root/'config/ekf_swerve.yaml')],stdout=log,stderr=subprocess.STDOUT)
try:
 until=time.monotonic()+1.5
 while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.01)
 for i in range(201):
  t=i*.02;ns=1_000_000_000+round(t*1e9);stamp=Time(sec=ns//10**9,nanosec=ns%10**9);clock.publish(Clock(clock=stamp))
  m=Odometry();m.header.stamp=stamp;m.header.frame_id='wheel_odom';m.child_frame_id='base_link';m.twist.twist.linear.x=.2
  for k in (0,7,35):m.twist.covariance[k]=.001
  wheel.publish(m)
  m=Imu();m.header.stamp=stamp;m.header.frame_id='base_link';m.orientation_covariance[0]=-1.;m.angular_velocity.z=.1
  for k in (0,4,8):m.angular_velocity_covariance[k]=.0001
  imu.publish(m)
  if i%5==0:
   m=PoseWithCovarianceStamped();m.header.stamp=stamp;m.header.frame_id='odom';m.pose.pose.position.x=2*math.sin(.1*t);m.pose.pose.position.y=2*(1-math.cos(.1*t));m.pose.pose.orientation.z=math.sin(.05*t);m.pose.pose.orientation.w=math.cos(.05*t)
   for k in (0,7,35):m.pose.covariance[k]=.0004
   vo.publish(m)
  until=time.monotonic()+.01
  while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.001)
 assert len(output)>50,len(output)
 last=output[-1];t=last.header.stamp.sec+last.header.stamp.nanosec/1e9-1
 error=math.hypot(last.pose.pose.position.x-2*math.sin(.1*t),last.pose.pose.position.y-2*(1-math.cos(.1*t)))
 result={'received':len(output),'position_error_m':error,'scope':'synthetic curved motion ROS integration; not real sensor calibration'}
 (root/'evidence/2026-10-26/physx-sensors/ekf-transport.json').write_text(json.dumps(result,indent=2));print(result);assert error<.05
finally:
 process.terminate();process.wait(timeout=5);log.close();n.destroy_node();rclpy.shutdown()
