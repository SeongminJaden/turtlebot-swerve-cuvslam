"""Lockstep stereo-to-cuVSLAM bridge; each reply uses this exact image stamp.

ROS_DOMAIN_ID must isolate the demonstration. This is a simulation clock;
wall time is deliberately slower. Never treats missing tracking as success.
"""
import argparse,json,socket,time,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import Image,CameraInfo
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped,PoseStamped
from nav_msgs.msg import Odometry,Path as RosPath
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from pose_math import matrix,quaternion
from pose_quality import PoseQuality

p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=18767);p.add_argument('--directory',type=Path,required=True);p.add_argument('--pose-quality-gate',action='store_true');a=p.parse_args()
a.directory.mkdir(parents=True,exist_ok=True)
rclpy.init();n=Node('active_camera_lockstep_bridge')
clock=n.create_publisher(Clock,'/clock',10)
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
static=n.create_publisher(TFMessage,'/tf_static',qos);dynamic=n.create_publisher(TFMessage,'/tf',10)
base_pose=n.create_publisher(PoseStamped,'/active_camera/base_pose',10)
pathpub=n.create_publisher(RosPath,'/active_camera/base_path',10)
gtpub=n.create_publisher(PoseStamped,'/active_camera/ground_truth',10)
path=RosPath();path.header.frame_id='odom';status={};odometry={};quality=PoseQuality()
def stamp_ns(x):return x.sec*1000000000+x.nanosec
def on_status(m):status[stamp_ns(m.header.stamp)]=int(m.vo_state)
def on_odom(m):odometry[stamp_ns(m.header.stamp)]=m
n.create_subscription(VisualSlamStatus,'/visual_slam/status',on_status,100)
n.create_subscription(Odometry,'/visual_slam/tracking/odometry',on_odom,100)
transforms=[];streams=[]
for side,y in [('left',.025),('right',-.025)]:
 f='arm_'+side+'_optical';t=TransformStamped();t.header.frame_id='camera_rig';t.child_frame_id=f;t.transform.translation.y=y
 t.transform.rotation.x=-.5;t.transform.rotation.y=.5;t.transform.rotation.z=-.5;t.transform.rotation.w=.5;transforms.append(t)
 info=CameraInfo();info.header.frame_id=f;info.width=640;info.height=480;info.distortion_model='plumb_bob';info.d=[0.]*5
 info.k=[400.,0.,320.,0.,400.,240.,0.,0.,1.];info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
 info.p=[400.,0.,320.,0. if side=='left' else -20.,0.,400.,240.,0.,0.,0.,1.,0.]
 streams.append((side,n.create_publisher(Image,f'/arm_stereo/{side}/image_rect',10),n.create_publisher(CameraInfo,f'/arm_stereo/{side}/camera_info',10),info))
static.publish(TFMessage(transforms=transforms))
def posemsg(mat,stamp):
 m=PoseStamped();m.header.frame_id='odom';m.header.stamp=stamp;m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,mat[:3,3]);q=quaternion(mat[:3,:3]);m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=map(float,q);return m
server=socket.socket();server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);server.bind(('127.0.0.1',a.port));server.listen(1)
print('ROS_BRIDGE_READY',flush=True)
try:
 conn,_=server.accept();reader=conn.makefile('r')
 for line in reader:
  req=json.loads(line);ns=int(req['stamp_ns']);stamp=Time(sec=ns//1000000000,nanosec=ns%1000000000)
  clock.publish(Clock(clock=stamp));static.publish(TFMessage(transforms=transforms))
  # FK is measured at exactly the stereo exposure time, never latest TF.
  world_base=matrix(req['base_pose']);world_rig=matrix(req['rig_pose']);base_rig=np.linalg.inv(world_base)@world_rig
  tr=TransformStamped();tr.header.frame_id='base_link';tr.header.stamp=stamp;tr.child_frame_id='camera_rig';tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=map(float,base_rig[:3,3]);q=quaternion(base_rig[:3,:3]);tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=map(float,q);dynamic.publish(TFMessage(transforms=[tr]))
  pixels=np.load(a.directory/'stereo.npy')
  for i,(side,ip,cp,info) in enumerate(streams if not req.get('drop_images',False) else []):
   im=Image();im.header.frame_id='arm_'+side+'_optical';im.header.stamp=stamp;im.height=480;im.width=640;im.encoding='rgb8';im.step=1920;im.data=pixels[i,:,:,:3].tobytes();info.header.stamp=stamp;cp.publish(info);ip.publish(im)
  deadline=time.monotonic()+(0. if req.get('drop_images',False) else 2.)
  while ns not in status and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.01)
  # Odometry and status can arrive in either order.
  for _ in range(3):rclpy.spin_once(n,timeout_sec=.001)
  state=status.get(ns,0);result={'vo_state':state,'tracking_ok':state==1,'stamp_ns':ns if ns in status else None,'base_estimate':None}
  if ns not in status:print('MISSING_STATUS',ns,'recent',sorted(status)[-3:],flush=True)
  if ns in odometry and state==1:
   op=odometry[ns].pose.pose
   odom_rig=matrix([op.position.x,op.position.y,op.position.z,op.orientation.x,op.orientation.y,op.orientation.z,op.orientation.w])
   est=odom_rig@np.linalg.inv(base_rig)
   result['base_estimate']=[*est[:3,3].tolist(),*quaternion(est[:3,:3]).tolist()]
  if a.pose_quality_gate:
   result['tracking_ok']=bool(result['tracking_ok'] and result['base_estimate'] is not None and quality.update(result['base_estimate'],ns))
   result['pose_quality']=quality.reason
  if result['base_estimate'] is not None and result['tracking_ok']:
   msg=posemsg(matrix(result['base_estimate']),stamp)
   base_pose.publish(msg);path.header.stamp=stamp;path.poses.append(msg);pathpub.publish(path)
   base_tf=TransformStamped();base_tf.header=msg.header;base_tf.child_frame_id='base_link'
   base_tf.transform.translation.x=msg.pose.position.x;base_tf.transform.translation.y=msg.pose.position.y;base_tf.transform.translation.z=msg.pose.position.z;base_tf.transform.rotation=msg.pose.orientation
   dynamic.publish(TFMessage(transforms=[base_tf]))
  gtpub.publish(posemsg(world_base,stamp))
  # Deliberately publishes pose only: rig twist/covariance are not base twist/covariance.
  conn.sendall((json.dumps(result)+'\n').encode())
  status={k:v for k,v in status.items() if k>=ns-1000000000};odometry={k:v for k,v in odometry.items() if k>=ns-1000000000}
finally:server.close();n.destroy_node();rclpy.shutdown()
