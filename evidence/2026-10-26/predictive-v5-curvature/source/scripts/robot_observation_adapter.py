"""Controller observations from ROS IMU fusion and measured JointState only."""
import argparse,json,sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from sensor_msgs.msg import Imu,JointState
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped
from pose_math import matrix
from turtlebot_swerve.imu_tilt import ImuTilt
p=argparse.ArgumentParser();p.add_argument('--directory',type=Path,required=True);a=p.parse_args()
rclpy.init();node=rclpy.create_node('robot_policy_observations');tilt=ImuTilt();state={};previous=-1
stamp=lambda m:m.header.stamp.sec*10**9+m.header.stamp.nanosec

def on_imu(m):
 ns=stamp(m);w=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z];f=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]
 q=tilt.update(ns/1e9,w,f)
 state['imu_stamp_ns']=ns;state['gyro']=w;state['tilt_xyzw']=None if q is None else q.tolist()
def on_joint(m):
 if len(m.position)!=len(m.name) or len(m.velocity)!=len(m.name):return
 state['joint_stamp_ns']=stamp(m);state['encoders']={'names':list(m.name),'position':list(m.position),'velocity':list(m.velocity)}
def write():
 global previous
 if not all(k in state for k in ('imu_stamp_ns','joint_stamp_ns','tilt_xyzw')):return
 ns=min(state['imu_stamp_ns'],state['joint_stamp_ns'])
 if ns<=previous:return
 previous=ns;out=dict(state,stamp_ns=ns)
 tmp=a.directory/'observations.tmp';tmp.write_text(json.dumps(out));tmp.replace(a.directory/'latest-observations.json')
def on_filtered(m):
 p=m.pose.pose;v=m.twist.twist.linear
 pose=[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
 if not all(math.isfinite(x) for x in pose):return
 out={'stamp_ns':stamp(m),'pose':pose,'body_velocity':[v.x,v.y,v.z],'covariance':list(m.pose.covariance)}
 tmp=a.directory/'filtered.tmp';tmp.write_text(json.dumps(out));tmp.replace(a.directory/'latest-filtered.json')
def on_tf(m):
 for tr in m.transforms:
  if tr.header.frame_id=='map' and tr.child_frame_id=='odom':
   p,q=tr.transform.translation,tr.transform.rotation
   out={'stamp_ns':tr.header.stamp.sec*10**9+tr.header.stamp.nanosec,'matrix':matrix([p.x,p.y,p.z,q.x,q.y,q.z,q.w]).tolist()}
   tmp=a.directory/'map-transform.tmp';tmp.write_text(json.dumps(out));tmp.replace(a.directory/'latest-map-transform.json')
def on_amcl(m):
 out={'stamp_ns':stamp(m),'covariance':list(m.pose.covariance)}
 tmp=a.directory/'amcl.tmp';tmp.write_text(json.dumps(out));tmp.replace(a.directory/'latest-amcl.json')
node.create_subscription(TFMessage,'/tf',on_tf,100)
node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',on_amcl,10)
node.create_subscription(Odometry,'/odometry/filtered',on_filtered,100)
node.create_subscription(Imu,'/imu/base/data_raw',on_imu,100)
node.create_subscription(JointState,'/joint_states',on_joint,100)
node.create_timer(.005,write)
try:rclpy.spin(node)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
