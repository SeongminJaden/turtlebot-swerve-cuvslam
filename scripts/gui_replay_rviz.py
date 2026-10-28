"""Publish measured PhysX trajectory for RViz GUI recording; NOT live localization."""
import json, math, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped,TransformStamped
from tf2_msgs.msg import TFMessage
root=Path(__file__).resolve().parents[1]
maze=json.loads((root/'assets/maze/maze.json').read_text())
traj=np.load(root/'evidence/2026-10-26/maze-baseline-hysteresis/poses.npz')['trajectory']
rclpy.init();n=Node('recorded_physx_visualization');pub=n.create_publisher(MarkerArray,'/lecture/maze_and_robot',10);paths=n.create_publisher(RosPath,'/lecture/measured_path',10);tf=n.create_publisher(TFMessage,'/tf',10)
begin=time.monotonic()
try:
 while rclpy.ok() and time.monotonic()-begin<240:
  t=(time.monotonic()-begin)*2%float(traj[-1,0]);idx=int(np.searchsorted(traj[:,0],t));row=traj[min(idx,len(traj)-1)];stamp=n.get_clock().now().to_msg();arr=MarkerArray()
  for i,w in enumerate(maze['panels']):
   m=Marker();m.header.frame_id='map';m.header.stamp=stamp;m.ns='maze';m.id=i;m.type=Marker.CUBE;m.action=Marker.ADD;m.pose.position.x=w['center'][0];m.pose.position.y=w['center'][1];m.pose.position.z=.2;m.pose.orientation.z=math.sin(w['yaw']/2);m.pose.orientation.w=math.cos(w['yaw']/2);m.scale.x=w['length'];m.scale.y=.018;m.scale.z=.38;m.color.r=.3;m.color.g=.65;m.color.b=.85;m.color.a=1.;arr.markers.append(m)
  m=Marker();m.header.frame_id='map';m.header.stamp=stamp;m.ns='measured_robot';m.id=100;m.type=Marker.ARROW;m.pose.position.x=float(row[1]);m.pose.position.y=float(row[2]);m.pose.position.z=.22;m.pose.orientation.x=float(row[4]);m.pose.orientation.y=float(row[5]);m.pose.orientation.z=float(row[6]);m.pose.orientation.w=float(row[7]);m.scale.x=.4;m.scale.y=.16;m.scale.z=.16;m.color.r=1.;m.color.g=.5;m.color.b=.15;m.color.a=1.;arr.markers.append(m)
  label=Marker();label.header.frame_id='map';label.ns='scope';label.id=101;label.type=Marker.TEXT_VIEW_FACING;label.pose.position.x=2.;label.pose.position.y=-.5;label.pose.position.z=.5;label.pose.orientation.w=1.;label.scale.z=.15;label.color.r=label.color.g=label.color.b=label.color.a=1.;label.text=f'Recorded PhysX baseline | t={t:.1f}s | NOT live cuVSLAM';arr.markers.append(label)
  path=RosPath();path.header.frame_id='map';path.header.stamp=stamp
  for rr in traj[:idx:5]:
   p=PoseStamped();p.header=path.header;p.pose.position.x=float(rr[1]);p.pose.position.y=float(rr[2]);p.pose.position.z=.1;p.pose.orientation.w=1.;path.poses.append(p)
  tr=TransformStamped();tr.header.frame_id='map';tr.header.stamp=stamp;tr.child_frame_id='recorded_base';tr.transform.translation.x=float(row[1]);tr.transform.translation.y=float(row[2]);tr.transform.rotation.w=1.;tf.publish(TFMessage(transforms=[tr]));pub.publish(arr);paths.publish(path);rclpy.spin_once(n,timeout_sec=.02);time.sleep(.03)
finally:n.destroy_node();rclpy.shutdown()
