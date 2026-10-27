"""Replay recorded PhysX samples through actual ROS publishers/fusion."""
import json,os,subprocess,sys,time,tempfile
from pathlib import Path
import rclpy
from sensor_msgs.msg import Imu,JointState
from nav_msgs.msg import Odometry
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
rclpy.init();n=rclpy.create_node('physx_sensor_transport_test');seen={'imu':0,'joints':0,'odom':0}
subs=[]
for key,typ,topic in [('imu',Imu,'/imu/base/data_raw'),('joints',JointState,'/joint_states'),('odom',Odometry,'/wheel/odometry')]:
 def callback(m,k=key):seen[k]+=1
 subs.append(n.create_subscription(typ,topic,callback,10))
children=[]
try:
 with tempfile.TemporaryDirectory() as directory:
  children.append(subprocess.Popen([sys.executable,'scripts/isaac_sensor_bridge.py','--directory',directory],cwd=ROOT))
  children.append(subprocess.Popen([sys.executable,'-c','from turtlebot_swerve.dual_imu_node import main; main()','--ros-args','--params-file','config/dual_imu_sim.yaml'],cwd=ROOT))
  children.append(subprocess.Popen([sys.executable,'scripts/robot_observation_adapter.py','--directory',directory],cwd=ROOT))
  deadline=time.monotonic()+2
  while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
  rows=json.loads((ROOT/'evidence/2026-10-26/physx-sensors/samples.json').read_text())
  for row in rows:
   tmp=Path(directory)/'tmp';tmp.write_text(json.dumps(row));tmp.replace(Path(directory)/'latest-sensors.json')
   until=time.monotonic()+1/15
   while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.002)
  observation=json.loads((Path(directory)/'latest-observations.json').read_text())
  assert observation['tilt_xyzw'] is not None and len(observation['encoders']['position'])==13
  print('ROS_SENSOR_COUNTS',seen,flush=True)
  (ROOT/'evidence/2026-10-26/physx-sensors/ros-report.json').write_text(json.dumps(seen,indent=2))
  assert all(value>100 for value in seen.values()),seen
finally:
 for child in children:
  child.terminate()
 for child in children:
  try:child.wait(timeout=5)
  except subprocess.TimeoutExpired:child.kill();child.wait()
 n.destroy_node();rclpy.shutdown()
