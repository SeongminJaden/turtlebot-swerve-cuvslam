"""Periodic Nav2 action requests; atomic planner-to-policy transport with freshness."""
import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as RosPath, OccupancyGrid
from nvblox_msgs.msg import DistanceMapSlice
from turtlebot_swerve.live_path import resample_path
p=argparse.ArgumentParser();p.add_argument('--directory',type=Path,required=True);p.add_argument('--route',type=Path,default=Path('assets/maze/route_clearance.json'));a=p.parse_args()
goal=json.loads(a.route.read_text())['route'][-1]
rclpy.init();n=rclpy.create_node('nav2_policy_path',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');pub=n.create_publisher(RosPath,'/policy/planned_path',10)
slice_saved=-1
slice_ns=-1;busy=False;revision=0;requests=0;successes=0;last_status='waiting';started=0.
def write(valid,points=None,status=''):
    data={'valid':valid,'frame_id':'map','stamp_ns':n.get_clock().now().nanoseconds,'slice_stamp_ns':slice_ns,'revision':revision,'points':points or [],'status':status,'requests':requests,'successes':successes}
    f=a.directory/'nav2-path.tmp';f.write_text(json.dumps(data));f.replace(a.directory/'latest-nav2-path.json')
    with (a.directory/'nav2-path-events.jsonl').open('a') as f:f.write(json.dumps(data)+'\n')
def on_slice(m):
    global slice_ns,slice_saved
    if m.width and m.height and any(v!=m.unknown_value for v in m.data):slice_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
    if slice_ns-slice_saved>=1_000_000_000:
        slice_saved=slice_ns
        data={'stamp_ns':slice_ns,'frame_id':m.header.frame_id,'resolution':m.resolution,'width':m.width,'height':m.height,'origin':[m.origin.x,m.origin.y,m.origin.z],'unknown_value':m.unknown_value,'data':list(m.data)}
        f=a.directory/'nvblox-slice.tmp';f.write_text(json.dumps(data));f.replace(a.directory/'latest-nvblox-slice.json')
def on_costmap(m):
    data={'stamp_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec,'frame_id':m.header.frame_id,'resolution':m.info.resolution,'width':m.info.width,'height':m.info.height,'origin':[m.info.origin.position.x,m.info.origin.position.y],'data':list(m.data)}
    f=a.directory/'costmap.tmp';f.write_text(json.dumps(data));f.replace(a.directory/'latest-costmap.json')
n.create_subscription(OccupancyGrid,'/global_costmap/costmap',on_costmap,10)
n.create_subscription(DistanceMapSlice,'/navigation/map_slice',on_slice,10)
def finished(f):
    global busy,revision,successes
    busy=False
    try:
        result=f.result();r=result.result
        if result.status!=4:write(False,status=f'planner_status_{result.status}');return
        pts=resample_path([[q.pose.position.x,q.pose.position.y] for q in r.path.poses]);revision+=1;successes+=1
        pub.publish(r.path);write(True,pts.tolist(),'planned')
    except Exception as e:write(False,status=str(e))
def accepted(f):
    global busy
    try:
        h=f.result()
        if not h.accepted:busy=False;write(False,status='rejected');return
        h.get_result_async().add_done_callback(finished)
    except Exception as e:busy=False;write(False,status=str(e))
def tick():
    global busy,requests,started
    now=n.get_clock().now().nanoseconds
    if busy:return
    if not 0<=(now-slice_ns)/1e9<=1.0:write(False,status='waiting_for_fresh_nvblox');return
    if not client.server_is_ready():return
    g=ComputePathToPose.Goal();g.goal.header.frame_id='map';g.goal.header.stamp=n.get_clock().now().to_msg();g.goal.pose.position.x=float(goal[0]);g.goal.pose.position.y=float(goal[1]);g.goal.pose.orientation.w=1.;g.planner_id='GridBased';g.use_start=False
    busy=True;requests+=1;started=time.monotonic();client.send_goal_async(g).add_done_callback(accepted)
n.create_timer(1.,tick)
try:rclpy.spin(n)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:
    n.destroy_node()
    if rclpy.ok():rclpy.shutdown()
