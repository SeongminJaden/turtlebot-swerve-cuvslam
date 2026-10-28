"""Replay captured static stereo images to test ROS wiring, NOT live SLAM accuracy.

Run in an isolated ROS_DOMAIN_ID without another /clock or camera publisher.
"""
import argparse
from pathlib import Path
import time
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time

p = argparse.ArgumentParser()
p.add_argument('--images', type=Path, default=Path('checks'))
p.add_argument('--seconds', type=float, default=10)
a = p.parse_args()
rclpy.init()
node = Node('static_stereo_replay')
clock = node.create_publisher(Clock, '/clock', 10)
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)
tf = node.create_publisher(TFMessage, '/tf_static', qos)
transforms = []
streams = []
for side, sign in [('left', 1), ('right', -1)]:
    pixels = PILImage.open(a.images/f'sim61_{side}.png').convert('RGB')
    if pixels.size != (640, 480):
        raise ValueError('Expected 640x480 capture')
    frame = f'sim_{side}_optical'
    image = Image()
    image.header.frame_id = frame
    image.height, image.width = 480, 640
    image.encoding = 'rgb8'
    image.step = 640*3
    image.data = pixels.tobytes()
    info = CameraInfo()
    info.header.frame_id = frame
    info.width, info.height = 640, 480
    info.distortion_model = 'plumb_bob'
    info.d = [0.]*5
    info.k = [400.,0.,320.,0.,400.,240.,0.,0.,1.]
    info.r = [1.,0.,0.,0.,1.,0.,0.,0.,1.]
    info.p = [400.,0.,320.,0. if side=='left' else -20.,0.,400.,240.,0.,0.,0.,1.,0.]
    prefix = f'/front_stereo_camera/{side}'
    streams.append((node.create_publisher(Image,prefix+'/image_rect_color',10),image,
                    node.create_publisher(CameraInfo,prefix+'/camera_info',10),info))
    tr = TransformStamped()
    tr.header.frame_id, tr.child_frame_id = 'base_link', frame
    tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = .25, sign*.025, .25
    tr.transform.rotation.x, tr.transform.rotation.y = -.5, .5
    tr.transform.rotation.z, tr.transform.rotation.w = -.5, .5
    transforms.append(tr)
try:
    tf.publish(TFMessage(transforms=transforms))
    start = time.monotonic()
    count = 0
    while rclpy.ok() and time.monotonic()-start < a.seconds:
        t = Time(sec=1+count//30, nanosec=(count%30)*1_000_000_000//30)
        clock.publish(Clock(clock=t))
        for ip, im, cp, ci in streams:
            im.header.stamp = ci.header.stamp = t
            cp.publish(ci)
            ip.publish(im)
        count += 1
        rclpy.spin_once(node, timeout_sec=0)
        time.sleep(max(0, start + count/30 - time.monotonic()))
    print(f'Published {count} synchronized static stereo pairs; not a motion test')
finally:
    node.destroy_node()
    rclpy.shutdown()
