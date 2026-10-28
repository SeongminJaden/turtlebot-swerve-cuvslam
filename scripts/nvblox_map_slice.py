"""Publish only distance slices with an exact-time, planar map transform.

The native costmap plugin can proceed after lookup failure. Reframing here
keeps missing/future TF from placing odom obstacles at the map origin.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from nvblox_msgs.msg import DistanceMapSlice
from pose_math import matrix
from turtlebot_swerve.map_slice import reproject_slice


class MapSliceBridge(Node):
    def __init__(self):
        super().__init__('nvblox_map_slice', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = None
        self.pub = self.create_publisher(DistanceMapSlice, '/navigation/map_slice', 10)
        self.create_subscription(DistanceMapSlice, '/nvblox_node/static_map_slice', self.receive, 10)
        self.create_timer(.02, self.flush)

    def receive(self, msg):
        self.pending = msg

    def flush(self):
        msg = self.pending
        if msg is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
        if age > .5 or age < 0:
            self.pending = None
            return
        try:
            tf = self.buffer.lookup_transform('map', msg.header.frame_id, stamp).transform
        except TransformException:
            return  # Retry after TF arrives; never substitute identity/latest TF.
        try:
            t, q = tf.translation, tf.rotation
            transform = matrix([t.x, t.y, t.z, q.x, q.y, q.z, q.w])
            data, origin = reproject_slice(np.asarray(msg.data).reshape(msg.height, msg.width),
                                          [msg.origin.x, msg.origin.y], msg.resolution,
                                          transform, msg.unknown_value)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            self.pending = None
            return
        msg.header.frame_id = 'map'
        msg.height, msg.width = data.shape
        msg.origin.x, msg.origin.y = map(float, origin)
        msg.data = data.astype(np.float32).ravel().tolist()
        self.pub.publish(msg)
        self.pending = None


def main():
    rclpy.init()
    node = MapSliceBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
