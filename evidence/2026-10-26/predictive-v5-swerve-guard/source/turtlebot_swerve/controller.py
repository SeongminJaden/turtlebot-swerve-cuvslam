"""ROS adapter. JointState commands are targets, never measured feedback."""
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from .kinematics import load_modules, solve


class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')
        default = Path(get_package_share_directory('turtlebot_swerve'))/'urdf/m38_arm.urdf'
        self.declare_parameter('urdf_path', str(default))
        self.modules = load_modules(self.get_parameter('urdf_path').value)
        self.twist = (0., 0., 0.)
        self.last_command = -math.inf
        self.feedback = {}
        self.feedback_time = -math.inf
        self.angles = {m.name: 0. for m in self.modules}
        self.publisher = self.create_publisher(JointState, '/swerve/joint_commands', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_subscription(JointState, '/joint_states', self.state, 10)
        self.last_tick = time.monotonic()
        self.create_timer(.02, self.tick)

    def command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        try:
            solve(self.modules, *values)
        except ValueError:
            self.twist = (0., 0., 0.)
            self.last_command = -math.inf
            self.get_logger().warning('Invalid cmd_vel rejected; stopping wheels')
        else:
            self.twist = values
            self.last_command = time.monotonic()

    def state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        names = [m.name + '_steer_joint' for m in self.modules]
        if all(n in positions and math.isfinite(positions[n]) for n in names):
            self.feedback = positions
            self.feedback_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, .1)
        self.last_tick = now
        active = now-self.last_command <= .5 and now-self.feedback_time <= .5
        targets = solve(self.modules, *(self.twist if active else (0., 0., 0.)), self.angles)
        # Wait for all measured steering angles to align before rolling.
        aligned = active and all(abs(targets[m.name].steering - self.feedback[m.name+'_steer_joint']) < .05
                                 for m in self.modules)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for m in self.modules:
            target = targets[m.name]
            delta = target.steering-self.angles[m.name]
            self.angles[m.name] += max(-dt, min(dt, delta))  # URDF limit: 1 rad/s
            msg.name.append(m.name+'_steer_joint')
            msg.position.append(self.angles[m.name])
        # Separate messages avoid mixed position/velocity sentinel values.
        self.publisher.publish(msg)
        wheels = JointState()
        wheels.header = msg.header
        wheels.name = [m.name+'_wheel_joint' for m in self.modules]
        wheels.velocity = [targets[m.name].velocity if aligned else 0. for m in self.modules]
        self.publisher.publish(wheels)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
