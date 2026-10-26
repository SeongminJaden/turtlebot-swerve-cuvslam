"""Local ROS integration check with synthetic feedback, not a physics test."""
import math
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from turtlebot_swerve.controller import SwerveController

rclpy.init()
controller = SwerveController()
probe = Node('swerve_integration_probe')
executor = SingleThreadedExecutor()
executor.add_node(controller)
executor.add_node(probe)
commands = []
probe.create_subscription(JointState, '/swerve/joint_commands', lambda msg: commands.append(msg), 10)
pub = probe.create_publisher(Twist, '/cmd_vel', 10)
feedback = probe.create_publisher(JointState, '/joint_states', 10)


def run(seconds, supply_command=False, supply_feedback=False):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        if supply_command:
            msg = Twist()
            msg.linear.x = .1
            pub.publish(msg)
        if supply_feedback:
            state = JointState()
            state.name = [m.name+'_steer_joint' for m in controller.modules]
            state.position = [math.pi/2, math.pi/2, 0., 0.]
            feedback.publish(state)
        executor.spin_once(timeout_sec=.02)


try:
    run(.6, True, False)
    assert commands and all(not m.velocity or max(map(abs,m.velocity)) == 0 for m in commands)
    print('PASS missing feedback stops all wheels')
    commands.clear()
    run(2., True, True)
    assert any(m.velocity and max(map(abs,m.velocity)) > 0 for m in commands)
    print('PASS aligned measured feedback enables wheel commands')
    commands.clear()
    run(.8)
    wheels = [m for m in commands if m.velocity]
    assert wheels and max(map(abs,wheels[-1].velocity)) == 0
    print('PASS stale command/feedback stops all wheels')
finally:
    executor.shutdown()
    controller.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()
