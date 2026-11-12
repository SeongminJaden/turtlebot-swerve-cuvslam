"""Depth/FK mapping and Nav2 planner; PPO remains the only drive controller."""
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    return LaunchDescription([
        Node(package='nvblox_ros',executable='nvblox_node',name='nvblox_node',
             parameters=[str(root/'config/nvblox_arm.yaml')],
             remappings=[('/camera_0/depth/image','/arm_stereo/left/depth'),
                         ('/camera_0/depth/camera_info','/arm_stereo/left/camera_info')],output='screen'),
        Node(package='nav2_planner',executable='planner_server',name='planner_server',
             parameters=[str(root/'config/nav2_nvblox.yaml')],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
             name='lifecycle_manager_planning',parameters=[{'use_sim_time':True,
             'autostart':True,'node_names':['planner_server']}],output='screen')])
