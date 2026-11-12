"""Known CAD map + stereo-derived planar scan; swerve uses AMCL Omni model."""
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
 root=Path(__file__).resolve().parents[1]
 return LaunchDescription([
  Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[{'use_sim_time':True,'yaml_filename':str(root/'assets/maze/localization/maze.yaml')}],output='screen'),
  Node(package='nav2_amcl',executable='amcl',name='amcl',parameters=[str(root/'config/amcl_swerve.yaml')],output='screen'),
  Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_localization',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['map_server','amcl']}],output='screen')])
