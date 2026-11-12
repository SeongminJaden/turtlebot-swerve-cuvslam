from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=os.path.join(get_package_share_directory('turtlebot_swerve'),'config','dual_imu.yaml')),
        Node(package='turtlebot_swerve',executable='dual_imu_fusion',name='dual_imu_fusion',parameters=[LaunchConfiguration('config')],output='screen')])
