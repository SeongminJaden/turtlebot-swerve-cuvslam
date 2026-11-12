from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    actions=[DeclareLaunchArgument('front_port'),DeclareLaunchArgument('rear_port')]
    for side,num in [('front',1),('rear',2)]:
        actions.append(Node(package='turtlebot_swerve',executable='opencr_imu_driver',namespace=f'opencr_{num}',name='imu_driver',parameters=[{'port':LaunchConfiguration(side+'_port'),'frame_id':f'opencr_{side}_imu','topic':f'/opencr_{num}/imu'}],output='screen'))
    return LaunchDescription(actions)
