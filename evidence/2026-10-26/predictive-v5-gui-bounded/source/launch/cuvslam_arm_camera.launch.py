"""Rigid stereo rig on a moving arm. Output pose belongs to camera_rig.

Do not publish odom->camera_rig into the robot TF tree: an adapter must compute
odom->base_link from the same image-time joint pose. IMU fusion is disabled.
"""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    node=ComposableNode(package='isaac_ros_cuvslam',plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',name='arm_visual_slam',parameters=[{
        'use_sim_time':True,'num_cameras':2,'tracking_mode':0,
        'rectified_images':True,'base_frame':'camera_rig',
        'odom_frame':'odom','map_frame':'map',
        'camera_optical_frames':['arm_left_optical','arm_right_optical'],
        'publish_odom_to_base_tf':False,'publish_map_to_odom_tf':False,
        'enable_localization_n_mapping':False,
        'enable_ground_constraint_in_odometry':False,
        'image_jitter_threshold_ms':200.0,
        'image_qos':'DEFAULT', 'image_qos_depth':10,
    }],remappings=[('visual_slam/image_0','/arm_stereo/left/image_rect'),('visual_slam/image_1','/arm_stereo/right/image_rect'),('visual_slam/camera_info_0','/arm_stereo/left/camera_info'),('visual_slam/camera_info_1','/arm_stereo/right/camera_info')])
    return LaunchDescription([ComposableNodeContainer(name='arm_cuvslam_container',namespace='',package='rclcpp_components',executable='component_container',composable_node_descriptions=[node],output='screen')])
