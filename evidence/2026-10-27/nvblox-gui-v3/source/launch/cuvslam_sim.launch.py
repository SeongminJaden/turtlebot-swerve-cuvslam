"""Isaac ROS release-5.0 stereo configuration; runtime validation pending."""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    node=ComposableNode(
        package='isaac_ros_cuvslam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            'use_sim_time':True,
            'enable_imu_fusion':False,
            'rectified_images':True,
            'base_frame':'base_link',
            'odom_frame':'odom',
            'map_frame':'map',
            'camera_optical_frames':['sim_left_optical','sim_right_optical'],
            'enable_slam_visualization':True,
        }],
        remappings=[
            ('visual_slam/image_0','/front_stereo_camera/left/image_rect_color'),
            ('visual_slam/camera_info_0','/front_stereo_camera/left/camera_info'),
            ('visual_slam/image_1','/front_stereo_camera/right/image_rect_color'),
            ('visual_slam/camera_info_1','/front_stereo_camera/right/camera_info'),
        ])
    return LaunchDescription([ComposableNodeContainer(
        name='cuvslam_container',namespace='',package='rclcpp_components',
        executable='component_container',composable_node_descriptions=[node],output='screen')])
