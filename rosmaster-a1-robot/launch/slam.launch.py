from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='slam_toolbox',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': False}, '/path/to/slam_toolbox_params.yaml']
        ),
        Node(
            package='isaac_ros_visual_slam',
            executable='vslam_node',
            name='vslam_node',
            output='screen',
            parameters=[{'use_sim_time': False}, '/path/to/isaac_vslam_params.yaml']
        ),
        Node(
            package='depthai_ros',
            executable='depthai_node',
            name='depthai_node',
            output='screen',
            parameters=[{'use_sim_time': False}, '/path/to/oakd_pro_camera.yaml']
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{'use_sim_time': False}, '/path/to/sllidar_c1.yaml']
        ),
    ])