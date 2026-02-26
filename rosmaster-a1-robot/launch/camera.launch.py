from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthai_ros',
            executable='depthai_node',
            name='oakd_pro_camera',
            output='screen',
            parameters=[{'config_file': '/path/to/config/oakd_pro_camera.yaml'}],
            remappings=[
                ('/oak/rgb/image_raw', '/oak/rgb/image_raw'),
                ('/oak/stereo/image_raw', '/oak/stereo/image_raw'),
            ]
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_c1',
            output='screen',
            parameters=[{'config_file': '/path/to/config/sllidar_c1.yaml'}]
        )
    ])