from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch',
            name='navigation',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                '/path/to/your/nav2_params.yaml'
            ],
            remappings=[
                ('/cmd_vel', '/robot/cmd_vel'),
                ('/scan', '/robot/scan'),
                ('/odom', '/robot/odom'),
                ('/imu/data', '/robot/imu/data'),
            ]
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                '/path/to/your/slam_toolbox_params.yaml'
            ]
        )
    ])