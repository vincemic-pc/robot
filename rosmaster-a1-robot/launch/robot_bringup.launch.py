from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosmaster_a1_robot',
            executable='start_robot',
            name='start_robot',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        Node(
            package='rosmaster_a1_robot',
            executable='voice_mapper',
            name='voice_mapper',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        Node(
            package='rosmaster_a1_robot',
            executable='navigation',
            name='navigation',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        Node(
            package='rosmaster_a1_robot',
            executable='slam',
            name='slam',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
    ])