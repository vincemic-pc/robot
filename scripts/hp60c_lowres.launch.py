from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    ascamera_node = Node(
        namespace="ascamera_hp60c",
        package="ascamera",
        executable="ascamera_node",
        respawn=True,
        output="both",
        parameters=[
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": "/home/jetson/yahboomcar_ros2_ws/software/library_ws/src/ascamera/configurationfiles"},
            {"color_pcl": False},
            {"pub_tfTree": True},
            {"depth_width": 320},
            {"depth_height": 240},
            {"rgb_width": 320},
            {"rgb_height": 240},
            {"fps": 10},
        ],
        remappings=[]
    )
    ld.add_action(ascamera_node)
    return ld
