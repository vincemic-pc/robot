#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=62
exec ros2 launch /home/jetson/robot_scripts/oakd_vslam.launch.py > /tmp/vslam.log 2>&1
