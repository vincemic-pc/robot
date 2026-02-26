#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=62
exec ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-PRO params_file:=/tmp/oakd_test2.yaml > /tmp/cam3.log 2>&1
