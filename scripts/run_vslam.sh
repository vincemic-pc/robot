#!/bin/bash
# run_vslam.sh — Isaac VSLAM (visual SLAM with OAK-D Pro stereo)
source /home/jetson/robot_scripts/ros2_env.sh
exec ros2 launch /home/jetson/robot_scripts/oakd_vslam.launch.py
