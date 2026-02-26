#!/bin/bash
# run_lidar.sh — SLLidar C1 laser scanner
source /home/jetson/robot_scripts/ros2_env.sh
exec ros2 launch sllidar_ros2 sllidar_c1_launch.py
