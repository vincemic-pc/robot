#!/bin/bash
# run_base.sh — Robot base bringup (motors, odometry, EKF, IMU)
source /home/jetson/robot_scripts/ros2_env.sh
exec ros2 launch yahboomcar_bringup yahboomcar_bringup_A1_launch.py
