#!/bin/bash
# run_arbiter.sh — Velocity arbiter (sole /cmd_vel publisher, safety filter, PID)
source /home/jetson/robot_scripts/ros2_env.sh

cd /home/jetson/robot_scripts
exec python3 velocity_arbiter.py
