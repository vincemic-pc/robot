#!/bin/bash
# ros2_env.sh — Shared ROS2 environment for all robot service wrappers.
# Sourced (not executed) by run_*.sh scripts.

source /opt/ros/humble/setup.bash

if [ -f ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash ]; then
    source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
fi

if [ -f ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash ]; then
    source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
fi

export ROS_DOMAIN_ID=62
