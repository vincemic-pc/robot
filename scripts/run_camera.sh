#!/bin/bash
# run_camera.sh — OAK-D Pro depth camera (depthai-ros)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source /home/jetson/robot_scripts/ros2_env.sh
exec ros2 launch depthai_ros_driver camera.launch.py \
    camera_model:=OAK-D-PRO \
    params_file:=${SCRIPT_DIR}/oakd_params.yaml
