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

if [ -f ~/robot_ws/install/setup.bash ]; then
    source ~/robot_ws/install/setup.bash
    # Workaround: some ament_cmake versions omit the ament_prefix_path hook,
    # so ros2 CLI can't find packages in this workspace.  Ensure the prefix
    # is on AMENT_PREFIX_PATH explicitly.
    case ":${AMENT_PREFIX_PATH:-}:" in
        *":$HOME/robot_ws/install/robot_interfaces:"*) ;;
        *) export AMENT_PREFIX_PATH="$HOME/robot_ws/install/robot_interfaces:${AMENT_PREFIX_PATH:-}" ;;
    esac
fi

export ROS_DOMAIN_ID=62
