#!/bin/bash
# run_tf.sh — Static TF publishers (base_link→camera_link→oak-d-base-frame)
# Launches both publishers; exits non-zero if either dies (triggers systemd restart).
source /home/jetson/robot_scripts/ros2_env.sh

ros2 run tf2_ros static_transform_publisher \
    0.05 0 0.1 0 0 0 base_link camera_link &
TF1_PID=$!

ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 camera_link oak-d-base-frame &
TF2_PID=$!

# Wait for either to exit — if one dies, kill the other and exit non-zero
wait -n $TF1_PID $TF2_PID 2>/dev/null
kill $TF1_PID $TF2_PID 2>/dev/null
wait 2>/dev/null
exit 1
