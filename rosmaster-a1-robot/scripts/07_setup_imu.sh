#!/bin/bash

# Setup IMU configuration for the robot

# Load the necessary environment variables
source /opt/ros/humble/setup.bash

# Start the IMU driver
ros2 run imu_driver imu_node &

# Wait for the IMU node to start
sleep 5

# Check if the IMU is publishing data
if ros2 topic list | grep -q "/imu/data"; then
    echo "IMU is successfully set up and publishing data."
else
    echo "Failed to set up IMU. Please check the connections and configurations."
fi