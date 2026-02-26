#!/bin/bash

# Start the necessary hardware components for the robot

# Start LiDAR
roslaunch sllidar_ros2 sllidar.launch

# Start the OAK-D Pro camera
roslaunch depthai_ros oakd.launch

# Start the IMU
roslaunch imu_driver imu.launch

# Start the robot driver
roslaunch robot_driver driver.launch

echo "Robot hardware components started successfully."