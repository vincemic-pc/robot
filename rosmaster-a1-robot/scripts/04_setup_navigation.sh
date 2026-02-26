#!/bin/bash

# Setup navigation using Nav2 and SLAM

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace
source ~/rosmaster-a1-robot/install/setup.bash

# Install necessary dependencies
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox

# Launch navigation
ros2 launch launch/navigation.launch.py &

# Wait for navigation to initialize
sleep 5

# Start SLAM
ros2 launch launch/slam.launch.py &

# Wait for SLAM to initialize
sleep 5

echo "Navigation and SLAM setup complete."