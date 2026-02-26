#!/bin/bash

# Setup OAK-D Pro camera for depth sensing

# Install depthai-ros package if not already installed
if ! dpkg -l | grep -q ros-humble-depthai-ros; then
    echo "Installing depthai-ros package..."
    sudo apt install -y ros-humble-depthai-ros
fi

# Load camera parameters from configuration file
CAMERA_CONFIG=~/config/oakd_pro_camera.yaml

if [ ! -f "$CAMERA_CONFIG" ]; then
    echo "Camera configuration file not found: $CAMERA_CONFIG"
    exit 1
fi

# Launch the camera node with the specified configuration
ros2 launch depthai_ros depthai_ros.launch.py config_file:=$CAMERA_CONFIG

echo "OAK-D Pro camera setup complete."