#!/bin/bash

# This script enables autonomous driving mode for the robot.

# Start the necessary hardware components
./start_robot.sh

# Wait for the hardware to initialize
sleep 5

# Run the voice mapper in autonomous mode
./rosmaster_control.sh run

# Optionally, you can add commands to monitor the robot's status or handle errors
echo "Autonomous driving mode activated. Monitoring robot status..." 

# Add any additional commands or logic for autonomous driving here
# For example, you could implement a loop to check the robot's status continuously

# End of script