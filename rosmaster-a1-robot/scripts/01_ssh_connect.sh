#!/bin/bash

# SSH connection helper script for Yahboom ROSMASTER A1 robot

ROBOT_IP="192.168.7.250"
SSH_USER="jetson"

# Connect to the robot via SSH
ssh ${SSH_USER}@${ROBOT_IP}