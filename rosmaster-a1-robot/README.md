# ROSMASTER A1 Robot Control

## Overview
The ROSMASTER A1 project is designed for the Yahboom ROSMASTER A1 robot, utilizing the Jetson Orin Nano. This project integrates voice control, vision, navigation, and SLAM functionalities to create a comprehensive robotic explorer.

## Features
- **Voice Control**: Utilize voice commands to control the robot's actions.
- **Vision**: Integrate camera capabilities for navigation and obstacle detection.
- **Navigation**: Implement Nav2 for path planning and exploration.
- **SLAM**: Support for both LiDAR and Visual SLAM using the OAK-D Pro camera.

## Project Structure
The project is organized into several directories and files:

- **scripts/**: Contains the main scripts for robot control and setup.
- **config/**: Configuration files for various components.
- **launch/**: Launch files to initialize the robot's systems.
- **msg/**: Message definitions for ROS communication.
- **srv/**: Service definitions for handling requests.
- **test/**: Unit tests for validating functionality.
- **maps/**: Directory for storing SLAM maps.
- **exploration_logs/**: Directory for logging exploration data.
- **.github/**: Contains integration instructions for Copilot.
- **package.xml**: Package manifest for dependencies and metadata.
- **setup.py**: Installation script for the package.
- **setup.cfg**: Configuration settings for package installation.
- **requirements.txt**: Lists Python dependencies required for the project.

## Getting Started
To get started with the ROSMASTER A1 robot, follow these steps:

1. **Connect to the Robot**:
   ```bash
   ssh jetson@192.168.7.250
   ```

2. **Set Up the Environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/.rosmaster_llm_config
   export ROS_DOMAIN_ID=62
   ```

3. **Run the Main Script**:
   ```bash
   ./rosmaster_control.sh run
   ```

4. **Install as an Auto-Start Service**:
   ```bash
   sudo ./rosmaster_control.sh service install
   ```

## Contributing
Contributions to the ROSMASTER A1 project are welcome. Please follow the development workflow outlined in the project documentation for making changes and submitting pull requests.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.