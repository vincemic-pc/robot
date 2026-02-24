#!/bin/bash
#===============================================================================
# ROSMASTER A1 - Navigation Setup Script
# Sets up SLAM mapping and autonomous navigation
# Uses Nav2 stack with ROS2 Humble
#===============================================================================

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
MAP_NAME="${MAP_NAME:-rosmaster_map}"
MAP_DIR="${MAP_DIR:-$HOME/maps}"

print_header() {
    echo -e "${BLUE}"
    echo "========================================"
    echo "  ROSMASTER A1 - Navigation Setup"
    echo "  SLAM Mapping & Navigation"
    echo "========================================"
    echo -e "${NC}"
}

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Source ROS2 environment
source_ros2() {
    print_info "Sourcing ROS2 environment..."
    
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    if [ -f ~/rosmaster_a1_ws/install/setup.bash ]; then
        source ~/rosmaster_a1_ws/install/setup.bash
    elif [ -f ~/yahboomcar_ws/install/setup.bash ]; then
        source ~/yahboomcar_ws/install/setup.bash
    fi
    
    # Set ROS domain ID if needed
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
}

# Install navigation dependencies
install_nav_deps() {
    print_info "Installing navigation dependencies..."
    
    sudo apt-get update
    sudo apt-get install -y \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-cartographer \
        ros-humble-cartographer-ros \
        ros-humble-robot-localization \
        ros-humble-tf2-ros \
        ros-humble-tf2-tools \
        ros-humble-tf-transformations \
        ros-humble-joint-state-publisher \
        ros-humble-robot-state-publisher \
        ros-humble-xacro \
        ros-humble-rviz2 \
        ros-humble-gazebo-ros-pkgs
    
    # Install Python dependencies
    pip3 install --user transforms3d
    
    print_info "Navigation dependencies installed!"
}

# Check LiDAR connection
check_lidar() {
    print_info "Checking LiDAR connection..."
    
    # Check for T-Mini Plus LiDAR (common USB serial devices)
    if ls /dev/ttyUSB* 2>/dev/null || ls /dev/ttyACM* 2>/dev/null; then
        print_info "Serial devices found:"
        ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
    else
        print_warning "No serial devices found. LiDAR may not be connected."
    fi
    
    # Check for specific LiDAR devices
    if lsusb | grep -qi "silicon\|ftdi\|cp210\|ch340"; then
        print_info "USB serial converter detected (likely LiDAR)"
    fi
}

# Setup serial port permissions
setup_serial_permissions() {
    print_info "Setting up serial port permissions..."
    
    # Add user to dialout group
    sudo usermod -aG dialout $USER
    
    # Create udev rules for LiDAR
    sudo tee /etc/udev/rules.d/99-lidar.rules > /dev/null << 'EOF'
# T-Mini Plus LiDAR
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="lidar"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", SYMLINK+="lidar"
KERNEL=="ttyACM*", MODE:="0666", SYMLINK+="lidar"

# Generic USB serial
KERNEL=="ttyUSB[0-9]*", MODE="0666"
EOF
    
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    print_info "Serial permissions configured!"
    print_warning "You may need to logout and login for group changes to take effect"
}

# Launch robot base (motors, odometry, tf)
launch_robot_base() {
    print_info "Launching robot base..."
    
    source_ros2
    
    # Launch the robot base with motor control and odometry
    if ros2 pkg list | grep -q "yahboomcar_bringup"; then
        ros2 launch yahboomcar_bringup bringup_launch.py &
    else
        print_warning "ROSMASTER A1 bringup package not found"
        print_info "Make sure the robot workspace is built and sourced"
    fi
    
    sleep 3
}

# Launch LiDAR
launch_lidar() {
    print_info "Launching LiDAR..."
    
    source_ros2
    
    # Try ROSMASTER A1 LiDAR launch
    if ros2 pkg list | grep -q "yahboomcar_bringup"; then
        ros2 launch yahboomcar_bringup lidar_launch.py &
    elif ros2 pkg list | grep -q "ldlidar\|sllidar\|rplidar"; then
        # Try generic LiDAR launch
        ros2 launch ldlidar ldlidar.launch.py 2>/dev/null &
    else
        print_warning "LiDAR package not found"
    fi
    
    sleep 2
    
    # Verify LiDAR topic
    if ros2 topic list | grep -q "/scan"; then
        print_info "LiDAR /scan topic detected!"
    else
        print_warning "LiDAR /scan topic not found"
    fi
}

# Launch SLAM for mapping
launch_slam_mapping() {
    print_info "Launching SLAM mapping mode..."
    
    source_ros2
    mkdir -p ${MAP_DIR}
    
    # Launch robot base first
    launch_robot_base
    launch_lidar
    
    # Launch SLAM Toolbox in mapping mode
    print_info "Starting SLAM Toolbox..."
    ros2 launch slam_toolbox online_async_launch.py \
        use_sim_time:=false \
        slam_params_file:=/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml &
    
    sleep 3
    
    # Launch RViz2 for visualization
    print_info "Launching RViz2..."
    ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz &
    
    print_info "SLAM mapping started!"
    echo ""
    echo "Instructions:"
    echo "  1. Drive the robot around using teleop to create the map"
    echo "  2. In another terminal, run: ./04_setup_navigation.sh teleop"
    echo "  3. When mapping is complete, run: ./04_setup_navigation.sh save-map"
    echo ""
}

# Launch Cartographer for mapping (alternative)
launch_cartographer_mapping() {
    print_info "Launching Cartographer mapping..."
    
    source_ros2
    mkdir -p ${MAP_DIR}
    
    launch_robot_base
    launch_lidar
    
    # Create Cartographer config
    cat > /tmp/cartographer_config.lua << 'EOF'
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

return options
EOF
    
    ros2 launch cartographer_ros cartographer.launch.py \
        use_sim_time:=false \
        configuration_directory:=/tmp \
        configuration_basename:=cartographer_config.lua &
    
    print_info "Cartographer mapping started!"
}

# Save map
save_map() {
    local map_name="${1:-$MAP_NAME}"
    print_info "Saving map as: ${map_name}..."
    
    source_ros2
    mkdir -p ${MAP_DIR}
    
    # Save map using map_saver
    ros2 run nav2_map_server map_saver_cli -f ${MAP_DIR}/${map_name} --ros-args -p save_map_timeout:=10000
    
    if [ -f "${MAP_DIR}/${map_name}.yaml" ]; then
        print_info "Map saved successfully!"
        echo "  YAML: ${MAP_DIR}/${map_name}.yaml"
        echo "  PGM:  ${MAP_DIR}/${map_name}.pgm"
    else
        print_error "Failed to save map"
    fi
}

# Launch navigation with existing map
launch_navigation() {
    local map_file="${1:-${MAP_DIR}/${MAP_NAME}.yaml}"
    
    print_info "Launching navigation with map: ${map_file}..."
    
    if [ ! -f "${map_file}" ]; then
        print_error "Map file not found: ${map_file}"
        print_info "Available maps in ${MAP_DIR}:"
        ls -la ${MAP_DIR}/*.yaml 2>/dev/null || echo "  No maps found"
        return 1
    fi
    
    source_ros2
    
    # Launch robot base
    launch_robot_base
    launch_lidar
    
    # Launch Nav2 navigation stack
    print_info "Launching Nav2 navigation stack..."
    ros2 launch nav2_bringup bringup_launch.py \
        map:=${map_file} \
        use_sim_time:=false \
        autostart:=true \
        params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml &
    
    sleep 5
    
    # Launch RViz2
    print_info "Launching RViz2..."
    ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz &
    
    print_info "Navigation started!"
    echo ""
    echo "Instructions:"
    echo "  1. In RViz2, use '2D Pose Estimate' to set initial pose"
    echo "  2. Use '2D Goal Pose' to send navigation goals"
    echo "  3. Or use voice commands / scripts for navigation"
    echo ""
}

# Launch teleop for manual control
launch_teleop() {
    print_info "Launching teleop keyboard control..."
    
    source_ros2
    
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args --remap cmd_vel:=/cmd_vel
}

# Launch joystick teleop
launch_joystick() {
    print_info "Launching joystick teleop..."
    
    source_ros2
    
    # Install joy package if needed
    sudo apt-get install -y ros-humble-joy ros-humble-teleop-twist-joy 2>/dev/null || true
    
    ros2 launch teleop_twist_joy teleop-launch.py \
        joy_config:=xbox \
        publish_stamped_twist:=false
}

# Create navigation points configuration
create_nav_points() {
    print_info "Creating navigation points configuration..."
    
    mkdir -p ~/rosmaster_a1_config
    
    cat > ~/rosmaster_a1_config/nav_points.yaml << 'EOF'
# ROSMASTER A1 Navigation Points
# Format: point_name: [x, y, theta]
# Coordinates are in meters, theta in radians

navigation_points:
  # Home/Starting position
  home: [0.0, 0.0, 0.0]
  zero: [0.0, 0.0, 0.0]
  
  # Kitchen area
  A: [2.0, 1.0, 1.57]
  kitchen: [2.0, 1.0, 1.57]
  
  # Living room
  B: [3.0, -1.0, 0.0]
  living_room: [3.0, -1.0, 0.0]
  
  # Bedroom
  C: [1.0, 3.0, 3.14]
  bedroom: [1.0, 3.0, 3.14]
  
  # Office
  D: [-1.0, 2.0, -1.57]
  office: [-1.0, 2.0, -1.57]
  
  # Bathroom
  E: [4.0, 2.0, 0.0]
  bathroom: [4.0, 2.0, 0.0]

# Map symbols to locations (for AI commands)
map_mapping:
  A: "kitchen"
  B: "living_room"
  C: "bedroom"
  D: "office"
  E: "bathroom"
  zero: "home"
EOF

    print_info "Navigation points created at ~/rosmaster_a1_config/nav_points.yaml"
    print_warning "Edit this file to match your actual environment!"
}

# Create navigation goal sender script
create_nav_goal_sender() {
    print_info "Creating navigation goal sender..."
    
    mkdir -p ~/rosmaster_a1_scripts
    
    cat > ~/rosmaster_a1_scripts/send_nav_goal.py << 'PYTHON_EOF'
#!/usr/bin/env python3
"""
ROSMASTER A1 Navigation Goal Sender
Sends navigation goals to Nav2 stack
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import yaml
import sys
import math
import os


class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        
        # Action client for Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Load navigation points
        self.nav_points = self.load_nav_points()
        
        self.get_logger().info("Navigation Goal Sender initialized!")
    
    def load_nav_points(self):
        """Load navigation points from YAML file"""
        config_path = os.path.expanduser('~/rosmaster_a1_config/nav_points.yaml')
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                return config.get('navigation_points', {})
        except FileNotFoundError:
            self.get_logger().warning(f"Config not found: {config_path}")
            return {
                'home': [0.0, 0.0, 0.0],
                'A': [1.0, 0.0, 0.0],
                'B': [0.0, 1.0, 1.57],
            }
    
    def send_goal(self, point_name):
        """Send navigation goal to a named point"""
        if point_name not in self.nav_points:
            self.get_logger().error(f"Unknown point: {point_name}")
            self.get_logger().info(f"Available points: {list(self.nav_points.keys())}")
            return False
        
        coords = self.nav_points[point_name]
        x, y, theta = coords[0], coords[1], coords[2]
        
        return self.send_goal_pose(x, y, theta)
    
    def send_goal_pose(self, x, y, theta):
        """Send navigation goal to specific coordinates"""
        self.get_logger().info(f"Sending goal: x={x}, y={y}, theta={theta}")
        
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available!")
            return False
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation complete!')
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Optionally log progress
        pass


def main(args=None):
    rclpy.init(args=args)
    
    nav_sender = NavGoalSender()
    
    if len(sys.argv) > 1:
        point_name = sys.argv[1]
        nav_sender.send_goal(point_name)
        
        # Spin until navigation complete
        rclpy.spin(nav_sender)
    else:
        print("Usage: send_nav_goal.py <point_name>")
        print(f"Available points: {list(nav_sender.nav_points.keys())}")
    
    nav_sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
PYTHON_EOF

    chmod +x ~/rosmaster_a1_scripts/send_nav_goal.py
    print_info "Navigation goal sender created!"
}

# Show usage
show_usage() {
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Setup Options:"
    echo "  install          Install navigation dependencies"
    echo "  check-lidar      Check LiDAR connection"
    echo "  setup-serial     Setup serial port permissions"
    echo "  setup-points     Create navigation points config"
    echo ""
    echo "SLAM Mapping Options:"
    echo "  slam             Launch SLAM mapping mode"
    echo "  cartographer     Launch Cartographer mapping (alternative)"
    echo "  save-map [name]  Save the current map"
    echo ""
    echo "Navigation Options:"
    echo "  nav [map.yaml]   Launch navigation with map"
    echo "  teleop           Launch keyboard teleop"
    echo "  joystick         Launch joystick teleop"
    echo ""
    echo "Environment variables:"
    echo "  MAP_NAME  Default map name (default: rosmaster_map)"
    echo "  MAP_DIR   Map storage directory (default: ~/maps)"
}

# Main execution
print_header

case "${1:-help}" in
    install)
        install_nav_deps
        ;;
    check-lidar)
        check_lidar
        ;;
    setup-serial)
        setup_serial_permissions
        ;;
    setup-points)
        create_nav_points
        create_nav_goal_sender
        ;;
    slam)
        launch_slam_mapping
        ;;
    cartographer)
        launch_cartographer_mapping
        ;;
    save-map)
        save_map "$2"
        ;;
    nav|navigation)
        launch_navigation "$2"
        ;;
    teleop)
        launch_teleop
        ;;
    joystick)
        launch_joystick
        ;;
    help|--help|-h)
        show_usage
        ;;
    *)
        print_error "Unknown option: $1"
        show_usage
        exit 1
        ;;
esac
