#!/bin/bash
#===============================================================================
# ROSMASTER A1 - Autonomous Driving Setup Script
# Sets up line following, obstacle avoidance, and autonomous navigation
#===============================================================================

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo -e "${BLUE}"
    echo "========================================"
    echo "  ROSMASTER A1 - Autonomous Driving"
    echo "  Line Following & Auto Navigation"
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
}

# Install autonomous driving dependencies
install_auto_deps() {
    print_info "Installing autonomous driving dependencies..."
    
    sudo apt-get update
    sudo apt-get install -y \
        ros-humble-cv-bridge \
        ros-humble-vision-opencv \
        ros-humble-image-transport \
        ros-humble-image-geometry \
        python3-opencv \
        python3-numpy
    
    pip3 install --user \
        opencv-python \
        numpy \
        scipy
    
    print_info "Dependencies installed!"
}

# Create line following node
create_line_follower() {
    print_info "Creating line following node..."
    
    mkdir -p ~/rosmaster_a1_scripts
    
    cat > ~/rosmaster_a1_scripts/line_follower.py << 'PYTHON_EOF'
#!/usr/bin/env python3
"""
ROSMASTER A1 Line Following Node
Follows colored lines on the ground using camera vision
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        # Parameters
        self.declare_parameter('line_color', 'green')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_gain', 0.005)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        
        self.line_color = self.get_parameter('line_color').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_gain = self.get_parameter('angular_gain').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/line_follower/debug', 10)
        self.status_pub = self.create_publisher(String, '/line_follower/status', 10)
        
        # Subscribers
        image_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        
        self.enable_sub = self.create_subscription(
            Bool, '/line_follower/enable', self.enable_callback, 10
        )
        
        self.color_sub = self.create_subscription(
            String, '/line_follower/color', self.color_callback, 10
        )
        
        # State
        self.enabled = False
        self.last_error = 0
        
        # Color ranges (HSV)
        self.color_ranges = {
            'red': {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255])},
            'red2': {'lower': np.array([160, 100, 100]), 'upper': np.array([180, 255, 255])},
            'green': {'lower': np.array([35, 100, 100]), 'upper': np.array([85, 255, 255])},
            'blue': {'lower': np.array([100, 100, 100]), 'upper': np.array([130, 255, 255])},
            'yellow': {'lower': np.array([20, 100, 100]), 'upper': np.array([40, 255, 255])},
        }
        
        self.get_logger().info(f"Line Follower initialized for {self.line_color} line")
    
    def enable_callback(self, msg):
        self.enabled = msg.data
        status = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f"Line following {status}")
        
        if not self.enabled:
            self.stop_robot()
    
    def color_callback(self, msg):
        if msg.data in self.color_ranges or msg.data == 'red':
            self.line_color = msg.data
            self.get_logger().info(f"Line color changed to: {self.line_color}")
    
    def image_callback(self, msg):
        if not self.enabled:
            return
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process image
            error, debug_image = self.process_image(cv_image)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_pub.publish(debug_msg)
            
            # Control robot
            if error is not None:
                self.follow_line(error)
            else:
                # No line detected
                self.search_for_line()
                
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
    
    def process_image(self, image):
        """Process image to detect line and calculate error"""
        height, width = image.shape[:2]
        
        # Use lower portion of image
        roi = image[int(height*0.6):, :]
        roi_height, roi_width = roi.shape[:2]
        
        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Create mask for line color
        mask = self.create_color_mask(hsv)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create debug image
        debug_image = roi.copy()
        cv2.drawContours(debug_image, contours, -1, (0, 255, 0), 2)
        
        if contours:
            # Find largest contour (assumed to be the line)
            largest_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) > 500:  # Minimum area threshold
                # Calculate centroid
                M = cv2.moments(largest_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # Draw centroid
                    cv2.circle(debug_image, (cx, cy), 10, (255, 0, 0), -1)
                    
                    # Calculate error (deviation from center)
                    center_x = roi_width // 2
                    error = cx - center_x
                    
                    # Draw center line and error
                    cv2.line(debug_image, (center_x, 0), (center_x, roi_height), (0, 0, 255), 2)
                    cv2.putText(debug_image, f"Error: {error}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    
                    status_msg = String()
                    status_msg.data = f"tracking:{self.line_color}:error={error}"
                    self.status_pub.publish(status_msg)
                    
                    return error, debug_image
        
        # No line found
        status_msg = String()
        status_msg.data = "searching"
        self.status_pub.publish(status_msg)
        
        return None, debug_image
    
    def create_color_mask(self, hsv):
        """Create color mask based on selected line color"""
        if self.line_color == 'red':
            # Red wraps around in HSV
            mask1 = cv2.inRange(hsv, self.color_ranges['red']['lower'], 
                               self.color_ranges['red']['upper'])
            mask2 = cv2.inRange(hsv, self.color_ranges['red2']['lower'], 
                               self.color_ranges['red2']['upper'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            color_range = self.color_ranges.get(self.line_color, self.color_ranges['green'])
            mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
        
        # Clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        return mask
    
    def follow_line(self, error):
        """Generate velocity command to follow line"""
        twist = Twist()
        
        # P controller for angular velocity
        angular_z = -error * self.angular_gain
        
        # Limit angular velocity
        angular_z = max(-0.5, min(0.5, angular_z))
        
        # Set velocities
        twist.linear.x = self.linear_speed
        twist.angular.z = angular_z
        
        self.cmd_vel_pub.publish(twist)
        self.last_error = error
    
    def search_for_line(self):
        """Search behavior when line is lost"""
        twist = Twist()
        
        # Turn in direction of last error
        if self.last_error > 0:
            twist.angular.z = -0.3
        else:
            twist.angular.z = 0.3
        
        twist.linear.x = 0.05  # Slow forward
        
        self.cmd_vel_pub.publish(twist)
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    
    follower = LineFollower()
    
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        follower.stop_robot()
    finally:
        follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYTHON_EOF

    chmod +x ~/rosmaster_a1_scripts/line_follower.py
    print_info "Line follower created!"
}

# Create obstacle avoidance node
create_obstacle_avoider() {
    print_info "Creating obstacle avoidance node..."
    
    mkdir -p ~/rosmaster_a1_scripts
    
    cat > ~/rosmaster_a1_scripts/obstacle_avoider.py << 'PYTHON_EOF'
#!/usr/bin/env python3
"""
ROSMASTER A1 Obstacle Avoidance Node
Uses LiDAR and depth camera for obstacle detection and avoidance
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import numpy as np
import math


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Parameters
        self.declare_parameter('min_distance', 0.5)  # meters
        self.declare_parameter('warning_distance', 1.0)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        
        self.min_distance = self.get_parameter('min_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/obstacle_avoider/status', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )
        
        self.enable_sub = self.create_subscription(
            Bool, '/obstacle_avoider/enable', self.enable_callback, 10
        )
        
        # CV Bridge for depth images
        self.bridge = CvBridge()
        
        # State
        self.enabled = False
        self.obstacle_detected = False
        self.closest_distance = float('inf')
        self.obstacle_angle = 0.0
        
        self.get_logger().info("Obstacle Avoider initialized!")
    
    def enable_callback(self, msg):
        self.enabled = msg.data
        status = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f"Obstacle avoidance {status}")
        
        if not self.enabled:
            self.stop_robot()
    
    def scan_callback(self, msg):
        if not self.enabled:
            return
        
        # Analyze laser scan
        ranges = np.array(msg.ranges)
        
        # Replace inf values
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[ranges == 0] = msg.range_max
        
        # Find closest obstacle
        min_idx = np.argmin(ranges)
        self.closest_distance = ranges[min_idx]
        
        # Calculate angle of closest obstacle
        self.obstacle_angle = msg.angle_min + min_idx * msg.angle_increment
        
        # Check different sectors
        num_readings = len(ranges)
        sector_size = num_readings // 5
        
        # Front sector (middle)
        front_start = num_readings // 2 - sector_size // 2
        front_end = num_readings // 2 + sector_size // 2
        front_min = np.min(ranges[front_start:front_end])
        
        # Left sector
        left_start = 0
        left_end = sector_size
        left_min = np.min(ranges[left_start:left_end])
        
        # Right sector
        right_start = num_readings - sector_size
        right_end = num_readings
        right_min = np.min(ranges[right_start:right_end])
        
        # Decision making
        if front_min < self.min_distance:
            self.obstacle_detected = True
            self.avoid_obstacle(front_min, left_min, right_min)
        elif front_min < self.warning_distance:
            self.obstacle_detected = True
            self.slow_down(front_min)
        else:
            self.obstacle_detected = False
            self.move_forward()
        
        # Publish status
        status_msg = String()
        status_msg.data = f"distance:{self.closest_distance:.2f}:obstacle:{self.obstacle_detected}"
        self.status_pub.publish(status_msg)
    
    def depth_callback(self, msg):
        """Process depth image for additional obstacle detection"""
        if not self.enabled:
            return
        
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Get center region of depth image
            height, width = depth_image.shape
            center_region = depth_image[height//3:2*height//3, width//3:2*width//3]
            
            # Calculate mean depth (convert from mm to m if needed)
            valid_depths = center_region[center_region > 0]
            if len(valid_depths) > 0:
                mean_depth = np.mean(valid_depths) / 1000.0  # Convert to meters
                
                # Additional obstacle check from depth camera
                if mean_depth < self.min_distance:
                    self.get_logger().debug(f"Depth camera obstacle at {mean_depth:.2f}m")
                    
        except Exception as e:
            self.get_logger().error(f"Depth processing error: {e}")
    
    def avoid_obstacle(self, front_dist, left_dist, right_dist):
        """Avoid obstacle by turning"""
        twist = Twist()
        
        # Stop forward motion
        twist.linear.x = 0.0
        
        # Turn away from obstacle
        if left_dist > right_dist:
            # More space on left, turn left
            twist.angular.z = self.angular_speed
            self.get_logger().debug("Turning left to avoid obstacle")
        else:
            # More space on right, turn right
            twist.angular.z = -self.angular_speed
            self.get_logger().debug("Turning right to avoid obstacle")
        
        self.cmd_vel_pub.publish(twist)
    
    def slow_down(self, distance):
        """Slow down when approaching obstacle"""
        twist = Twist()
        
        # Reduce speed based on distance
        speed_factor = (distance - self.min_distance) / (self.warning_distance - self.min_distance)
        twist.linear.x = self.linear_speed * speed_factor
        
        self.cmd_vel_pub.publish(twist)
    
    def move_forward(self):
        """Move forward when path is clear"""
        twist = Twist()
        twist.linear.x = self.linear_speed
        self.cmd_vel_pub.publish(twist)
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    
    avoider = ObstacleAvoider()
    
    try:
        rclpy.spin(avoider)
    except KeyboardInterrupt:
        avoider.stop_robot()
    finally:
        avoider.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYTHON_EOF

    chmod +x ~/rosmaster_a1_scripts/obstacle_avoider.py
    print_info "Obstacle avoider created!"
}

# Create autonomous patrol node
create_autonomous_patrol() {
    print_info "Creating autonomous patrol node..."
    
    mkdir -p ~/rosmaster_a1_scripts
    
    cat > ~/rosmaster_a1_scripts/autonomous_patrol.py << 'PYTHON_EOF'
#!/usr/bin/env python3
"""
ROSMASTER A1 Autonomous Patrol Node
Patrols between waypoints autonomously
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import yaml
import math
import os


class AutonomousPatrol(Node):
    def __init__(self):
        super().__init__('autonomous_patrol')
        
        # Parameters
        self.declare_parameter('patrol_points', ['A', 'B', 'C'])
        self.declare_parameter('loop', True)
        self.declare_parameter('wait_time', 5.0)  # seconds at each point
        
        self.patrol_points = self.get_parameter('patrol_points').value
        self.loop = self.get_parameter('loop').value
        self.wait_time = self.get_parameter('wait_time').value
        
        # Load navigation points
        self.nav_points = self.load_nav_points()
        
        # Action client for Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        
        # Subscribers
        self.enable_sub = self.create_subscription(
            Bool, '/patrol/enable', self.enable_callback, 10
        )
        
        # State
        self.enabled = False
        self.current_waypoint_idx = 0
        self.navigating = False
        
        self.get_logger().info(f"Autonomous Patrol initialized!")
        self.get_logger().info(f"Patrol points: {self.patrol_points}")
    
    def load_nav_points(self):
        """Load navigation points from YAML"""
        config_path = os.path.expanduser('~/rosmaster_a1_config/nav_points.yaml')
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                return config.get('navigation_points', {})
        except FileNotFoundError:
            return {
                'A': [1.0, 0.0, 0.0],
                'B': [0.0, 1.0, 1.57],
                'C': [-1.0, 0.0, 3.14],
            }
    
    def enable_callback(self, msg):
        self.enabled = msg.data
        
        if self.enabled:
            self.get_logger().info("Patrol enabled, starting...")
            self.current_waypoint_idx = 0
            self.start_patrol()
        else:
            self.get_logger().info("Patrol disabled")
            self.navigating = False
    
    def start_patrol(self):
        """Start patrol sequence"""
        if not self.enabled:
            return
        
        if self.current_waypoint_idx >= len(self.patrol_points):
            if self.loop:
                self.current_waypoint_idx = 0
            else:
                self.get_logger().info("Patrol complete!")
                self.enabled = False
                return
        
        point_name = self.patrol_points[self.current_waypoint_idx]
        self.navigate_to_point(point_name)
    
    def navigate_to_point(self, point_name):
        """Navigate to a named point"""
        if point_name not in self.nav_points:
            self.get_logger().error(f"Unknown point: {point_name}")
            self.current_waypoint_idx += 1
            self.start_patrol()
            return
        
        coords = self.nav_points[point_name]
        x, y, theta = coords[0], coords[1], coords[2]
        
        self.get_logger().info(f"Navigating to {point_name} ({x}, {y})")
        
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 not available!")
            return
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Send goal
        self.navigating = True
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Publish status
        status_msg = String()
        status_msg.data = f"navigating:{point_name}"
        self.status_pub.publish(status_msg)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected')
            self.current_waypoint_idx += 1
            self.start_patrol()
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        self.navigating = False
        point_name = self.patrol_points[self.current_waypoint_idx]
        
        self.get_logger().info(f"Reached {point_name}")
        
        # Publish status
        status_msg = String()
        status_msg.data = f"reached:{point_name}"
        self.status_pub.publish(status_msg)
        
        # Wait at waypoint
        self.create_timer(self.wait_time, self.waypoint_wait_complete, oneshot=True)
    
    def waypoint_wait_complete(self):
        """Called after waiting at waypoint"""
        self.current_waypoint_idx += 1
        self.start_patrol()
    
    def feedback_callback(self, feedback_msg):
        pass  # Optionally log progress


def main(args=None):
    rclpy.init(args=args)
    
    patrol = AutonomousPatrol()
    
    try:
        rclpy.spin(patrol)
    except KeyboardInterrupt:
        pass
    finally:
        patrol.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYTHON_EOF

    chmod +x ~/rosmaster_a1_scripts/autonomous_patrol.py
    print_info "Autonomous patrol created!"
}

# Launch line following mode
launch_line_following() {
    local color="${1:-green}"
    
    print_info "Launching line following mode (color: ${color})..."
    
    source_ros2
    
    # Launch camera if not already running
    if ! ros2 topic list | grep -q "/camera/color/image_raw"; then
        print_info "Starting camera..."
        ./02_setup_depth_camera.sh launch &
        sleep 5
    fi
    
    # Launch line follower
    python3 ~/rosmaster_a1_scripts/line_follower.py \
        --ros-args -p line_color:=${color} &
    
    sleep 2
    
    # Enable line following
    ros2 topic pub --once /line_follower/enable std_msgs/Bool "data: true"
    
    print_info "Line following started!"
    print_info "To change color: ros2 topic pub --once /line_follower/color std_msgs/String \"data: 'red'\""
    print_info "To stop: ros2 topic pub --once /line_follower/enable std_msgs/Bool \"data: false\""
}

# Launch obstacle avoidance mode
launch_obstacle_avoidance() {
    print_info "Launching obstacle avoidance mode..."
    
    source_ros2
    
    # Launch LiDAR if not already running
    if ! ros2 topic list | grep -q "/scan"; then
        print_info "Starting LiDAR..."
        ./04_setup_navigation.sh launch_lidar &
        sleep 3
    fi
    
    # Launch obstacle avoider
    python3 ~/rosmaster_a1_scripts/obstacle_avoider.py &
    
    sleep 2
    
    # Enable obstacle avoidance
    ros2 topic pub --once /obstacle_avoider/enable std_msgs/Bool "data: true"
    
    print_info "Obstacle avoidance started!"
}

# Launch autonomous patrol
launch_patrol() {
    print_info "Launching autonomous patrol..."
    
    source_ros2
    
    # Make sure navigation is running
    if ! ros2 topic list | grep -q "/navigate_to_pose"; then
        print_warning "Nav2 not detected. Start navigation first:"
        echo "  ./04_setup_navigation.sh nav"
        return 1
    fi
    
    # Launch patrol
    python3 ~/rosmaster_a1_scripts/autonomous_patrol.py &
    
    sleep 2
    
    # Enable patrol
    ros2 topic pub --once /patrol/enable std_msgs/Bool "data: true"
    
    print_info "Autonomous patrol started!"
}

# Show usage
show_usage() {
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Setup Options:"
    echo "  install          Install dependencies"
    echo "  setup            Create all autonomous driving scripts"
    echo ""
    echo "Run Options:"
    echo "  line [color]     Start line following (default: green)"
    echo "  obstacle         Start obstacle avoidance"
    echo "  patrol           Start autonomous patrol"
    echo ""
    echo "Available line colors: red, green, blue, yellow"
}

# Main execution
print_header

case "${1:-help}" in
    install)
        install_auto_deps
        ;;
    setup)
        create_line_follower
        create_obstacle_avoider
        create_autonomous_patrol
        ;;
    line)
        launch_line_following "$2"
        ;;
    obstacle)
        launch_obstacle_avoidance
        ;;
    patrol)
        launch_patrol
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
