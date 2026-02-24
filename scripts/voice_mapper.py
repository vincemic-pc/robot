#!/usr/bin/env python3
"""
Voice-Controlled Mapping Explorer with Multi-Camera and VSLAM Support
======================================================================
Combines:
- Whisper STT + GPT-4o Brain/Vision + TTS
- LiDAR Navigation (slam_toolbox)
- Isaac VSLAM with OAK-D Pro (when available)
- Camera abstraction: HP60C / OAK-D Pro / RealSense

SLAM Modes:
- 'lidar': slam_toolbox with LiDAR (default, works with any camera)
- 'vslam': Isaac ROS Visual SLAM with stereo camera (OAK-D Pro recommended)
- 'hybrid': Both LiDAR SLAM + VSLAM for maximum accuracy
"""
import os
import sys
import time
import base64
import random
import threading
import json
import tempfile
import subprocess
import math
import wave
import yaml
import datetime
import numpy as np
import pyaudio
from io import BytesIO
from openai import OpenAI
from pathlib import Path
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Dict, Tuple

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import Empty
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from action_msgs.msg import GoalStatus

# Try CV imports
try:
    from cv_bridge import CvBridge
    import cv2
    HAS_CV = True
except ImportError:
    HAS_CV = False

# Try scipy for audio resampling
try:
    from scipy.io import wavfile
    from scipy import signal
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# === Camera and SLAM Configuration ===

class CameraType(Enum):
    """Supported camera types"""
    HP60C = "hp60c"      # Yahboom HP60C (USB 2.0, limited)
    OAK_D_PRO = "oakd"   # Luxonis OAK-D Pro (USB 3.0, VSLAM-ready)
    REALSENSE = "realsense"  # Intel RealSense D435i
    

class SlamMode(Enum):
    """SLAM modes available"""
    LIDAR = "lidar"      # LiDAR-only SLAM (slam_toolbox)
    VSLAM = "vslam"      # Visual SLAM only (Isaac ROS)
    HYBRID = "hybrid"    # Both LiDAR + Visual SLAM


@dataclass
class CameraConfig:
    """Camera topic configuration"""
    camera_type: CameraType
    rgb_topic: str
    depth_topic: str
    left_topic: str = ""      # For stereo VSLAM
    right_topic: str = ""     # For stereo VSLAM
    camera_info_left: str = ""
    camera_info_right: str = ""
    imu_topic: str = ""       # For VIO
    needs_enhancement: bool = False  # HP60C needs brightness boost
    

# Pre-defined camera configurations
CAMERA_CONFIGS = {
    CameraType.HP60C: CameraConfig(
        camera_type=CameraType.HP60C,
        rgb_topic="/ascamera_hp60c/camera_publisher/rgb0/image",
        depth_topic="/ascamera_hp60c/camera_publisher/depth0/image",
        needs_enhancement=True,  # Dark images need enhancement
    ),
    CameraType.OAK_D_PRO: CameraConfig(
        camera_type=CameraType.OAK_D_PRO,
        rgb_topic="/oak/rgb/image_raw",
        depth_topic="/oak/stereo/image_raw",
        left_topic="/oak/left/image_rect",
        right_topic="/oak/right/image_rect",
        camera_info_left="/oak/left/camera_info",
        camera_info_right="/oak/right/camera_info",
        imu_topic="/oak/imu/data",
        needs_enhancement=False,
    ),
    CameraType.REALSENSE: CameraConfig(
        camera_type=CameraType.REALSENSE,
        rgb_topic="/camera/color/image_raw",
        depth_topic="/camera/depth/image_rect_raw",
        left_topic="/camera/infra1/image_rect_raw",
        right_topic="/camera/infra2/image_rect_raw",
        camera_info_left="/camera/infra1/camera_info",
        camera_info_right="/camera/infra2/camera_info",
        imu_topic="/camera/imu",
        needs_enhancement=False,
    ),
}


class DiscoveryLog:
    """Tracks exploration history with observations and path"""
    
    def __init__(self, log_dir="~/exploration_logs"):
        self.log_dir = Path(log_dir).expanduser()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self.session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_file = self.log_dir / f"exploration_{self.session_id}.yaml"
        
        self.path = []  # List of (x, y, theta, timestamp)
        self.discoveries = []  # List of {observation, position, timestamp, image_file}
        self.obstacles = []  # List of {position, direction, distance, timestamp}
        self.start_time = time.time()
        self.total_distance = 0.0
        
    def add_position(self, x, y, theta):
        """Record current position"""
        self.path.append({
            'x': float(x),
            'y': float(y),
            'theta': float(theta),
            'timestamp': time.time() - self.start_time
        })
        
        # Calculate distance traveled
        if len(self.path) > 1:
            prev = self.path[-2]
            dx = x - prev['x']
            dy = y - prev['y']
            self.total_distance += math.sqrt(dx*dx + dy*dy)
    
    def add_discovery(self, observation, x, y, theta, image_data=None):
        """Record an interesting observation"""
        discovery = {
            'observation': observation,
            'position': {'x': float(x), 'y': float(y), 'theta': float(theta)},
            'timestamp': time.time() - self.start_time,
            'time_str': datetime.datetime.now().strftime("%H:%M:%S")
        }
        
        # Save image if provided
        if image_data is not None and HAS_CV:
            img_file = self.log_dir / f"discovery_{self.session_id}_{len(self.discoveries):03d}.jpg"
            cv2.imwrite(str(img_file), image_data)
            discovery['image_file'] = str(img_file)
        
        self.discoveries.append(discovery)
        self.save()
        return len(self.discoveries)
    
    def add_obstacle(self, x, y, direction, distance):
        """Record detected obstacle"""
        self.obstacles.append({
            'position': {'x': float(x), 'y': float(y)},
            'direction': direction,
            'distance': float(distance),
            'timestamp': time.time() - self.start_time
        })
    
    def save(self):
        """Save log to YAML file"""
        data = {
            'session_id': self.session_id,
            'start_time': datetime.datetime.fromtimestamp(self.start_time).isoformat(),
            'duration_seconds': time.time() - self.start_time,
            'total_distance_meters': self.total_distance,
            'num_discoveries': len(self.discoveries),
            'num_obstacles': len(self.obstacles),
            'path_points': len(self.path),
            'discoveries': self.discoveries,
            'obstacles': self.obstacles[-100:],  # Keep last 100 obstacles
            'path': self.path[-1000:]  # Keep last 1000 path points
        }
        
        with open(self.session_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
    
    def get_summary(self):
        """Get exploration summary"""
        duration = time.time() - self.start_time
        return {
            'duration_minutes': duration / 60,
            'distance_meters': self.total_distance,
            'discoveries': len(self.discoveries),
            'obstacles_found': len(self.obstacles)
        }


class VoiceMapper(Node):
    def __init__(self, camera_type: CameraType = None, slam_mode: SlamMode = SlamMode.LIDAR):
        super().__init__("voice_mapper")
        
        # === Camera and SLAM Mode Selection ===
        # Auto-detect camera if not specified
        if camera_type is None:
            camera_type = self._detect_camera()
        
        self.camera_type = camera_type
        self.camera_config = CAMERA_CONFIGS[camera_type]
        self.slam_mode = slam_mode
        
        self.get_logger().info(f"📷 Camera: {camera_type.value}")
        self.get_logger().info(f"🗺️ SLAM Mode: {slam_mode.value}")
        
        # OpenAI client
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            self.get_logger().error("OPENAI_API_KEY not set!")
            sys.exit(1)
        
        self.client = OpenAI(api_key=api_key)
        self.get_logger().info("OpenAI client initialized")
        
        # Discovery log
        self.discovery_log = DiscoveryLog()
        
        # Robot control
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # VSLAM state
        self.vslam_process = None
        self.vslam_available = False
        self.vslam_tracking = False
        self.vslam_pose = None
        self.vslam_path = []  # Visual odometry path
        
        # VSLAM services (will be created when VSLAM starts)
        self.vslam_save_map_client = None
        self.vslam_load_map_client = None
        self.vslam_reset_client = None
        
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions - use camera config for topics
        # RGB camera - primary visual sensor
        self.camera_sub = self.create_subscription(
            Image, self.camera_config.rgb_topic,
            self.camera_callback, reliable_qos
        )
        self.get_logger().info(f"Subscribed to RGB: {self.camera_config.rgb_topic}")
        
        # Depth camera - for on-demand queries (Yahboom pattern)
        self.depth_sub = self.create_subscription(
            Image, self.camera_config.depth_topic,
            self.depth_callback, reliable_qos
        )
        self.get_logger().info(f"Subscribed to Depth: {self.camera_config.depth_topic}")
        
        # Stereo cameras for VSLAM (OAK-D Pro / RealSense)
        self.left_image = None
        self.right_image = None
        if self.camera_config.left_topic:
            self.left_sub = self.create_subscription(
                Image, self.camera_config.left_topic,
                self.left_callback, reliable_qos
            )
            self.right_sub = self.create_subscription(
                Image, self.camera_config.right_topic,
                self.right_callback, reliable_qos
            )
            self.get_logger().info(f"Subscribed to stereo: {self.camera_config.left_topic}")
        
        # VSLAM odometry (from Isaac ROS Visual SLAM)
        self.vslam_odom_sub = self.create_subscription(
            Odometry, "/visual_slam/tracking/odometry",
            self.vslam_odom_callback, best_effort_qos
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, best_effort_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, best_effort_qos
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, best_effort_qos
        )
        
        # Sensor state
        self.latest_image = None
        self.latest_image_time = None
        self.latest_depth = None
        self.latest_depth_time = None
        self.depth_query_cache = {}  # Cache for on-demand depth queries
        self.depth_cache_timeout = 0.5  # seconds
        self.latest_scan = None
        self.latest_odom = None
        self.latest_map = None
        self.map_info = None
        
        # Robot state
        self.running = True
        self.exploring = False
        self.mapping = False
        self.speaking = False
        self.listening = False
        self.slam_process = None
        self.emergency_stop_triggered = False
        
        # Position tracking
        self.current_position = {'x': 0, 'y': 0, 'theta': 0}
        self.start_position = None
        
        # Obstacle detection
        self.obstacles = {}
        self.obstacle_distances = {
            "front": float('inf'), "front_left": float('inf'),
            "front_right": float('inf'), "left": float('inf'),
            "right": float('inf'), "back": float('inf')
        }
        self.detected_gaps = []  # Doorway/gap detection
        self._last_gap_log = 0
        
        # Movement parameters (Ackerman steering)
        self.min_obstacle_dist = 0.5  # Emergency stop distance (was 0.4)
        self.slow_dist = 1.0  # Start slowing down (was 0.8)
        self.emergency_dist = 0.3  # Absolute minimum - reverse if closer
        self.linear_speed = 0.12  # Max forward speed (was 0.15)
        self.slow_speed = 0.06  # Speed near obstacles (was 0.08)
        self.angular_speed = 0.3  # Lower for Ackerman
        self.min_turn_radius = 0.3  # Ackerman constraint
        
        # Exploration settings
        self.look_interval = 15.0
        self.path_record_interval = 0.5
        self.last_path_record = 0
        self.stuck_counter = 0
        
        # Audio settings
        self.sample_rate = 48000
        self.whisper_rate = 16000
        self.channels = 1
        self.chunk_size = 4096
        self.silence_threshold = 500
        self.silence_duration = 1.5
        self.min_audio_duration = 0.3
        self.mic_index = self._find_microphone()
        
        # CV Bridge
        if HAS_CV:
            self.bridge = CvBridge()
        
        # Nav2 integration
        self._init_nav2()
        
        # Conversation
        self.conversation = []
        self.system_prompt = self._build_system_prompt()
        
        # Maps directory
        self.maps_dir = Path("~/maps").expanduser()
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info("Voice Mapper initialized!")

    def _find_microphone(self):
        """Find microphone - prefer PulseAudio"""
        p = pyaudio.PyAudio()
        pulse_index = None
        
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            name = info.get('name', '').lower()
            max_input = info.get('maxInputChannels', 0)
            
            if 'pulse' in name and max_input > 0:
                pulse_index = i
                break
        
        p.terminate()
        
        if pulse_index is not None:
            self.get_logger().info(f"Using PulseAudio device (index {pulse_index})")
            return pulse_index
        
        self.get_logger().warning("No PulseAudio found, using device 0")
        return 0

    def _init_nav2(self):
        """Initialize Nav2 action clients and services"""
        self.nav2_available = False
        self.nav2_process = None
        self.navigating = False
        self.current_goal = None
        self.nav_feedback = None
        
        # Nav2 action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Initial pose publisher (for AMCL)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Services for costmap clearing
        self.clear_global_costmap = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local_costmap = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        
        # Frontier points for exploration
        self.frontiers = []
        self.visited_goals = []
        
        self.get_logger().info("Nav2 clients initialized (will connect when Nav2 starts)")

    def start_nav2(self):
        """Launch Nav2 stack with our config"""
        if self.nav2_process is not None:
            self.get_logger().warning("Nav2 already running")
            return True
        
        self.get_logger().info("🚀 Starting Nav2 navigation stack...")
        
        # Check if we have a map
        if self.latest_map is None:
            self.get_logger().warning("No map available - Nav2 needs a map for navigation")
            self.speak("Starting mapping first, then navigation.")
            self.start_slam()
            time.sleep(3)
        
        try:
            # Launch Nav2 with our parameters
            nav2_cmd = [
                "ros2", "launch", "nav2_bringup", "navigation_launch.py",
                "use_sim_time:=false",
                f"params_file:=/home/jetson/robot_scripts/nav2_params.yaml"
            ]
            
            self.nav2_process = subprocess.Popen(
                nav2_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env={**os.environ, 'ROS_DOMAIN_ID': '62'}
            )
            
            self.get_logger().info("Nav2 launch started, waiting for action servers...")
            
            # Wait for Nav2 to be ready
            for i in range(30):  # 30 second timeout
                if self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
                    self.nav2_available = True
                    self.get_logger().info("✅ Nav2 is ready!")
                    self.beep("success")
                    return True
                self.get_logger().info(f"Waiting for Nav2... ({i+1}/30)")
            
            self.get_logger().error("❌ Nav2 failed to start within timeout")
            self.beep("error")
            self.speak("Navigation system failed to start. Using basic movement instead.")
            return False
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to start Nav2: {e}")
            self.beep("error")
            self.speak(f"Navigation system error. Using basic movement instead.")
            return False

    def stop_nav2(self):
        """Stop Nav2 stack"""
        if self.nav2_process:
            self.nav2_process.terminate()
            self.nav2_process.wait()
            self.nav2_process = None
            self.nav2_available = False
            self.get_logger().info("Nav2 stopped")

    def navigate_to(self, x, y, theta=0.0):
        """Navigate to a specific pose using Nav2"""
        if not self.nav2_available:
            self.get_logger().warning("Nav2 not available - starting it now...")
            if not self.start_nav2():
                return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        
        # Convert theta to quaternion (simplified for 2D)
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.get_logger().info(f"🎯 Navigating to ({x:.2f}, {y:.2f})")
        self.navigating = True
        self.current_goal = (x, y, theta)
        
        # Send goal
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, 
            feedback_callback=self._nav_feedback_callback
        )
        send_goal_future.add_done_callback(self._nav_goal_response_callback)
        return True

    def _nav_goal_response_callback(self, future):
        """Handle Nav2 goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Navigation goal rejected!")
            self.navigating = False
            return
        
        self.get_logger().info("Navigation goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_feedback_callback(self, feedback_msg):
        """Handle Nav2 navigation feedback"""
        feedback = feedback_msg.feedback
        self.nav_feedback = {
            'distance_remaining': feedback.distance_remaining,
            'time_elapsed': feedback.navigation_time.sec
        }

    def _nav_result_callback(self, future):
        """Handle Nav2 navigation result"""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Navigation succeeded!")
            self.beep("success")
            if self.current_goal:
                self.visited_goals.append(self.current_goal)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Navigation canceled")
        else:
            self.get_logger().error(f"❌ Navigation failed with status: {status}")
            self.beep("warning")
            # Explain why
            if status == GoalStatus.STATUS_ABORTED:
                self.speak("Navigation aborted. Path may be blocked.")
            elif status == GoalStatus.STATUS_REJECTED:
                self.speak("Navigation rejected. Goal may be unreachable.")
            else:
                self.speak("Navigation failed. Trying a different approach.")
        
        self.navigating = False
        self.current_goal = None

    def cancel_navigation(self):
        """Cancel current navigation"""
        if self.navigating and self.nav_to_pose_client:
            self.nav_to_pose_client._cancel_goal_async()
            self.navigating = False
            self.get_logger().info("Navigation canceled")

    def find_frontiers(self):
        """Find frontier points from the current map for exploration.
        Frontiers are edges between known (free) and unknown space.
        Prioritizes larger frontiers which often indicate doorways to new rooms.
        """
        if self.latest_map is None:
            self.get_logger().warning("No map available for frontier detection")
            return []
        
        map_data = np.array(self.latest_map.data).reshape(
            (self.latest_map.info.height, self.latest_map.info.width))
        
        resolution = self.latest_map.info.resolution
        origin_x = self.latest_map.info.origin.position.x
        origin_y = self.latest_map.info.origin.position.y
        
        # Find all frontier cells (free cells adjacent to unknown)
        frontier_cells = []
        
        for i in range(2, map_data.shape[0] - 2):
            for j in range(2, map_data.shape[1] - 2):
                if map_data[i, j] == 0:  # Free cell
                    # Check 8-connected neighbors for unknown (-1)
                    has_unknown = False
                    has_free = False
                    for di in [-1, 0, 1]:
                        for dj in [-1, 0, 1]:
                            if di == 0 and dj == 0:
                                continue
                            neighbor = map_data[i + di, j + dj]
                            if neighbor == -1:
                                has_unknown = True
                            elif neighbor == 0:
                                has_free = True
                    
                    # Frontier: free cell with unknown neighbor and at least one free neighbor
                    if has_unknown and has_free:
                        world_x = origin_x + j * resolution
                        world_y = origin_y + i * resolution
                        frontier_cells.append((world_x, world_y))
        
        self.get_logger().info(f"Found {len(frontier_cells)} raw frontier cells")
        
        if len(frontier_cells) == 0:
            return []
        
        # Cluster frontiers - larger clusters are better (doorways, corridors)
        clusters = self._cluster_frontiers_advanced(frontier_cells, resolution)
        
        self.get_logger().info(f"Clustered into {len(clusters)} frontier regions")
        
        return clusters

    def _cluster_frontiers_advanced(self, frontier_cells, resolution, min_cluster_size=3):
        """Cluster frontier cells and rank by size and potential.
        Larger clusters often indicate doorways to unexplored rooms.
        """
        if not frontier_cells:
            return []
        
        # Use simple distance-based clustering
        cluster_dist = resolution * 5  # Cells within 5 grid cells
        clusters = []
        used = set()
        
        for i, (x, y) in enumerate(frontier_cells):
            if i in used:
                continue
            
            # Start new cluster
            cluster = [(x, y)]
            used.add(i)
            
            # Find all connected frontier cells
            queue = [i]
            while queue:
                current = queue.pop(0)
                cx, cy = frontier_cells[current]
                
                for j, (x2, y2) in enumerate(frontier_cells):
                    if j in used:
                        continue
                    dist = math.sqrt((cx - x2)**2 + (cy - y2)**2)
                    if dist < cluster_dist:
                        cluster.append((x2, y2))
                        used.add(j)
                        queue.append(j)
            
            # Only keep clusters above minimum size (filters noise)
            if len(cluster) >= min_cluster_size:
                # Calculate centroid
                cx = sum(p[0] for p in cluster) / len(cluster)
                cy = sum(p[1] for p in cluster) / len(cluster)
                clusters.append({
                    'x': cx,
                    'y': cy,
                    'size': len(cluster),
                    'points': cluster
                })
        
        # Sort by size (larger = more interesting, likely doorways)
        clusters.sort(key=lambda c: c['size'], reverse=True)
        
        # Return as (x, y, size) tuples
        return [(c['x'], c['y'], c['size']) for c in clusters]

    def choose_frontier(self):
        """Choose best frontier to explore - prioritizes discovery of new areas.
        Strategy: Balance between exploration (large distant frontiers) and efficiency.
        """
        frontiers = self.find_frontiers()
        
        if not frontiers:
            self.get_logger().info("No frontiers found - map may be complete")
            return None
        
        # Filter out recently visited areas
        available = []
        for frontier in frontiers:
            fx, fy = frontier[0], frontier[1]
            size = frontier[2] if len(frontier) > 2 else 1
            
            # Check if too close to visited goals
            too_close = False
            for vx, vy, _ in self.visited_goals:
                if math.sqrt((fx - vx)**2 + (fy - vy)**2) < 0.8:
                    too_close = True
                    break
            
            if not too_close:
                available.append((fx, fy, size))
        
        if not available:
            self.get_logger().info("All frontiers near visited locations - clearing visited list")
            self.visited_goals = self.visited_goals[-3:]  # Keep only last 3
            available = [(f[0], f[1], f[2] if len(f) > 2 else 1) for f in frontiers]
        
        if not available:
            return None
        
        # Score each frontier: prefer LARGER and FARTHER frontiers (new rooms!)
        cx, cy = self.current_position['x'], self.current_position['y']
        
        scored = []
        for fx, fy, size in available:
            dist = math.sqrt((fx - cx)**2 + (fy - cy)**2)
            
            # Skip if too close or too far
            if dist < 0.5:
                continue
            if dist > 10.0:  # Don't go too far in one step
                continue
            
            # Score: larger frontiers get bonus, medium distance preferred
            # This encourages going through doorways (large frontiers) to new rooms
            size_score = min(size / 10.0, 3.0)  # Cap size bonus at 3x
            
            # Prefer frontiers 2-5m away (good for room exploration)
            if dist < 2.0:
                dist_score = dist / 2.0
            elif dist < 5.0:
                dist_score = 1.0 + (dist - 2.0) * 0.1  # Slight bonus for 2-5m
            else:
                dist_score = 1.3 - (dist - 5.0) * 0.05  # Penalty for very far
            
            score = size_score * dist_score
            scored.append((fx, fy, score, size, dist))
        
        if not scored:
            # Fallback: just pick any frontier
            return (available[0][0], available[0][1])
        
        # Sort by score (highest first)
        scored.sort(key=lambda x: x[2], reverse=True)
        
        # Log top choices
        self.get_logger().info("Top frontier candidates:")
        for i, (fx, fy, score, size, dist) in enumerate(scored[:3]):
            self.get_logger().info(f"  {i+1}. ({fx:.2f}, {fy:.2f}) score={score:.2f} size={size} dist={dist:.2f}m")
        
        # Sometimes pick 2nd or 3rd best for variety (exploration)
        if len(scored) >= 3 and random.random() < 0.3:
            choice = random.choice(scored[:3])
            self.get_logger().info(f"🎲 Randomly chose frontier {scored.index(choice)+1} for variety")
        else:
            choice = scored[0]
        
        return (choice[0], choice[1])

    def _build_system_prompt(self):
        # Build dynamic prompt based on camera capabilities
        vslam_capable = self.camera_type in [CameraType.OAK_D_PRO, CameraType.REALSENSE]
        
        base_prompt = """You are a voice-controlled mapping robot with camera, depth camera, LiDAR, and SLAM capabilities.

CAPABILITIES:
- Move and explore environments
- Build maps using SLAM (Simultaneous Localization and Mapping)
- Look around and describe what you see
- Measure distance to specific objects using depth camera
- Track and log discoveries
- Navigate to specific locations using Nav2 path planning
- Avoid obstacles automatically
- Discover new rooms by navigating through doorways"""

        if vslam_capable:
            base_prompt += """
- Visual SLAM (VSLAM) for precise localization using stereo camera
- Save and load visual maps for relocalization"""

        base_prompt += """

COMMANDS (respond with JSON):

1. Movement: {"action": "move", "linear": 0.15, "angular": 0.0, "duration": 2.0, "speech": "Moving forward"}
   - linear: -0.2 to 0.2 m/s (Ackerman steering limits speed)
   - angular: -0.3 to 0.3 rad/s (turning rate)

2. Look/Observe: {"action": "look", "speech": "Let me see..."}

3. Get Distance: {"action": "get_dist", "target": "door", "speech": "Measuring distance..."}
   - Finds object in camera, measures distance with depth sensor
   - Examples: "door", "person", "chair", "wall", "obstacle"

4. Start Mapping: {"action": "start_mapping", "speech": "Starting SLAM mapping!"}
   - Launches slam_toolbox (LiDAR SLAM) to build a 2D occupancy map

5. Stop Mapping: {"action": "stop_mapping", "map_name": "my_map", "speech": "Saving the map"}
   - Saves the current map"""

        if vslam_capable:
            base_prompt += """

6. Start Visual SLAM: {"action": "start_vslam", "speech": "Starting visual SLAM!"}
   - Launches Isaac VSLAM for high-precision visual odometry
   - Requires OAK-D Pro or RealSense camera
   - Better for indoor localization with rich visual features

7. Stop Visual SLAM: {"action": "stop_vslam", "speech": "Stopping visual SLAM"}
   - Stops the VSLAM tracking

8. VSLAM Status: {"action": "vslam_status", "speech": "Checking visual slam..."}
   - Reports VSLAM tracking status and path length"""

        base_prompt += """

9. Start Exploring: {"action": "explore", "speech": "Beginning exploration!"}
   - Uses Nav2 frontier exploration to discover new rooms through doorways

10. Navigate To: {"action": "navigate", "x": 1.5, "y": 2.0, "speech": "Navigating to that location"}
    - Uses Nav2 path planning to reach coordinates

11. Stop: {"action": "stop", "speech": "Stopping"}

12. Status: {"action": "status", "speech": "Checking status..."}
    - Reports mapping progress, discoveries, distance

13. Map Status: {"action": "map_status", "speech": "Checking map..."}
    - Reports map size and coverage

14. Just talk: {"action": "speak", "speech": "Your response"}

CONTEXT:
- You know obstacle distances from LiDAR
- You can measure distance to specific objects with depth camera
- You track discoveries and interesting observations
- You can see map coverage grow as you explore
- You remember what you've seen
- Nav2 provides intelligent path planning around obstacles
- Doorway detection helps you find new rooms to explore"""

        if vslam_capable:
            base_prompt += f"""
- Visual SLAM available with {self.camera_type.value} camera
- VSLAM provides more accurate indoor localization than wheel odometry"""

        base_prompt += """

PERSONALITY:
- Enthusiastic cartographer/explorer
- Curious about new areas
- Celebrates discoveries
- Brief, natural responses

Always respond with valid JSON. Keep speech short (1-2 sentences)."""
        
        return base_prompt

    # === Sensor Callbacks ===
    
    def camera_callback(self, msg):
        self.latest_image = msg
        self.latest_image_time = time.time()

    def depth_callback(self, msg):
        """Depth image callback - cached for on-demand queries (Yahboom pattern)"""
        self.latest_depth = msg
        self.latest_depth_time = time.time()
        # Clear cache when new depth frame arrives
        self.depth_query_cache = {}

    def left_callback(self, msg):
        """Left stereo camera callback for VSLAM"""
        self.left_image = msg
    
    def right_callback(self, msg):
        """Right stereo camera callback for VSLAM"""
        self.right_image = msg
    
    def vslam_odom_callback(self, msg):
        """Isaac VSLAM odometry callback - high-frequency visual odometry"""
        self.vslam_pose = msg.pose.pose
        self.vslam_tracking = True
        
        # Update current position from VSLAM if in VSLAM or hybrid mode
        if self.slam_mode in [SlamMode.VSLAM, SlamMode.HYBRID]:
            pos = msg.pose.pose.position
            orient = msg.pose.pose.orientation
            
            # Extract yaw from quaternion
            siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
            cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
            theta = math.atan2(siny_cosp, cosy_cosp)
            
            # Track VSLAM path
            self.vslam_path.append({
                'x': pos.x, 'y': pos.y, 'z': pos.z,
                'theta': theta, 'time': time.time()
            })
            
            # Keep path manageable
            if len(self.vslam_path) > 10000:
                self.vslam_path = self.vslam_path[-5000:]

    def scan_callback(self, msg):
        self.latest_scan = msg
        if not msg.ranges:
            return
        
        ranges = np.array(msg.ranges)
        num_points = len(ranges)
        
        # Replace inf/nan with max range (indicates open space, not obstacle!)
        max_range = msg.range_max if msg.range_max > 0 else 12.0
        ranges_clean = np.where(np.isinf(ranges) | np.isnan(ranges), max_range, ranges)
        
        # LiDAR has 720 points covering 360°
        # Index 0 = front (0°), going counter-clockwise
        # Index 180 = left (90°), 360 = back (180°), 540 = right (270°)
        angle_per_point = 360.0 / num_points  # 0.5° per point
        
        # Define sectors with angles (more intuitive)
        # Front cone: -45° to +45° for main navigation
        front_half_angle = 45  # degrees
        front_points = int(front_half_angle / angle_per_point)
        
        sectors = {
            "front": (0, front_points),  # 0 to +45°
            "front_right": (num_points - front_points, num_points),  # -45° to 0°
            "front_left": (front_points, front_points * 2),  # +45° to +90°
            "left": (int(num_points * 0.2), int(num_points * 0.35)),  # +72° to +126°
            "back": (int(num_points * 0.4), int(num_points * 0.6)),  # +144° to +216°
            "right": (int(num_points * 0.65), int(num_points * 0.8)),  # +234° to +288°
        }
        
        # Process each sector
        for sector, (start, end) in sectors.items():
            if start < end:
                sector_ranges = ranges_clean[start:end]
            else:
                sector_ranges = np.concatenate([ranges_clean[start:], ranges_clean[:end]])
            
            if len(sector_ranges) > 0:
                # Use percentile to ignore outliers (dust, sensor noise)
                min_dist = np.percentile(sector_ranges, 10)  # 10th percentile
                self.obstacle_distances[sector] = min_dist
                
                # Mark as obstacle only if consistently close
                close_count = np.sum(sector_ranges < self.min_obstacle_dist)
                obstacle_ratio = close_count / len(sector_ranges)
                
                # Need >30% of sector blocked to count as obstacle
                is_blocked = obstacle_ratio > 0.3 and min_dist < self.min_obstacle_dist
                
                # Log significant obstacles (not every frame)
                if is_blocked and not self.obstacles.get(sector, False):
                    self.discovery_log.add_obstacle(
                        self.current_position['x'],
                        self.current_position['y'],
                        sector, min_dist
                    )
                    self.get_logger().warning(f"⚠️ Obstacle {sector}: {min_dist:.2f}m ({obstacle_ratio*100:.0f}% blocked)")
                
                self.obstacles[sector] = is_blocked
        
        # Wide front arc for emergency stop (combine front sectors)
        front_wide_start = num_points - front_points  # -45°
        front_wide_end = front_points * 2  # +90°
        front_wide = np.concatenate([ranges_clean[front_wide_start:], ranges_clean[:front_wide_end]])
        self.min_front_distance = np.percentile(front_wide, 5) if len(front_wide) > 0 else max_range
        self.obstacle_distances["front_wide"] = self.min_front_distance
        
        # === DOORWAY/GAP DETECTION ===
        # Look for gaps: sequences where distance jumps significantly (doorways!)
        self._detect_doorways(ranges_clean, num_points, angle_per_point)
        
        # Emergency stop check
        if self.min_front_distance < self.emergency_dist:
            self.get_logger().error(f"🛑 EMERGENCY: Object at {self.min_front_distance:.2f}m!")
            self.emergency_stop_triggered = True
            if self.exploring:
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
    
    def _detect_doorways(self, ranges, num_points, angle_per_point):
        """Detect doorways/gaps in LiDAR data - openings where distance suddenly increases."""
        # Look at front 180° arc (front_right through front to front_left)
        front_arc_points = int(num_points / 2)
        start_idx = num_points - int(front_arc_points / 2)
        
        # Get front arc (wrapping around)
        front_arc = np.concatenate([ranges[start_idx:], ranges[:int(front_arc_points / 2)]])
        
        # Detect gaps: where distance increases by >1m compared to neighbors
        gaps = []
        window = 10  # Points to check
        min_gap_width = 0.6  # Minimum 60cm gap (door width)
        
        for i in range(window, len(front_arc) - window):
            center_dist = front_arc[i]
            left_dist = np.min(front_arc[i-window:i])
            right_dist = np.min(front_arc[i+1:i+window+1])
            
            # Gap: center is much farther than both sides
            if center_dist > left_dist + 1.0 and center_dist > right_dist + 1.0:
                # Check gap width (consecutive far points)
                gap_start = i
                gap_end = i
                while gap_start > 0 and front_arc[gap_start-1] > left_dist + 0.5:
                    gap_start -= 1
                while gap_end < len(front_arc)-1 and front_arc[gap_end+1] > right_dist + 0.5:
                    gap_end += 1
                
                gap_width_points = gap_end - gap_start + 1
                gap_width_degrees = gap_width_points * angle_per_point
                gap_width_meters = 2 * center_dist * math.tan(math.radians(gap_width_degrees / 2))
                
                if gap_width_meters >= min_gap_width:
                    # Calculate gap angle from robot's front (0°)
                    gap_center_idx = (gap_start + gap_end) // 2
                    angle_from_start = gap_center_idx * angle_per_point
                    gap_angle = angle_from_start - 90  # Convert to -90 to +90 range
                    
                    gaps.append({
                        'angle': gap_angle,
                        'distance': center_dist,
                        'width': gap_width_meters,
                        'center_idx': i
                    })
        
        # Store detected gaps
        self.detected_gaps = gaps
        
        if gaps and not hasattr(self, '_last_gap_log') or time.time() - self._last_gap_log > 5:
            self._last_gap_log = time.time()
            for gap in gaps[:3]:  # Log top 3
                self.get_logger().info(f"🚪 Gap detected: {gap['angle']:.0f}° away, {gap['distance']:.1f}m, width={gap['width']:.1f}m")

    def odom_callback(self, msg):
        self.latest_odom = msg
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_position = {'x': pos.x, 'y': pos.y, 'theta': theta}
        
        if self.start_position is None:
            self.start_position = self.current_position.copy()
        
        # Record path periodically
        if time.time() - self.last_path_record > self.path_record_interval:
            self.discovery_log.add_position(pos.x, pos.y, theta)
            self.last_path_record = time.time()

    def map_callback(self, msg):
        self.latest_map = msg
        self.map_info = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y
        }
        
        # Calculate map coverage
        data = np.array(msg.data)
        known = np.sum((data >= 0) & (data <= 100))
        total = len(data)
        self.map_coverage = known / total if total > 0 else 0

    # === Yahboom Depth Camera Pattern ===
    
    def get_dist(self, x: int, y: int):
        """
        Yahboom-style single-point depth query: get_dist(x, y)
        
        This is how Yahboom uses the depth camera - NOT for streaming point clouds,
        but for on-demand distance queries to specific pixel coordinates.
        
        Args:
            x: Pixel x coordinate (0-639)
            y: Pixel y coordinate (0-479)
            
        Returns:
            dict with 'distance' (meters), 'valid' (bool)
        """
        # Check cache
        cache_key = (x, y)
        if cache_key in self.depth_query_cache:
            cached_time, cached_result = self.depth_query_cache[cache_key]
            if time.time() - cached_time < self.depth_cache_timeout:
                return cached_result
        
        # Check if we have depth data
        if self.latest_depth is None:
            self.get_logger().warning("No depth data available")
            return {'distance': -1.0, 'valid': False, 'x': x, 'y': y}
        
        # Check depth age
        if self.latest_depth_time and (time.time() - self.latest_depth_time) > 2.0:
            self.get_logger().warning("Depth data is stale")
            return {'distance': -1.0, 'valid': False, 'x': x, 'y': y}
        
        try:
            # Convert depth image
            if HAS_CV:
                depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding="passthrough")
            else:
                # Manual conversion
                depth_data = np.frombuffer(self.latest_depth.data, dtype=np.uint16)
                depth_image = depth_data.reshape((self.latest_depth.height, self.latest_depth.width))
            
            # Clamp coordinates
            x = max(0, min(x, depth_image.shape[1] - 1))
            y = max(0, min(y, depth_image.shape[0] - 1))
            
            # Get depth value at pixel (sample a small region for robustness)
            region_size = 5
            x_start = max(0, x - region_size)
            x_end = min(depth_image.shape[1], x + region_size + 1)
            y_start = max(0, y - region_size)
            y_end = min(depth_image.shape[0], y + region_size + 1)
            
            region = depth_image[y_start:y_end, x_start:x_end]
            valid_depths = region[region > 0]
            
            if len(valid_depths) == 0:
                result = {'distance': -1.0, 'valid': False, 'x': x, 'y': y}
            else:
                # HP60C depth is in millimeters, convert to meters
                depth_mm = float(np.median(valid_depths))
                depth_m = depth_mm / 1000.0
                
                # Validate range (HP60C range is 0.2m - 8m)
                if 0.1 < depth_m < 10.0:
                    result = {'distance': depth_m, 'valid': True, 'x': x, 'y': y}
                    self.get_logger().info(f"📏 Depth at ({x},{y}): {depth_m:.2f}m")
                else:
                    result = {'distance': depth_m, 'valid': False, 'x': x, 'y': y}
            
            # Cache result
            self.depth_query_cache[cache_key] = (time.time(), result)
            return result
            
        except Exception as e:
            self.get_logger().error(f"Depth query error: {e}")
            return {'distance': -1.0, 'valid': False, 'x': x, 'y': y}

    def get_center_distance(self):
        """Get distance to object in center of frame (common use case)"""
        result = self.get_dist(320, 240)  # Center of 640x480
        return result['distance'] if result['valid'] else None

    def find_object_distance(self, target_name: str):
        """
        Find an object in the RGB image and get its distance using depth camera.
        This is how Yahboom combines RGB vision with depth queries.
        
        Args:
            target_name: Name of object to find (e.g., "person", "bottle", "chair", "door")
            
        Returns:
            dict with 'found', 'distance', 'description' or None on error
        """
        if not self.latest_image:
            return {'found': False, 'error': 'No camera image'}
        
        # First, use GPT-4o vision to find the object and get its approximate location
        image_b64, _ = self.image_to_base64(self.latest_image)
        if not image_b64:
            return {'found': False, 'error': 'Image processing failed'}
        
        try:
            # Ask GPT-4o to locate the object
            response = self.client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": """You are a robot vision system. When asked to find an object, respond with JSON containing:
- found: true/false
- x: approximate x pixel coordinate (0-639, where 320 is center)
- y: approximate y pixel coordinate (0-479, where 240 is center)  
- description: brief description of what you see

If the object is not visible, set found to false. Only respond with valid JSON."""},
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": f"Find the {target_name} in this image. Return its approximate pixel coordinates."},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}", "detail": "low"}}
                        ]
                    }
                ],
                max_tokens=100,
                temperature=0.3
            )
            
            result_text = response.choices[0].message.content.strip()
            
            # Parse JSON response
            try:
                # Handle markdown code blocks
                if "```" in result_text:
                    result_text = result_text.split("```")[1]
                    if result_text.startswith("json"):
                        result_text = result_text[4:]
                
                vision_result = json.loads(result_text)
            except:
                # Try to extract JSON
                start = result_text.find('{')
                end = result_text.rfind('}') + 1
                if start >= 0 and end > start:
                    vision_result = json.loads(result_text[start:end])
                else:
                    return {'found': False, 'error': 'Could not parse vision response'}
            
            if not vision_result.get("found", False):
                return {'found': False, 'description': f"Could not find {target_name} in view"}
            
            # Get depth at the detected location
            x = int(vision_result.get("x", 320))
            y = int(vision_result.get("y", 240))
            description = vision_result.get("description", f"Found {target_name}")
            
            depth_result = self.get_dist(x, y)
            
            if depth_result['valid']:
                return {
                    'found': True,
                    'distance': depth_result['distance'],
                    'description': description,
                    'pixel_x': x,
                    'pixel_y': y
                }
            else:
                # Depth not available, but we found it visually
                return {
                    'found': True,
                    'distance': -1,
                    'description': f"{description} (depth unavailable)",
                    'pixel_x': x,
                    'pixel_y': y
                }
                
        except Exception as e:
            self.get_logger().error(f"Object detection error: {e}")
            return {'found': False, 'error': str(e)}

    # === SLAM Functions ===
    
    def start_slam(self):
        """Start SLAM mapping"""
        if self.mapping:
            return "Already mapping!"
        
        self.get_logger().info("🗺️ Starting SLAM...")
        
        try:
            # Launch slam_toolbox
            cmd = [
                "ros2", "launch", "slam_toolbox", "online_sync_launch.py",
                "use_sim_time:=false"
            ]
            
            self.slam_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env={**os.environ, 'ROS_DOMAIN_ID': '62'}
            )
            
            self.mapping = True
            time.sleep(3)  # Give SLAM time to start
            
            self.get_logger().info("✅ SLAM started!")
            self.beep("success")
            return "SLAM mapping started!"
            
        except Exception as e:
            self.get_logger().error(f"❌ SLAM start failed: {e}")
            self.beep("error")
            self.speak("Mapping system failed to start. Check LiDAR sensor.")
            return f"Failed to start mapping: {e}"
    
    def stop_slam(self, map_name=None):
        """Stop SLAM and save map"""
        if not self.mapping:
            return "Not currently mapping"
        
        if map_name is None:
            map_name = f"map_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        map_path = self.maps_dir / map_name
        
        self.get_logger().info(f"💾 Saving map to {map_path}...")
        
        try:
            # Save map using map_saver
            save_cmd = [
                "ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", str(map_path),
                "--ros-args", "-p", "save_map_timeout:=10000"
            ]
            
            result = subprocess.run(
                save_cmd,
                capture_output=True,
                text=True,
                timeout=30,
                env={**os.environ, 'ROS_DOMAIN_ID': '62'}
            )
            
            if result.returncode == 0:
                self.get_logger().info(f"✅ Map saved: {map_path}")
            else:
                self.get_logger().warning(f"Map save warning: {result.stderr}")
            
        except Exception as e:
            self.get_logger().error(f"Map save error: {e}")
        
        # Stop SLAM process
        if self.slam_process:
            self.slam_process.terminate()
            try:
                self.slam_process.wait(timeout=5)
            except:
                self.slam_process.kill()
            self.slam_process = None
        
        self.mapping = False
        
        # Also save discovery log
        self.discovery_log.save()
        
        return f"Map saved as {map_name}"
    
    def get_map_status(self):
        """Get current map statistics"""
        if not self.mapping or self.map_info is None:
            return "No active map. Say 'start mapping' first."
        
        width_m = self.map_info['width'] * self.map_info['resolution']
        height_m = self.map_info['height'] * self.map_info['resolution']
        coverage_pct = getattr(self, 'map_coverage', 0) * 100
        
        summary = self.discovery_log.get_summary()
        
        status = (
            f"Map: {width_m:.1f}m x {height_m:.1f}m, "
            f"{coverage_pct:.0f}% explored. "
            f"Traveled {summary['distance_meters']:.1f}m, "
            f"found {summary['discoveries']} interesting things."
        )
        return status

    # === Isaac VSLAM Functions ===
    
    def start_isaac_vslam(self):
        """Start Isaac ROS Visual SLAM for stereo camera localization.
        
        Requires:
        - OAK-D Pro or RealSense camera with stereo support
        - ros-humble-isaac-ros-visual-slam package installed
        - Jetson GPU for CUDA acceleration
        
        Isaac VSLAM provides:
        - High-frequency visual odometry (200Hz+ on Jetson)
        - Loop closure detection
        - Relocalization in saved maps
        - 3D point cloud landmarks
        """
        if self.vslam_process is not None:
            self.get_logger().warning("Isaac VSLAM already running")
            return True
        
        # Check camera compatibility
        if self.camera_type == CameraType.HP60C:
            self.get_logger().error("❌ HP60C not suitable for VSLAM - use OAK-D Pro or RealSense")
            self.speak("Current camera is not compatible with visual slam. Need OAK-D Pro.")
            return False
        
        if not self.camera_config.left_topic:
            self.get_logger().error("❌ No stereo camera configured for VSLAM")
            return False
        
        self.get_logger().info("🎯 Starting Isaac ROS Visual SLAM...")
        
        try:
            # Build topic remapping for camera
            # Isaac VSLAM expects: /visual_slam/image_0, /visual_slam/image_1, etc.
            remappings = []
            
            if self.camera_type == CameraType.OAK_D_PRO:
                # OAK-D Pro topic remapping
                remappings = [
                    f"/visual_slam/image_0:={self.camera_config.left_topic}",
                    f"/visual_slam/image_1:={self.camera_config.right_topic}",
                    f"/visual_slam/camera_info_0:={self.camera_config.camera_info_left}",
                    f"/visual_slam/camera_info_1:={self.camera_config.camera_info_right}",
                ]
                if self.camera_config.imu_topic:
                    remappings.append(f"/visual_slam/imu:={self.camera_config.imu_topic}")
            
            elif self.camera_type == CameraType.REALSENSE:
                # RealSense topic remapping
                remappings = [
                    f"/visual_slam/image_0:={self.camera_config.left_topic}",
                    f"/visual_slam/image_1:={self.camera_config.right_topic}",
                    f"/visual_slam/camera_info_0:={self.camera_config.camera_info_left}",
                    f"/visual_slam/camera_info_1:={self.camera_config.camera_info_right}",
                ]
                if self.camera_config.imu_topic:
                    remappings.append(f"/visual_slam/imu:={self.camera_config.imu_topic}")
            
            # Launch Isaac VSLAM node
            cmd = [
                "ros2", "launch", "isaac_ros_visual_slam", "isaac_ros_visual_slam.launch.py",
                "num_cameras:=2",
                "enable_imu_fusion:=true" if self.camera_config.imu_topic else "enable_imu_fusion:=false",
                "enable_localization_n_mapping:=true",
                "enable_slam_visualization:=true",
            ]
            
            # Add remappings
            for remap in remappings:
                cmd.append(f"--remap")
                cmd.append(remap)
            
            self.get_logger().info(f"Launch command: {' '.join(cmd[:5])}...")
            
            self.vslam_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env={**os.environ, 'ROS_DOMAIN_ID': '62'}
            )
            
            # Wait for VSLAM to initialize
            self.get_logger().info("Waiting for Isaac VSLAM to initialize...")
            
            for i in range(30):  # 30 second timeout
                time.sleep(1)
                # Check if we're getting VSLAM odometry
                if self.vslam_tracking:
                    self.vslam_available = True
                    self.get_logger().info("✅ Isaac VSLAM is tracking!")
                    self.beep("success")
                    return True
                self.get_logger().info(f"Waiting for VSLAM tracking... ({i+1}/30)")
            
            self.get_logger().error("❌ Isaac VSLAM failed to start tracking")
            self.stop_isaac_vslam()
            return False
            
        except FileNotFoundError:
            self.get_logger().error("❌ Isaac VSLAM not installed! Run: sudo apt install ros-humble-isaac-ros-visual-slam")
            self.speak("Isaac visual slam is not installed.")
            return False
        except Exception as e:
            self.get_logger().error(f"❌ Isaac VSLAM start failed: {e}")
            return False
    
    def stop_isaac_vslam(self):
        """Stop Isaac VSLAM"""
        if self.vslam_process:
            self.get_logger().info("Stopping Isaac VSLAM...")
            self.vslam_process.terminate()
            try:
                self.vslam_process.wait(timeout=5)
            except:
                self.vslam_process.kill()
            self.vslam_process = None
            self.vslam_available = False
            self.vslam_tracking = False
            self.get_logger().info("Isaac VSLAM stopped")
    
    def save_vslam_map(self, map_name=None):
        """Save Isaac VSLAM map for later relocalization.
        
        The VSLAM map contains visual landmarks and can be used for:
        - Relocalization in previously mapped areas
        - Loop closure detection
        """
        if not self.vslam_available:
            self.get_logger().warning("VSLAM not running - cannot save map")
            return False
        
        if map_name is None:
            map_name = f"vslam_map_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        map_path = self.maps_dir / f"{map_name}.vslam"
        
        try:
            # Call Isaac VSLAM save_map service
            from isaac_ros_visual_slam_interfaces.srv import FilePath
            
            if self.vslam_save_map_client is None:
                self.vslam_save_map_client = self.create_client(
                    FilePath, '/visual_slam/save_map')
            
            if not self.vslam_save_map_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("VSLAM save_map service not available")
                return False
            
            request = FilePath.Request()
            request.file_path = str(map_path)
            
            future = self.vslam_save_map_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.result() is not None and future.result().success:
                self.get_logger().info(f"✅ VSLAM map saved: {map_path}")
                return True
            else:
                self.get_logger().error("Failed to save VSLAM map")
                return False
                
        except ImportError:
            self.get_logger().error("Isaac VSLAM interfaces not available")
            return False
        except Exception as e:
            self.get_logger().error(f"VSLAM map save error: {e}")
            return False
    
    def load_vslam_map(self, map_path):
        """Load a previously saved VSLAM map for relocalization."""
        if not self.vslam_available:
            self.get_logger().warning("VSLAM not running - start it first")
            return False
        
        try:
            from isaac_ros_visual_slam_interfaces.srv import FilePath
            
            if self.vslam_load_map_client is None:
                self.vslam_load_map_client = self.create_client(
                    FilePath, '/visual_slam/load_map')
            
            if not self.vslam_load_map_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("VSLAM load_map service not available")
                return False
            
            request = FilePath.Request()
            request.file_path = str(map_path)
            
            future = self.vslam_load_map_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.result() is not None and future.result().success:
                self.get_logger().info(f"✅ VSLAM map loaded: {map_path}")
                return True
            else:
                self.get_logger().error("Failed to load VSLAM map")
                return False
                
        except ImportError:
            self.get_logger().error("Isaac VSLAM interfaces not available")
            return False
        except Exception as e:
            self.get_logger().error(f"VSLAM map load error: {e}")
            return False
    
    def get_vslam_status(self):
        """Get VSLAM status and tracking quality."""
        if not self.vslam_available:
            return "VSLAM not running. Use OAK-D Pro camera and say 'start visual slam'."
        
        if not self.vslam_tracking:
            return "VSLAM running but not tracking. Check camera view."
        
        path_length = len(self.vslam_path)
        if path_length > 1:
            # Calculate total VSLAM path distance
            total_dist = 0
            for i in range(1, len(self.vslam_path)):
                p1, p2 = self.vslam_path[i-1], self.vslam_path[i]
                dx = p2['x'] - p1['x']
                dy = p2['y'] - p1['y']
                dz = p2['z'] - p1['z']
                total_dist += math.sqrt(dx*dx + dy*dy + dz*dz)
            
            return f"VSLAM tracking! Path: {total_dist:.2f}m, {path_length} poses recorded."
        
        return "VSLAM tracking, collecting visual landmarks."
    
    def _detect_camera(self):
        """Auto-detect which camera is connected based on available topics."""
        self.get_logger().info("Auto-detecting camera...")
        
        # Get list of available topics
        topic_names = [name for name, _ in self.get_topic_names_and_types()]
        
        # Check for OAK-D Pro topics
        if "/oak/rgb/image_raw" in topic_names or "/oak/left/image_rect" in topic_names:
            self.get_logger().info("📷 Detected: OAK-D Pro")
            return CameraType.OAK_D_PRO
        
        # Check for RealSense topics
        if "/camera/color/image_raw" in topic_names or "/camera/infra1/image_rect_raw" in topic_names:
            self.get_logger().info("📷 Detected: Intel RealSense")
            return CameraType.REALSENSE
        
        # Check for HP60C topics
        if "/ascamera_hp60c" in str(topic_names):
            self.get_logger().info("📷 Detected: Angstrong HP60C")
            return CameraType.HP60C
        
        # Default to HP60C (current robot camera)
        self.get_logger().info("📷 Defaulting to HP60C (no other camera detected)")
        return CameraType.HP60C

    # === Audio Functions ===
    
    def listen(self):
        """Record speech"""
        if self.speaking:
            return None
        
        self.listening = True
        p = pyaudio.PyAudio()
        
        try:
            stream = p.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                input_device_index=self.mic_index,
                frames_per_buffer=self.chunk_size
            )
        except Exception as e:
            self.get_logger().error(f"Mic error: {e}")
            p.terminate()
            self.listening = False
            return None
        
        frames = []
        silent_chunks = 0
        max_silent = int(self.silence_duration * self.sample_rate / self.chunk_size)
        recording = False
        
        try:
            while self.running and not self.speaking:
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                level = np.abs(np.frombuffer(data, dtype=np.int16)).mean()
                
                if level > self.silence_threshold:
                    if not recording:
                        self.get_logger().info("🎤 Listening...")
                        recording = True
                    frames.append(data)
                    silent_chunks = 0
                elif recording:
                    frames.append(data)
                    silent_chunks += 1
                    if silent_chunks >= max_silent:
                        break
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()
            self.listening = False
        
        duration = len(frames) * self.chunk_size / self.sample_rate
        if duration < self.min_audio_duration:
            return None
        
        # Save and resample
        temp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
        with wave.open(temp.name, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(b''.join(frames))
        
        if HAS_SCIPY:
            try:
                rate, data = wavfile.read(temp.name)
                num_samples = int(len(data) * self.whisper_rate / rate)
                resampled = signal.resample(data, num_samples).astype(np.int16)
                
                resampled_file = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
                wavfile.write(resampled_file.name, self.whisper_rate, resampled)
                os.unlink(temp.name)
                return resampled_file.name
            except Exception as e:
                self.get_logger().warning(f"Resample failed: {e}")
        
        return temp.name

    def transcribe(self, audio_file):
        """Transcribe audio using Whisper"""
        try:
            with open(audio_file, "rb") as f:
                result = self.client.audio.transcriptions.create(
                    model="whisper-1", file=f, language="en"
                )
            text = result.text.strip()
            self.get_logger().info(f"🗣️ You said: {text}")
            return text
        except Exception as e:
            self.get_logger().error(f"Transcription error: {e}")
            return None
        finally:
            try:
                os.unlink(audio_file)
            except:
                pass

    def beep(self, pattern="warning"):
        """Play a beep pattern for alerts when TTS fails.
        Patterns: 'warning' (2 beeps), 'error' (3 fast beeps), 'success' (1 beep), 'attention' (long beep)
        Uses ffmpeg to generate short tones for faster playback.
        """
        def play_tone(freq=800, duration=0.15):
            """Play a short tone using ffmpeg and aplay"""
            try:
                # Generate a short sine wave and play it
                cmd = f"ffmpeg -f lavfi -i 'sine=frequency={freq}:duration={duration}' -f wav - 2>/dev/null | aplay -D plughw:0,0 -q 2>/dev/null"
                subprocess.run(cmd, shell=True, timeout=2)
            except Exception:
                pass
        
        try:
            if pattern == "warning":
                # Two short beeps
                play_tone(800, 0.15)
                time.sleep(0.1)
                play_tone(800, 0.15)
            elif pattern == "error":
                # Three fast high beeps
                for _ in range(3):
                    play_tone(1200, 0.1)
                    time.sleep(0.05)
            elif pattern == "success":
                # One pleasant beep
                play_tone(600, 0.2)
            elif pattern == "attention":
                # Long low beep
                play_tone(400, 0.5)
        except Exception as e:
            self.get_logger().error(f"Beep failed: {e}")

    def speak(self, text, beep_on_fail=True):
        """Text to speech with boosted volume. Falls back to beep if TTS fails."""
        if not text or self.speaking:
            return
        
        self.speaking = True
        self.get_logger().info(f"🔊 {text}")
        
        try:
            response = self.client.audio.speech.create(
                model="tts-1", voice="nova", input=text, response_format="mp3"
            )
            
            temp_file = tempfile.NamedTemporaryFile(suffix=".mp3", delete=False)
            temp_file.write(response.content)
            temp_file.close()
            
            wav_file = temp_file.name.replace(".mp3", ".wav")
            # Boost volume by 3x (about 10dB) with ffmpeg filter
            result = subprocess.run(
                ["ffmpeg", "-y", "-i", temp_file.name, "-af", "volume=3.0", "-ar", "48000", "-ac", "1", wav_file],
                capture_output=True, timeout=10
            )
            if result.returncode != 0:
                raise Exception(f"ffmpeg failed: {result.stderr.decode()}")
            
            result = subprocess.run(
                ["aplay", "-D", "plughw:0,0", wav_file],
                capture_output=True, timeout=30
            )
            if result.returncode != 0:
                raise Exception(f"aplay failed: {result.stderr.decode()}")
            
            os.unlink(wav_file)
            os.unlink(temp_file.name)
        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")
            if beep_on_fail:
                self.beep("warning")
        finally:
            self.speaking = False

    # === Vision Functions ===
    
    def enhance_image(self, cv_image):
        """Enhance dark camera images"""
        if cv_image is None:
            return None
        
        # Apply brightness enhancement
        enhanced = cv2.convertScaleAbs(cv_image, alpha=15.0, beta=30)
        return enhanced
    
    def image_to_base64(self, image_msg):
        """Convert ROS Image to base64 with enhancement"""
        try:
            if HAS_CV:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
                cv_image = self.enhance_image(cv_image)
                cv_image = cv2.resize(cv_image, (640, 480))
                _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                return base64.b64encode(buffer).decode('utf-8'), cv_image
            return None, None
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")
            return None, None

    def observe(self):
        """Look and describe surroundings"""
        if not self.latest_image:
            return "Camera not available.", None
        
        image_b64, cv_image = self.image_to_base64(self.latest_image)
        if not image_b64:
            return "Image processing failed.", None
        
        self.get_logger().info("📷 Looking around...")
        
        try:
            response = self.client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": "You are a mapping robot. Describe what you see concisely, noting landmarks, obstacles, open spaces, doors, and anything useful for navigation. If dark, mention it briefly."},
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": "What do you see? Note anything useful for mapping."},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}", "detail": "low"}}
                        ]
                    }
                ],
                max_tokens=100,
                temperature=0.7
            )
            
            observation = response.choices[0].message.content.strip()
            self.get_logger().info(f"👁️ {observation}")
            
            # Log discovery
            discovery_num = self.discovery_log.add_discovery(
                observation,
                self.current_position['x'],
                self.current_position['y'],
                self.current_position['theta'],
                cv_image
            )
            
            return observation, cv_image
        except Exception as e:
            self.get_logger().error(f"Vision error: {e}")
            return "Vision processing failed.", None

    # === Movement Functions ===
    
    def emergency_stop(self):
        """Immediate stop - publish multiple times to ensure delivery"""
        twist = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.02)
        self.get_logger().warning("🚨 EMERGENCY STOP!")

    def move(self, linear, angular, duration):
        """Move with robust obstacle avoidance"""
        twist = Twist()
        start_time = time.time()
        rate_hz = 20
        
        while (time.time() - start_time) < duration and self.running:
            # CRITICAL: Emergency stop check - anything dangerously close
            if hasattr(self, 'emergency_stop_triggered') and self.emergency_stop_triggered:
                self.get_logger().error("🚨 EMERGENCY: Object at {:.2f}m! Stopping!".format(
                    self.obstacle_distances.get("front", 99)))
                self.emergency_stop()
                self.emergency_stop_triggered = False
                # Back up slightly
                self.get_logger().info("↩️ Backing up...")
                backup_twist = Twist()
                backup_twist.linear.x = -0.1
                for _ in range(20):  # 1 second backup
                    self.cmd_vel_pub.publish(backup_twist)
                    time.sleep(0.05)
                self.emergency_stop()
                return
            
            # Safety checks - check multiple front sectors
            front_dist = self.obstacle_distances.get("front", 10)
            front_wide = self.obstacle_distances.get("front_wide", 10)
            fl = self.obstacle_distances.get("front_left", 10)
            fr = self.obstacle_distances.get("front_right", 10)
            min_front = min(front_dist, front_wide, fl, fr)
            
            if linear > 0:
                # Check for obstacle stop distance
                if min_front < self.min_obstacle_dist:
                    self.get_logger().warning(f"🛑 Obstacle at {min_front:.2f}m (F:{front_dist:.2f} FL:{fl:.2f} FR:{fr:.2f}) - stopping!")
                    break
            
            if linear < 0 and self.obstacles.get("back", False):
                self.get_logger().warning("🛑 Obstacle behind - stopping!")
                break
            
            # Speed control based on proximity
            actual_linear = linear
            if linear > 0:
                if min_front < self.slow_dist:
                    # Proportional slowdown
                    speed_factor = max(0.3, (min_front - self.min_obstacle_dist) / (self.slow_dist - self.min_obstacle_dist))
                    actual_linear = self.slow_speed * speed_factor
                    self.get_logger().debug(f"Slowing: dist={min_front:.2f}m, factor={speed_factor:.2f}")
            
            # Ackerman constraint: can't turn in place
            if abs(angular) > 0.1 and abs(actual_linear) < 0.05:
                actual_linear = 0.05 * (1 if linear >= 0 else -1)
            
            twist.linear.x = float(actual_linear)
            twist.angular.z = float(angular)
            self.cmd_vel_pub.publish(twist)
            
            time.sleep(1.0 / rate_hz)
        
        # Stop
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def choose_exploration_direction(self):
        """Smart direction for mapping exploration with safety margins"""
        front = self.obstacle_distances.get("front", 10)
        front_wide = self.obstacle_distances.get("front_wide", 10)
        front_left = self.obstacle_distances.get("front_left", 10)
        front_right = self.obstacle_distances.get("front_right", 10)
        left = self.obstacle_distances.get("left", 10)
        right = self.obstacle_distances.get("right", 10)
        
        # Use minimum of front sensors for safety
        min_front = min(front, front_wide, front_left, front_right)
        
        self.get_logger().debug(f"Explore check: F:{front:.2f} FW:{front_wide:.2f} FL:{front_left:.2f} FR:{front_right:.2f}")
        
        # If anything close, turn away from it
        if min_front < self.min_obstacle_dist:
            # Too close - need to turn sharply
            if front_left > front_right:
                self.get_logger().info(f"↰ Turning left (obstacle at {min_front:.2f}m)")
                return (0.0, self.angular_speed * 1.2)  # Sharper turn, slower
            else:
                self.get_logger().info(f"↱ Turning right (obstacle at {min_front:.2f}m)")
                return (0.0, -self.angular_speed * 1.2)
        
        # Prefer unexplored directions (sides with more space)
        if min_front > self.slow_dist:
            # Clear ahead - mostly straight with slight variation
            if random.random() < 0.15:
                # Occasional turn to explore sides
                if left > right:
                    return (self.linear_speed * 0.7, self.angular_speed * 0.3)
                else:
                    return (self.linear_speed * 0.7, -self.angular_speed * 0.3)
            return (self.linear_speed, random.uniform(-0.05, 0.05))
        
        # Need to turn - pick the clearer direction
        if front_left > front_right:
            return (self.slow_speed * 0.5, self.angular_speed)
        else:
            return (self.slow_speed * 0.5, -self.angular_speed)

    # === Brain Functions ===
    
    def get_context(self):
        """Current context for GPT"""
        summary = self.discovery_log.get_summary()
        
        context = f"""
Status:
- Mapping: {'active' if self.mapping else 'not started'}
- Exploring: {'yes' if self.exploring else 'no'}
- Position: ({self.current_position['x']:.1f}, {self.current_position['y']:.1f})
- Distance traveled: {summary['distance_meters']:.1f}m
- Discoveries: {summary['discoveries']}
- LiDAR: front={self.obstacle_distances.get('front', 0):.1f}m, left={self.obstacle_distances.get('left', 0):.1f}m, right={self.obstacle_distances.get('right', 0):.1f}m
"""
        if self.map_info:
            context += f"- Map size: {self.map_info['width']}x{self.map_info['height']} cells\n"
        
        return context

    def think(self, user_input):
        """Process with GPT-4o"""
        context = self.get_context()
        
        messages = [
            {"role": "system", "content": self.system_prompt + "\n\n" + context}
        ]
        
        for msg in self.conversation[-4:]:
            messages.append(msg)
        
        messages.append({"role": "user", "content": user_input})
        
        try:
            response = self.client.chat.completions.create(
                model="gpt-4o",
                messages=messages,
                max_tokens=150,
                temperature=0.7
            )
            
            reply = response.choices[0].message.content.strip()
            
            self.conversation.append({"role": "user", "content": user_input})
            self.conversation.append({"role": "assistant", "content": reply})
            if len(self.conversation) > 10:
                self.conversation = self.conversation[-6:]
            
            return reply
        except Exception as e:
            self.get_logger().error(f"GPT error: {e}")
            return '{"action": "speak", "speech": "Sorry, I had trouble processing that."}'

    def execute(self, response_json):
        """Execute action from JSON"""
        try:
            data = json.loads(response_json)
        except json.JSONDecodeError:
            start = response_json.find('{')
            end = response_json.rfind('}') + 1
            if start >= 0 and end > start:
                try:
                    data = json.loads(response_json[start:end])
                except:
                    return
            else:
                return
        
        action = data.get("action", "speak")
        speech = data.get("speech", "")
        
        if action == "move":
            linear = max(-0.2, min(0.2, float(data.get("linear", 0))))
            angular = max(-0.3, min(0.3, float(data.get("angular", 0))))
            duration = min(5.0, float(data.get("duration", 2)))
            
            if speech:
                self.speak(speech)
            self.move(linear, angular, duration)
            
        elif action == "look":
            if speech:
                self.speak(speech)
            observation, _ = self.observe()
            if observation:
                self.speak(observation)
        
        elif action == "get_dist":
            target = data.get("target", "object")
            if speech:
                self.speak(speech)
            
            result = self.find_object_distance(target)
            if result.get('found'):
                distance = result.get('distance', -1)
                description = result.get('description', f'Found {target}')
                if distance > 0:
                    self.speak(f"{description}. It's about {distance:.1f} meters away.")
                else:
                    self.speak(f"{description}.")
            else:
                error_msg = result.get('description', result.get('error', f'Could not find {target}'))
                self.speak(error_msg)
                
        elif action == "start_mapping":
            result = self.start_slam()
            self.speak(speech if speech else result)
            
        elif action == "stop_mapping":
            map_name = data.get("map_name")
            result = self.stop_slam(map_name)
            self.speak(speech if speech else result)
            
        elif action == "explore":
            if speech:
                self.speak(speech)
            if not self.mapping:
                self.speak("Starting mapping first...")
                self.start_slam()
                time.sleep(2)
            self.start_exploration()
            
        elif action == "navigate":
            x = data.get("x", 0)
            y = data.get("y", 0)
            if speech:
                self.speak(speech)
            if not self.mapping:
                self.speak("I need a map first. Starting mapping...")
                self.start_slam()
                time.sleep(2)
            self.navigate_to(x, y)
            
        elif action == "stop":
            self.stop_exploration()
            self.cancel_navigation()
            if speech:
                self.speak(speech)
                
        elif action == "status":
            summary = self.discovery_log.get_summary()
            status = f"I've been exploring for {summary['duration_minutes']:.1f} minutes, "
            status += f"traveled {summary['distance_meters']:.1f} meters, "
            status += f"and made {summary['discoveries']} discoveries."
            if self.mapping:
                status += " Currently mapping."
            self.speak(status)
            
        elif action == "map_status":
            status = self.get_map_status()
            self.speak(status)
        
        elif action == "start_vslam":
            if speech:
                self.speak(speech)
            if self.start_isaac_vslam():
                self.speak("Visual SLAM is now tracking!")
            else:
                self.speak("Could not start visual SLAM. Check camera connection.")
        
        elif action == "stop_vslam":
            self.stop_isaac_vslam()
            if speech:
                self.speak(speech)
            else:
                self.speak("Visual SLAM stopped.")
        
        elif action == "vslam_status":
            status = self.get_vslam_status()
            self.speak(status)
        
        elif action == "save_vslam_map":
            map_name = data.get("map_name")
            if speech:
                self.speak(speech)
            if self.save_vslam_map(map_name):
                self.speak(f"Visual map saved as {map_name}")
            else:
                self.speak("Could not save visual map.")
            
        elif action == "speak" and speech:
            self.speak(speech)

    # === Exploration ===
    
    def exploration_loop(self):
        """Autonomous exploration using Nav2 frontier-based navigation.
        Aggressively explores to discover new rooms and go through doorways.
        """
        last_observation = time.time() - self.look_interval + 5
        consecutive_failures = 0
        max_failures = 5
        no_frontier_count = 0
        exploration_start = time.time()
        
        self.get_logger().info("🚀 Starting adventurous exploration!")
        self.speak("Starting exploration! I'll look for new rooms and doorways.")
        
        # Start Nav2 if not running
        if not self.nav2_available:
            self.get_logger().info("Starting Nav2 for exploration...")
            if not self.start_nav2():
                self.get_logger().warning("Nav2 not ready - using reactive mode while waiting")
                # Try reactive exploration while Nav2 initializes
                self._try_nav2_with_reactive_fallback()
                return
        
        while self.running and self.exploring:
            # Periodic observation to see interesting things
            if time.time() - last_observation >= self.look_interval:
                # Stop for observation
                if self.navigating:
                    self.cancel_navigation()
                time.sleep(0.5)
                
                observation, _ = self.observe()
                if observation:
                    self.speak(observation)
                last_observation = time.time()
                
                while self.speaking and self.running:
                    time.sleep(0.1)
            
            # If not currently navigating, find next frontier
            if not self.navigating and self.exploring:
                frontier = self.choose_frontier()
                
                if frontier is None:
                    no_frontier_count += 1
                    self.get_logger().warning(f"No frontiers found (attempt {no_frontier_count})")
                    
                    if no_frontier_count >= 3:
                        # Try a random walk to discover new areas
                        self.get_logger().info("🎲 No frontiers - doing random exploration walk")
                        self._random_exploration_walk()
                        no_frontier_count = 0
                        continue
                    
                    # Wait and try again - map might be updating
                    time.sleep(2)
                    continue
                
                no_frontier_count = 0
                fx, fy = frontier
                dist = math.sqrt((fx - self.current_position['x'])**2 + 
                                (fy - self.current_position['y'])**2)
                self.get_logger().info(f"🎯 Exploring frontier at ({fx:.2f}, {fy:.2f}) - {dist:.1f}m away")
                
                # Navigate to frontier
                if not self.navigate_to(fx, fy):
                    consecutive_failures += 1
                    self.get_logger().warning(f"Navigation failed ({consecutive_failures}/{max_failures})")
                    
                    if consecutive_failures >= max_failures:
                        self.get_logger().warning("Many failures - clearing costmaps and doing random walk")
                        self._clear_costmaps()
                        self._random_exploration_walk()
                        consecutive_failures = 0
                else:
                    consecutive_failures = 0
            
            # Wait a bit before checking again
            time.sleep(0.5)
            
            # Check navigation progress
            if self.navigating and self.nav_feedback:
                dist = self.nav_feedback.get('distance_remaining', 0)
                elapsed = self.nav_feedback.get('time_elapsed', 0)
                
                # If stuck (no progress for 30s), try something else
                if elapsed > 30 and dist > 0.5:
                    self.get_logger().warning(f"Stuck! {elapsed}s elapsed, still {dist:.2f}m away")
                    self.cancel_navigation()
                    self._clear_costmaps()
                    consecutive_failures += 1
            
            # Log periodic progress
            if int(time.time() - exploration_start) % 60 == 0:
                summary = self.discovery_log.get_summary()
                self.get_logger().info(f"📊 Explored {summary['distance_meters']:.1f}m, {summary['discoveries']} discoveries")
        
        # Stop everything
        self.cancel_navigation()
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Exploration ended")

    def _random_exploration_walk(self):
        """Do a smart exploration walk - prioritize detected doorways/gaps!"""
        self.get_logger().info("🎲 Smart exploration walk to find new areas...")
        
        # FIRST: Check for detected doorways/gaps - these are the best targets!
        if self.detected_gaps:
            best_gap = max(self.detected_gaps, key=lambda g: g['width'])
            self.get_logger().info(f"🚪 Found doorway! angle={best_gap['angle']:.0f}°, width={best_gap['width']:.1f}m, dist={best_gap['distance']:.1f}m")
            
            # Turn toward the gap
            twist = Twist()
            gap_angle = best_gap['angle']  # Already in degrees from front
            
            if abs(gap_angle) > 10:  # Need to turn
                twist.angular.z = 0.3 if gap_angle > 0 else -0.3
                turn_time = abs(gap_angle) / 30  # Rough turn time estimate
                turn_time = min(turn_time, 3.0)  # Cap at 3s
                
                self.get_logger().info(f"Turning {gap_angle:.0f}° toward doorway...")
                for _ in range(int(turn_time * 20)):
                    if not self.exploring or not self.running:
                        break
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(0.05)
            
            # Drive through the gap
            twist = Twist()
            twist.linear.x = 0.15  # Slightly faster for doorways
            drive_time = min(best_gap['distance'] / 0.15 + 2.0, 8.0)  # Drive through + extra
            
            self.get_logger().info(f"🚀 Driving through doorway for {drive_time:.1f}s")
            self.speak("I see an opening, going through!")
            
            for _ in range(int(drive_time * 20)):
                if not self.exploring or not self.running:
                    break
                # Check for obstacles but be more tolerant for doorways
                front = self.obstacle_distances.get("front", 10)
                if front < 0.35:  # Tighter tolerance for doorways
                    self.get_logger().warning("Too close - stopping")
                    break
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)
            
            # Stop
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        # FALLBACK: Look for the direction with most open space
        best_direction = None
        best_distance = 0
        
        directions = [
            ("front", self.obstacle_distances.get("front", 0)),
            ("front_left", self.obstacle_distances.get("front_left", 0)),
            ("front_right", self.obstacle_distances.get("front_right", 0)),
            ("left", self.obstacle_distances.get("left", 0)),
            ("right", self.obstacle_distances.get("right", 0)),
        ]
        
        for name, dist in directions:
            if dist > best_distance:
                best_distance = dist
                best_direction = name
        
        self.get_logger().info(f"Best direction: {best_direction} ({best_distance:.2f}m clear)")
        
        # Turn toward best direction
        twist = Twist()
        if best_direction == "left" or best_direction == "front_left":
            twist.angular.z = 0.4
            turn_time = 1.5
        elif best_direction == "right" or best_direction == "front_right":
            twist.angular.z = -0.4
            turn_time = 1.5
        else:
            turn_time = 0
        
        # Turn
        if turn_time > 0:
            for _ in range(int(turn_time * 20)):
                if not self.exploring or not self.running:
                    break
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)
        
        # Drive forward
        twist = Twist()
        twist.linear.x = 0.12
        drive_time = min(best_distance / 0.12, 5.0)  # Drive for distance or max 5s
        
        self.get_logger().info(f"Driving forward for {drive_time:.1f}s")
        for _ in range(int(drive_time * 20)):
            if not self.exploring or not self.running:
                break
            # Check for obstacles
            front = self.obstacle_distances.get("front", 10)
            if front < 0.5:
                self.get_logger().warning("Obstacle detected during random walk")
                break
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        
        # Stop
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def _try_nav2_with_reactive_fallback(self):
        """Try to use Nav2, fall back to reactive if it fails."""
        # Try starting Nav2 in background
        nav2_thread = threading.Thread(target=self.start_nav2, daemon=True)
        nav2_thread.start()
        
        # Use reactive exploration while waiting
        start = time.time()
        while self.running and self.exploring and time.time() - start < 30:
            if self.nav2_available:
                self.get_logger().info("Nav2 ready! Switching to Nav2 exploration")
                self.exploration_loop()
                return
            
            # Do reactive exploration step
            self._reactive_exploration_step()
            time.sleep(0.1)
        
        if self.nav2_available:
            self.exploration_loop()
        else:
            self.get_logger().warning("Nav2 didn't start - continuing with reactive exploration")
            self._reactive_exploration_loop()

    def _reactive_exploration_step(self):
        """Single step of reactive exploration."""
        front = self.obstacle_distances.get("front", 10)
        front_wide = self.obstacle_distances.get("front_wide", 10)
        fl = self.obstacle_distances.get("front_left", 10)
        fr = self.obstacle_distances.get("front_right", 10)
        min_front = min(front, front_wide, fl, fr)
        
        twist = Twist()
        
        if min_front < self.min_obstacle_dist:
            # Turn away from obstacle
            left = self.obstacle_distances.get("left", 10)
            right = self.obstacle_distances.get("right", 10)
            twist.angular.z = 0.4 if left > right else -0.4
        else:
            # Go forward with slight random variation
            twist.linear.x = 0.1
            twist.angular.z = random.uniform(-0.1, 0.1)
        
        self.cmd_vel_pub.publish(twist)

    def _reactive_exploration_loop(self):
        """Fallback reactive exploration (no Nav2) - now with doorway detection!"""
        last_observation = time.time() - self.look_interval + 5
        stuck_count = 0
        last_gap_attempt = 0
        
        self.get_logger().info("🚀 Reactive exploration with doorway detection")
        
        while self.running and self.exploring:
            # Periodic observation
            if time.time() - last_observation >= self.look_interval:
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.5)
                
                observation, _ = self.observe()
                if observation:
                    self.speak(observation)
                last_observation = time.time()
                
                while self.speaking and self.running:
                    time.sleep(0.1)
            
            # PRIORITY 1: Check for doorways/gaps and go through them!
            if self.detected_gaps and time.time() - last_gap_attempt > 10:
                best_gap = max(self.detected_gaps, key=lambda g: g['width'])
                if best_gap['width'] >= 0.7 and best_gap['distance'] < 5.0:  # Good doorway!
                    self.get_logger().info(f"🚪 Doorway spotted! Going through...")
                    last_gap_attempt = time.time()
                    self._random_exploration_walk()  # This now prioritizes gaps
                    continue
            
            # Check if we need to turn away from obstacle FIRST
            front = self.obstacle_distances.get("front", 10)
            front_wide = self.obstacle_distances.get("front_wide", 10)
            fl = self.obstacle_distances.get("front_left", 10)
            fr = self.obstacle_distances.get("front_right", 10)
            min_front = min(front, front_wide, fl, fr)
            
            if min_front < self.min_obstacle_dist:
                # Obstacle too close - turn away first!
                stuck_count += 1
                self.get_logger().warning(f"🔄 Obstacle at {min_front:.2f}m - turning away (attempt {stuck_count})")
                
                # Choose turn direction based on which side is clearer
                left = self.obstacle_distances.get("left", 10)
                right = self.obstacle_distances.get("right", 10)
                
                if stuck_count > 5:
                    # Really stuck - back up first
                    self.get_logger().warning("↩️ Stuck! Backing up...")
                    twist = Twist()
                    twist.linear.x = -0.1
                    for _ in range(30):  # 1.5 seconds
                        self.cmd_vel_pub.publish(twist)
                        time.sleep(0.05)
                    stuck_count = 0
                
                # Turn in place toward clearer side
                twist = Twist()
                twist.linear.x = 0.0  # No forward motion
                if left > right or fl > fr:
                    twist.angular.z = self.angular_speed * 1.5  # Turn left
                    self.get_logger().info(f"↰ Turning left (L:{left:.2f}m > R:{right:.2f}m)")
                else:
                    twist.angular.z = -self.angular_speed * 1.5  # Turn right
                    self.get_logger().info(f"↱ Turning right (R:{right:.2f}m > L:{left:.2f}m)")
                
                # Turn for a bit
                for _ in range(20):  # 1 second of turning
                    if not self.exploring or not self.running:
                        break
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(0.05)
                
                # Stop and re-check
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
                continue  # Re-evaluate before moving forward
            
            # Clear ahead - reset stuck counter and move forward
            stuck_count = 0
            
            # Move forward
            if self.exploring and self.running:
                linear, angular = self.choose_exploration_direction()
                self.move(linear, angular, 2.0)
            
            time.sleep(0.05)
        
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Reactive exploration ended")

    def _clear_costmaps(self):
        """Clear Nav2 costmaps"""
        try:
            if self.clear_global_costmap.wait_for_service(timeout_sec=1.0):
                self.clear_global_costmap.call_async(ClearEntireCostmap.Request())
            if self.clear_local_costmap.wait_for_service(timeout_sec=1.0):
                self.clear_local_costmap.call_async(ClearEntireCostmap.Request())
            self.get_logger().info("Costmaps cleared")
        except Exception as e:
            self.get_logger().warning(f"Failed to clear costmaps: {e}")

    def start_exploration(self):
        if self.exploring:
            return
        self.exploring = True
        self.stuck_counter = 0
        self.explore_thread = threading.Thread(target=self.exploration_loop, daemon=True)
        self.explore_thread.start()
        self.get_logger().info("🚀 Exploration started!")

    def stop_exploration(self):
        self.exploring = False
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Exploration stopped")

    # === Voice Loop ===
    
    def voice_loop(self):
        """Voice input thread"""
        while self.running:
            if self.speaking:
                time.sleep(0.1)
                continue
            
            audio_file = self.listen()
            if not audio_file:
                continue
            
            text = self.transcribe(audio_file)
            if not text or len(text) < 2:
                continue
            
            response = self.think(text)
            self.get_logger().info(f"🤖 {response}")
            self.execute(response)

    # === Sensor Monitoring ===
    
    def _sensor_monitor_loop(self):
        """Monitor sensors and alert on failures."""
        last_scan_time = time.time()
        last_odom_time = time.time()
        last_image_time = time.time()
        
        scan_warned = False
        odom_warned = False
        image_warned = False
        
        while self.running:
            time.sleep(2)  # Check every 2 seconds
            
            now = time.time()
            
            # Check LiDAR (critical for safety)
            if self.latest_scan:
                last_scan_time = now
                scan_warned = False
            elif now - last_scan_time > 5 and not scan_warned:
                self.get_logger().error("⚠️ LiDAR data lost!")
                self.beep("error")
                if self.exploring:
                    self.speak("Warning! LiDAR sensor lost. Stopping for safety.")
                    self.stop_exploration()
                scan_warned = True
            
            # Check odometry
            if self.latest_odom:
                last_odom_time = now
                odom_warned = False
            elif now - last_odom_time > 5 and not odom_warned:
                self.get_logger().warning("⚠️ Odometry data lost")
                odom_warned = True
            
            # Check camera (less critical)
            if self.latest_image:
                last_image_time = now
                image_warned = False
            elif now - last_image_time > 10 and not image_warned:
                self.get_logger().warning("⚠️ Camera data lost")
                image_warned = True
            
            # Clear old data to detect fresh updates
            self.latest_scan = None
            self.latest_odom = None
            # Don't clear image as frequently

    # === Main ===
    
    def run(self):
        """Main entry"""
        print("\n" + "="*60)
        print("🗺️  VOICE-CONTROLLED MAPPING EXPLORER")
        print("="*60)
        print("Commands:")
        print("  - 'Start mapping' - Begin SLAM")
        print("  - 'Start exploring' - Auto explore & map")
        print("  - 'What do you see?' - Look around")
        print("  - 'Stop' / 'Save the map'")
        print("  - 'Status' / 'Map status'")
        print("="*60 + "\n")
        
        # Wait for sensors with status reporting (30 second timeout for slow hardware)
        self.get_logger().info("Waiting for sensors (up to 30s)...")
        start_wait = time.time()
        while (time.time() - start_wait) < 30:
            rclpy.spin_once(self, timeout_sec=0.5)
            # Check if we have all important sensors
            if self.latest_scan and self.latest_odom:
                self.get_logger().info("Core sensors ready!")
                break
            if self.latest_scan:
                self.get_logger().info(f"  LiDAR ready, waiting for odom/camera... ({int(30 - (time.time() - start_wait))}s)")
        
        sensors = []
        missing_sensors = []
        
        if self.latest_image:
            sensors.append("camera")
        else:
            missing_sensors.append("camera")
        
        if self.latest_scan:
            sensors.append("LiDAR")
        else:
            missing_sensors.append("LiDAR")
        
        if self.latest_odom:
            sensors.append("odometry")
        else:
            missing_sensors.append("odometry")
        
        # Report sensor status
        if missing_sensors:
            self.get_logger().error(f"❌ Missing sensors: {', '.join(missing_sensors)}")
            self.beep("error")
            time.sleep(0.5)
            self.speak(f"Warning! Missing sensors: {', '.join(missing_sensors)}. Some features may not work.")
        
        self.get_logger().info(f"✅ Available sensors: {', '.join(sensors)}")
        
        if len(sensors) == 0:
            self.get_logger().error("No sensors detected! Robot cannot operate safely.")
            self.beep("error")
            self.beep("error")
            self.speak("Critical error! No sensors detected. Please check hardware connections.")
            return
        
        # Startup success!
        self.beep("success")
        self.speak(f"Mapper ready with {len(sensors)} sensors. Say 'start mapping' to begin!")
        
        # Start sensor monitoring thread
        self.sensor_monitor_thread = threading.Thread(target=self._sensor_monitor_loop, daemon=True)
        self.sensor_monitor_thread.start()
        
        # Voice thread
        voice_thread = threading.Thread(target=self.voice_loop, daemon=True)
        voice_thread.start()
        
        # ROS spin
        while self.running and rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
            except KeyboardInterrupt:
                break
        
        # Cleanup
        self.running = False
        self.exploring = False
        
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Save discovery log
        self.discovery_log.save()
        
        # Stop SLAM if running
        if self.mapping:
            self.stop_slam()
        
        summary = self.discovery_log.get_summary()
        print(f"\n📋 Session Summary:")
        print(f"   Duration: {summary['duration_minutes']:.1f} minutes")
        print(f"   Distance: {summary['distance_meters']:.2f}m")
        print(f"   Discoveries: {summary['discoveries']}")
        print(f"   Log saved: {self.discovery_log.session_file}")


def main():
    rclpy.init()
    robot = VoiceMapper()
    
    try:
        robot.run()
    except KeyboardInterrupt:
        pass
    finally:
        robot.running = False
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
