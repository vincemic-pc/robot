#!/usr/bin/env python3
# ⚠️ DEPRECATED: This script is hardcoded for the removed HP60C camera.
# Use voice_mapper.py instead, which supports OAK-D Pro via CameraConfig abstraction.
# This file is kept for reference only and is not maintained.
"""
Yahboom-Style Voice Explorer Robot (DEPRECATED)
================================================
Matches Yahboom's proven depth camera usage pattern:
- RGB stream for visual features (following, tracking, object detection)
- Single-point depth queries on-demand (get_dist(x,y) for distance to objects)
- LiDAR for navigation and obstacle avoidance (NOT depth point cloud)

This approach works reliably because:
1. RGB-only streaming stays within USB 2.0 bandwidth (~23 MB/s)
2. Depth queries are on-demand, not continuous streaming
3. LiDAR handles all navigation/costmap needs
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
import numpy as np
import pyaudio
from io import BytesIO
from openai import OpenAI
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, List

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry

# Try CV imports
try:
    from cv_bridge import CvBridge
    import cv2
    HAS_CV = True
except ImportError:
    HAS_CV = False
    print("Warning: OpenCV not available, using PIL fallback")

# Try scipy for audio resampling
try:
    from scipy.io import wavfile
    from scipy import signal
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


@dataclass
class DepthQuery:
    """Result of a single-point depth query (Yahboom style)"""
    x: int  # pixel x coordinate
    y: int  # pixel y coordinate  
    distance: float  # distance in meters
    valid: bool  # whether the measurement is valid


class YahboomExplorer(Node):
    """
    Voice-controlled explorer using Yahboom's depth camera pattern:
    - RGB for vision (GPT-4o analysis)
    - On-demand depth queries (not streaming)
    - LiDAR for navigation
    """
    
    def __init__(self):
        super().__init__("yahboom_explorer")
        
        # OpenAI client
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            self.get_logger().error("OPENAI_API_KEY not set!")
            sys.exit(1)
        
        self.client = OpenAI(api_key=api_key)
        self.get_logger().info("OpenAI client initialized")
        
        # Robot control
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
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
        
        # === Yahboom-style sensor subscriptions ===
        # RGB camera - primary visual sensor (RELIABLE QoS for HP60C)
        self.rgb_sub = self.create_subscription(
            Image, "/ascamera_hp60c/camera_publisher/rgb0/image",
            self.rgb_callback, reliable_qos
        )
        
        # Depth image - for on-demand queries only (not continuous processing)
        # Subscribe but don't process every frame - only when queried
        self.depth_sub = self.create_subscription(
            Image, "/ascamera_hp60c/camera_publisher/depth0/image",
            self.depth_callback, reliable_qos
        )
        
        # LiDAR - PRIMARY navigation sensor (Yahboom uses this for Nav2)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, best_effort_qos
        )
        
        # Odometry
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, best_effort_qos
        )
        
        # === Sensor state ===
        self.latest_rgb = None
        self.latest_rgb_time = None
        self.latest_depth = None  # Cached depth frame for queries
        self.latest_depth_time = None
        self.latest_scan = None
        self.latest_odom = None
        
        # Depth query caching - don't process every depth frame
        self.depth_query_cache = {}
        self.depth_cache_timeout = 0.5  # seconds
        
        # === Robot state ===
        self.running = True
        self.exploring = False
        self.speaking = False
        self.listening = False
        
        # Position tracking
        self.current_position = {'x': 0, 'y': 0, 'theta': 0}
        self.total_distance = 0.0
        self.last_position = None
        
        # LiDAR-based obstacle detection (primary navigation)
        self.obstacle_distances = {
            "front": float('inf'), "front_left": float('inf'),
            "front_right": float('inf'), "left": float('inf'),
            "right": float('inf'), "back": float('inf')
        }
        self.obstacles = {}
        
        # Movement parameters
        self.min_obstacle_dist = 0.4
        self.slow_dist = 0.8
        self.linear_speed = 0.15
        self.slow_speed = 0.08
        self.angular_speed = 0.4
        
        # Exploration state
        self.look_interval = 15.0
        self.discoveries = []
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
        
        # Conversation
        self.conversation = []
        self.system_prompt = self._build_system_prompt()
        
        self.get_logger().info("="*50)
        self.get_logger().info("Yahboom-Style Explorer initialized!")
        self.get_logger().info("- RGB: visual features & GPT-4o vision")
        self.get_logger().info("- Depth: on-demand distance queries")
        self.get_logger().info("- LiDAR: navigation & obstacle avoidance")
        self.get_logger().info("="*50)

    def _find_microphone(self):
        """Find microphone - prefer PulseAudio on Jetson"""
        p = pyaudio.PyAudio()
        pulse_index = None
        default_index = None
        
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            name = info.get('name', '').lower()
            max_input = info.get('maxInputChannels', 0)
            
            if 'pulse' in name and max_input > 0:
                pulse_index = i
            elif name == 'default' and max_input > 0:
                default_index = i
        
        p.terminate()
        
        if pulse_index is not None:
            self.get_logger().info(f"Using PulseAudio (index {pulse_index})")
            return pulse_index
        elif default_index is not None:
            return default_index
        return 0

    def _build_system_prompt(self):
        return """You are a voice-controlled explorer robot with a camera, depth sensor, and LiDAR.

SENSORS:
- RGB Camera: See and describe the environment
- Depth Camera: Measure distance to specific objects (on-demand)
- LiDAR: 360° obstacle detection for safe navigation

CAPABILITIES & COMMANDS (respond with JSON):

1. Movement: {"action": "move", "linear": 0.15, "angular": 0.0, "duration": 2.0, "speech": "Moving forward!"}
   - linear: -0.3 to 0.3 (negative=backward)
   - angular: -0.5 to 0.5 (positive=left, negative=right)

2. Look/Observe: {"action": "look", "speech": "Let me see..."}
   - Takes RGB image and describes what you see

3. Get Distance: {"action": "get_dist", "target": "object name", "speech": "Measuring distance..."}
   - Uses depth camera to measure distance to a detected object
   - Example: {"action": "get_dist", "target": "person", "speech": "Let me check how far away they are"}

4. Start exploring: {"action": "explore", "speech": "Starting exploration!"}

5. Stop: {"action": "stop", "speech": "Stopping now."}

6. Just talk: {"action": "speak", "speech": "Your response here"}

7. Status: {"action": "status", "speech": "Reporting..."}

8. Follow color: {"action": "follow_color", "color": "red/green/blue/yellow", "speech": "Following the color!"}

NAVIGATION:
- LiDAR provides obstacle distances (front, left, right, etc.)
- You automatically avoid obstacles while moving
- Depth camera is for measuring distance to specific objects, NOT for navigation

PERSONALITY:
- Friendly, curious explorer
- Brief responses (1-2 sentences)
- Report distances when asked about objects

Always respond with valid JSON only."""

    # === Sensor Callbacks ===
    
    def rgb_callback(self, msg):
        """RGB image callback - primary visual sensor"""
        self.latest_rgb = msg
        self.latest_rgb_time = time.time()

    def depth_callback(self, msg):
        """Depth image callback - cached for on-demand queries"""
        self.latest_depth = msg
        self.latest_depth_time = time.time()
        # Clear cache when new depth frame arrives
        self.depth_query_cache = {}

    def scan_callback(self, msg):
        """LiDAR callback - PRIMARY navigation sensor"""
        self.latest_scan = msg
        if not msg.ranges:
            return
        
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges) | np.isnan(ranges), 10.0, ranges)
        num_points = len(ranges)
        
        # Sector definitions for obstacle detection
        sectors = {
            "front": (int(num_points * 0.45), int(num_points * 0.55)),
            "front_left": (int(num_points * 0.55), int(num_points * 0.7)),
            "front_right": (int(num_points * 0.3), int(num_points * 0.45)),
            "left": (int(num_points * 0.7), int(num_points * 0.85)),
            "right": (int(num_points * 0.15), int(num_points * 0.3)),
            "back": (0, int(num_points * 0.1)),
        }
        
        for sector, (start, end) in sectors.items():
            if start < end:
                sector_ranges = ranges[start:end]
            else:
                sector_ranges = np.concatenate([ranges[start:], ranges[:end]])
            
            if len(sector_ranges) > 0:
                min_dist = np.min(sector_ranges)
                self.obstacle_distances[sector] = min_dist
                self.obstacles[sector] = min_dist < self.min_obstacle_dist

    def odom_callback(self, msg):
        """Odometry callback for position tracking"""
        self.latest_odom = msg
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        new_pos = {'x': pos.x, 'y': pos.y, 'theta': theta}
        
        if self.last_position:
            dx = new_pos['x'] - self.last_position['x']
            dy = new_pos['y'] - self.last_position['y']
            self.total_distance += math.sqrt(dx*dx + dy*dy)
        
        self.last_position = self.current_position.copy()
        self.current_position = new_pos

    # === Yahboom-Style Depth Query ===
    
    def get_dist(self, x: int, y: int) -> DepthQuery:
        """
        Yahboom-style single-point depth query: get_dist(x, y)
        
        This is how Yahboom uses the depth camera - NOT for streaming point clouds,
        but for on-demand distance queries to specific pixel coordinates.
        
        Args:
            x: Pixel x coordinate (0-639)
            y: Pixel y coordinate (0-479)
            
        Returns:
            DepthQuery with distance in meters
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
            return DepthQuery(x, y, -1.0, False)
        
        # Check depth age
        if self.latest_depth_time and (time.time() - self.latest_depth_time) > 2.0:
            self.get_logger().warning("Depth data is stale")
            return DepthQuery(x, y, -1.0, False)
        
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
                result = DepthQuery(x, y, -1.0, False)
            else:
                # HP60C depth is in millimeters, convert to meters
                depth_mm = float(np.median(valid_depths))
                depth_m = depth_mm / 1000.0
                
                # Validate range (HP60C range is 0.2m - 8m)
                if 0.1 < depth_m < 10.0:
                    result = DepthQuery(x, y, depth_m, True)
                    self.get_logger().info(f"📏 Depth at ({x},{y}): {depth_m:.2f}m")
                else:
                    result = DepthQuery(x, y, depth_m, False)
            
            # Cache result
            self.depth_query_cache[cache_key] = (time.time(), result)
            return result
            
        except Exception as e:
            self.get_logger().error(f"Depth query error: {e}")
            return DepthQuery(x, y, -1.0, False)

    def get_center_distance(self) -> Optional[float]:
        """Get distance to object in center of frame (common use case)"""
        result = self.get_dist(320, 240)  # Center of 640x480
        return result.distance if result.valid else None

    def find_object_distance(self, target_name: str) -> Optional[Tuple[float, str]]:
        """
        Find an object in the RGB image and get its distance.
        This is how Yahboom combines RGB vision with depth queries.
        
        Args:
            target_name: Name of object to find (e.g., "person", "bottle", "chair")
            
        Returns:
            Tuple of (distance_meters, description) or None if not found
        """
        if not self.latest_rgb:
            return None
        
        # First, use GPT-4o vision to find the object and get its approximate location
        image_b64 = self.image_to_base64(self.latest_rgb)
        if not image_b64:
            return None
        
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
                
                result = json.loads(result_text)
            except:
                # Try to extract JSON
                start = result_text.find('{')
                end = result_text.rfind('}') + 1
                if start >= 0 and end > start:
                    result = json.loads(result_text[start:end])
                else:
                    return None
            
            if not result.get("found", False):
                return None
            
            # Get depth at the detected location
            x = int(result.get("x", 320))
            y = int(result.get("y", 240))
            description = result.get("description", f"Found {target_name}")
            
            depth_result = self.get_dist(x, y)
            
            if depth_result.valid:
                return (depth_result.distance, description)
            else:
                # Depth not available, but we found it visually
                return (-1.0, f"{description} (distance unavailable)")
                
        except Exception as e:
            self.get_logger().error(f"Object detection error: {e}")
            return None

    # === Audio Functions ===
    
    def listen(self):
        """Record speech and return audio file path"""
        if self.speaking:
            return None
            
        self.listening = True
        p = pyaudio.PyAudio()
        
        try:
            dev_info = p.get_device_info_by_index(self.mic_index)
            actual_channels = 1
            actual_rate = int(dev_info.get('defaultSampleRate', self.sample_rate))
            if actual_rate not in [16000, 44100, 48000]:
                actual_rate = 44100
        except Exception as e:
            actual_channels = 1
            actual_rate = 44100
        
        try:
            stream = p.open(
                format=pyaudio.paInt16,
                channels=actual_channels,
                rate=actual_rate,
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
        max_silent = int(self.silence_duration * actual_rate / self.chunk_size)
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
        
        duration = len(frames) * self.chunk_size / actual_rate
        if duration < self.min_audio_duration:
            return None
        
        # Save and resample
        temp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
        with wave.open(temp.name, 'wb') as wf:
            wf.setnchannels(actual_channels)
            wf.setsampwidth(2)
            wf.setframerate(actual_rate)
            wf.writeframes(b''.join(frames))
        
        if HAS_SCIPY and actual_rate != self.whisper_rate:
            try:
                rate, data = wavfile.read(temp.name)
                num_samples = int(len(data) * self.whisper_rate / rate)
                resampled = signal.resample(data, num_samples).astype(np.int16)
                
                resampled_file = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
                wavfile.write(resampled_file.name, self.whisper_rate, resampled)
                os.unlink(temp.name)
                
                self.get_logger().info(f"📝 Recorded {duration:.1f}s")
                return resampled_file.name
            except:
                pass
        
        return temp.name

    def transcribe(self, audio_file):
        """Transcribe audio using Whisper"""
        try:
            with open(audio_file, "rb") as f:
                result = self.client.audio.transcriptions.create(
                    model="whisper-1", file=f, language="en"
                )
            text = result.text.strip()
            self.get_logger().info(f"🗣️ You: {text}")
            return text
        except Exception as e:
            self.get_logger().error(f"Transcription error: {e}")
            return None
        finally:
            try:
                os.unlink(audio_file)
            except:
                pass

    def speak(self, text):
        """Text to speech using OpenAI TTS"""
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
            
            try:
                wav_file = temp_file.name.replace(".mp3", ".wav")
                subprocess.run(
                    ["ffmpeg", "-y", "-i", temp_file.name, "-ar", "48000", "-ac", "1", wav_file],
                    capture_output=True, timeout=10
                )
                subprocess.run(
                    ["aplay", "-D", "plughw:0,0", wav_file],
                    capture_output=True, timeout=30
                )
                os.unlink(wav_file)
            except:
                try:
                    subprocess.run(["mpv", "--no-video", temp_file.name], capture_output=True, timeout=30)
                except:
                    pass
            
            os.unlink(temp_file.name)
        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")
        finally:
            self.speaking = False

    # === Vision Functions (RGB-based) ===
    
    def image_to_base64(self, image_msg):
        """Convert ROS Image to base64 - RGB only"""
        try:
            if HAS_CV:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
                
                # Enhance dark images (HP60C issue)
                mean_brightness = cv_image.mean()
                if mean_brightness < 30:
                    cv_image = cv2.convertScaleAbs(cv_image, alpha=3.0, beta=30)
                
                cv_image = cv2.resize(cv_image, (640, 480))
                _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                return base64.b64encode(buffer).decode('utf-8')
            else:
                from PIL import Image as PILImage
                from PIL import ImageEnhance
                img_array = np.frombuffer(image_msg.data, dtype=np.uint8)
                img_array = img_array.reshape((image_msg.height, image_msg.width, 3))
                if image_msg.encoding == "bgr8":
                    img_array = img_array[:, :, ::-1]
                pil_img = PILImage.fromarray(img_array)
                
                # Enhance if dark
                if np.mean(img_array) < 30:
                    enhancer = ImageEnhance.Brightness(pil_img)
                    pil_img = enhancer.enhance(3.0)
                
                pil_img = pil_img.resize((640, 480))
                buffer = BytesIO()
                pil_img.save(buffer, format="JPEG", quality=80)
                return base64.b64encode(buffer.getvalue()).decode('utf-8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return None

    def observe(self) -> str:
        """Look at the world and describe what we see (RGB-based vision)"""
        if not self.latest_rgb:
            return "I can't see anything - camera isn't working."
        
        image_b64 = self.image_to_base64(self.latest_rgb)
        if not image_b64:
            return "Had trouble processing the camera image."
        
        self.get_logger().info("📷 Looking around (RGB)...")
        
        # Also get center distance if depth is available
        center_dist = self.get_center_distance()
        dist_context = ""
        if center_dist and center_dist > 0:
            dist_context = f" The object in the center is about {center_dist:.1f} meters away."
        
        try:
            response = self.client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": "You are a curious robot explorer. Describe what you see in 1-2 short sentences. Be specific about objects, colors, and interesting details. If the image is dark, mention that briefly."},
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": "What do you see?"},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}", "detail": "low"}}
                        ]
                    }
                ],
                max_tokens=100,
                temperature=0.8
            )
            
            observation = response.choices[0].message.content.strip()
            if dist_context:
                observation += dist_context
            
            self.get_logger().info(f"👁️ {observation}")
            
            self.discoveries.append({
                "observation": observation,
                "position": self.current_position.copy(),
                "time": time.time(),
                "center_distance": center_dist
            })
            
            return observation
        except Exception as e:
            self.get_logger().error(f"Vision error: {e}")
            return "I had trouble seeing."

    # === Movement Functions (LiDAR-based navigation) ===
    
    def move(self, linear: float, angular: float, duration: float):
        """Move robot with LiDAR-based obstacle checking"""
        twist = Twist()
        start_time = time.time()
        
        while (time.time() - start_time) < duration and self.running:
            # LiDAR-based safety checks
            if linear > 0 and self.obstacles.get("front", False):
                self.get_logger().warning("🛑 LiDAR: Obstacle ahead!")
                break
            if linear < 0 and self.obstacles.get("back", False):
                self.get_logger().warning("🛑 LiDAR: Obstacle behind!")
                break
            
            # Slow down near obstacles
            actual_linear = linear
            front_dist = self.obstacle_distances.get("front", 10)
            if linear > 0 and front_dist < self.slow_dist:
                actual_linear = self.slow_speed * (1 if linear > 0 else -1)
            
            twist.linear.x = float(actual_linear)
            twist.angular.z = float(angular)
            self.cmd_vel_pub.publish(twist)
            
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        
        # Stop
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def choose_direction(self) -> Tuple[float, float]:
        """Smart direction selection based on LiDAR"""
        front = self.obstacle_distances.get("front", 10)
        front_left = self.obstacle_distances.get("front_left", 10)
        front_right = self.obstacle_distances.get("front_right", 10)
        
        # Clear path ahead
        if front > self.slow_dist and front_left > self.min_obstacle_dist and front_right > self.min_obstacle_dist:
            if random.random() < 0.1:
                return (self.linear_speed * 0.5, random.choice([-1, 1]) * self.angular_speed * 0.5)
            return (self.linear_speed, random.uniform(-0.1, 0.1))
        
        # Need to turn
        if front <= self.slow_dist:
            return (0, self.angular_speed if front_left > front_right else -self.angular_speed)
        
        # Stuck recovery
        self.stuck_counter += 1
        if self.stuck_counter > 5:
            self.stuck_counter = 0
            return (-self.slow_speed, random.choice([-1, 1]) * self.angular_speed)
        
        return (0, self.angular_speed if front_left > front_right else -self.angular_speed)

    # === Brain Functions ===
    
    def get_context(self) -> str:
        """Get current robot context for GPT"""
        context = f"""
Current status:
- Position: x={self.current_position['x']:.1f}, y={self.current_position['y']:.1f}
- Distance traveled: {self.total_distance:.1f}m
- Exploring: {'yes' if self.exploring else 'no'}
- LiDAR obstacles: front={self.obstacle_distances.get('front', 0):.1f}m, left={self.obstacle_distances.get('left', 0):.1f}m, right={self.obstacle_distances.get('right', 0):.1f}m
- Sensors: RGB={'✓' if self.latest_rgb else '✗'}, Depth={'✓' if self.latest_depth else '✗'}, LiDAR={'✓' if self.latest_scan else '✗'}
- Discoveries: {len(self.discoveries)}
"""
        if self.discoveries:
            context += f"- Last seen: {self.discoveries[-1]['observation'][:50]}..."
        return context

    def think(self, user_input: str) -> str:
        """Process user input with GPT-4o"""
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
            return '{"action": "speak", "speech": "Sorry, I had trouble thinking."}'

    def execute(self, response_json: str):
        """Execute action from JSON response"""
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
            linear = max(-0.3, min(0.3, float(data.get("linear", 0))))
            angular = max(-0.5, min(0.5, float(data.get("angular", 0))))
            duration = min(5.0, float(data.get("duration", 2)))
            
            if speech:
                self.speak(speech)
            
            self.get_logger().info(f"🚗 Moving: linear={linear:.2f}, angular={angular:.2f}")
            self.move(linear, angular, duration)
            
        elif action == "look":
            if speech:
                self.speak(speech)
            observation = self.observe()
            if observation:
                self.speak(observation)
                
        elif action == "get_dist":
            target = data.get("target", "object")
            if speech:
                self.speak(speech)
            
            result = self.find_object_distance(target)
            if result:
                distance, description = result
                if distance > 0:
                    self.speak(f"{description} It's about {distance:.1f} meters away.")
                else:
                    self.speak(f"{description}")
            else:
                self.speak(f"I couldn't find a {target} in my view.")
                
        elif action == "explore":
            if speech:
                self.speak(speech)
            self.start_exploration()
            
        elif action == "stop":
            self.stop_exploration()
            if speech:
                self.speak(speech)
                
        elif action == "status":
            status = f"I've traveled {self.total_distance:.1f} meters and found {len(self.discoveries)} interesting things. "
            status += f"Front is clear at {self.obstacle_distances.get('front', 0):.1f} meters."
            if self.latest_depth:
                center = self.get_center_distance()
                if center and center > 0:
                    status += f" Center object is {center:.1f} meters away."
            self.speak(status)
            
        elif action == "follow_color":
            color = data.get("color", "red")
            if speech:
                self.speak(speech)
            self.follow_color(color)
            
        elif action == "speak" and speech:
            self.speak(speech)

    def follow_color(self, color: str, duration: float = 10.0):
        """Simple color following using RGB camera"""
        self.get_logger().info(f"🎨 Following {color} for {duration}s")
        
        color_ranges = {
            "red": ([0, 100, 100], [10, 255, 255]),
            "green": ([40, 50, 50], [80, 255, 255]),
            "blue": ([100, 100, 100], [130, 255, 255]),
            "yellow": ([20, 100, 100], [40, 255, 255]),
        }
        
        if color not in color_ranges:
            self.speak(f"I don't know how to follow {color}")
            return
        
        lower, upper = color_ranges[color]
        lower = np.array(lower)
        upper = np.array(upper)
        
        start_time = time.time()
        
        while (time.time() - start_time) < duration and self.running:
            if not self.latest_rgb or not HAS_CV:
                time.sleep(0.1)
                continue
            
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_rgb, desired_encoding="bgr8")
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower, upper)
                
                # Find contours
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if contours:
                    largest = max(contours, key=cv2.contourArea)
                    M = cv2.moments(largest)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        
                        # Steer towards color
                        error = (cx - 320) / 320.0  # -1 to 1
                        angular = -error * self.angular_speed
                        linear = self.slow_speed if abs(error) < 0.3 else 0
                        
                        # Check LiDAR before moving
                        if linear > 0 and self.obstacles.get("front", False):
                            linear = 0
                        
                        twist = Twist()
                        twist.linear.x = linear
                        twist.angular.z = angular
                        self.cmd_vel_pub.publish(twist)
                
            except Exception as e:
                self.get_logger().error(f"Color follow error: {e}")
            
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        
        # Stop
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    # === Exploration ===
    
    def exploration_loop(self):
        """Autonomous exploration with periodic observations"""
        last_observation = time.time() - self.look_interval + 5
        
        while self.running and self.exploring:
            if time.time() - last_observation >= self.look_interval:
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.5)
                
                observation = self.observe()
                if observation:
                    self.speak(observation)
                last_observation = time.time()
                
                while self.speaking and self.running:
                    rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.exploring and self.running:
                linear, angular = self.choose_direction()
                self.move(linear, angular, 2.0)
            
            rclpy.spin_once(self, timeout_sec=0.05)
        
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def start_exploration(self):
        if self.exploring:
            return
        self.exploring = True
        self.stuck_counter = 0
        self.explore_thread = threading.Thread(target=self.exploration_loop, daemon=True)
        self.explore_thread.start()
        self.get_logger().info("🚀 Exploration started (LiDAR navigation)")

    def stop_exploration(self):
        self.exploring = False
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Exploration stopped")

    # === Main Loop ===
    
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
            self.get_logger().info(f"🤖 Response: {response}")
            self.execute(response)

    def run(self):
        """Main entry point"""
        print("\n" + "="*60)
        print("🤖 YAHBOOM-STYLE VOICE EXPLORER")
        print("="*60)
        print("Sensor Strategy (matches Yahboom tutorials):")
        print("  📷 RGB Camera: Visual features & GPT-4o vision")
        print("  📏 Depth Camera: On-demand distance queries")
        print("  📡 LiDAR: Navigation & obstacle avoidance")
        print("-"*60)
        print("Commands:")
        print("  - 'Go forward' / 'Turn left' / 'Back up'")
        print("  - 'What do you see?' / 'Look around'")
        print("  - 'How far is the [object]?' (depth query)")
        print("  - 'Follow the red color'")
        print("  - 'Start exploring' / 'Stop'")
        print("  - 'Status' / 'How far have you gone?'")
        print("="*60)
        print("Press Ctrl+C to quit\n")
        
        # Wait for sensors
        self.get_logger().info("Waiting for sensors...")
        start_wait = time.time()
        while (time.time() - start_wait) < 10:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.latest_rgb and self.latest_scan:
                break
        
        sensors = []
        if self.latest_rgb:
            sensors.append("RGB camera")
        if self.latest_depth:
            sensors.append("depth (for queries)")
        if self.latest_scan:
            sensors.append("LiDAR")
        if self.latest_odom:
            sensors.append("odometry")
        
        self.get_logger().info(f"✅ Sensors: {', '.join(sensors)}")
        
        # Startup message
        intro = f"Hello! I'm ready with {len(sensors)} sensors. "
        intro += "I use LiDAR for navigation and can tell you distances to objects. "
        intro += "Say 'start exploring' or ask me anything!"
        self.speak(intro)
        
        # Start voice thread
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
        
        print(f"\n📋 Session Summary:")
        print(f"   Distance traveled: {self.total_distance:.2f}m")
        print(f"   Discoveries: {len(self.discoveries)}")


def main():
    rclpy.init()
    robot = YahboomExplorer()
    
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
