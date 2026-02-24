#!/bin/bash
#===============================================================================
# ROSMASTER A1 - LLM Integration Setup
# Sets up Large Language Model integration for intelligent robot control
# Supports: OpenAI GPT-4, Anthropic Claude, Google Gemini, Local Ollama
#===============================================================================

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${CYAN}"
    echo "╔════════════════════════════════════════════╗"
    echo "║   ROSMASTER A1 - LLM Brain Integration     ║"
    echo "║   Intelligent Voice & Vision Control       ║"
    echo "╚════════════════════════════════════════════╝"
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

# Source ROS2
source_ros2() {
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
    if [ -f ~/rosmaster_a1_ws/install/setup.bash ]; then
        source ~/rosmaster_a1_ws/install/setup.bash
    elif [ -f ~/yahboomcar_ws/install/setup.bash ]; then
        source ~/yahboomcar_ws/install/setup.bash
    fi
}

# Install LLM dependencies
install_llm_deps() {
    print_info "Installing LLM dependencies..."
    
    pip3 install --user \
        openai \
        anthropic \
        google-generativeai \
        requests \
        python-dotenv
    
    print_info "LLM Python packages installed!"
}

# Install Ollama for local LLM
install_ollama() {
    print_info "Installing Ollama for local LLM inference..."
    
    # Check if running on Jetson (ARM64)
    if uname -m | grep -q "aarch64"; then
        print_info "Detected ARM64 (Jetson), installing Ollama..."
        curl -fsSL https://ollama.com/install.sh | sh
    else
        print_info "Installing Ollama..."
        curl -fsSL https://ollama.com/install.sh | sh
    fi
    
    # Start Ollama service
    print_info "Starting Ollama service..."
    ollama serve &
    sleep 5
    
    # Pull a model
    print_info "Pulling llama3.2 model (this may take a while)..."
    ollama pull llama3.2
    
    print_info "Ollama installed and ready!"
}

# Configure API keys
configure_api_keys() {
    print_info "Configuring API keys..."
    
    CONFIG_FILE=~/.rosmaster_llm_config
    
    echo ""
    echo "Choose your LLM provider:"
    echo "  1) OpenAI (GPT-4, GPT-4o)"
    echo "  2) Anthropic (Claude)"
    echo "  3) Ollama (Local - no API key needed)"
    echo "  4) Google (Gemini)"
    echo ""
    read -p "Enter choice [1-4]: " provider_choice
    
    case ${provider_choice} in
        1)
            read -p "Enter OpenAI API key: " api_key
            echo "export OPENAI_API_KEY='${api_key}'" > ${CONFIG_FILE}
            echo "export LLM_PROVIDER='openai'" >> ${CONFIG_FILE}
            echo "export LLM_MODEL='gpt-4o'" >> ${CONFIG_FILE}
            ;;
        2)
            read -p "Enter Anthropic API key: " api_key
            echo "export ANTHROPIC_API_KEY='${api_key}'" > ${CONFIG_FILE}
            echo "export LLM_PROVIDER='anthropic'" >> ${CONFIG_FILE}
            echo "export LLM_MODEL='claude-sonnet-4-20250514'" >> ${CONFIG_FILE}
            ;;
        3)
            echo "export LLM_PROVIDER='ollama'" > ${CONFIG_FILE}
            echo "export LLM_MODEL='llama3.2'" >> ${CONFIG_FILE}
            echo "export OLLAMA_HOST='http://localhost:11434'" >> ${CONFIG_FILE}
            ;;
        4)
            read -p "Enter Google AI API key: " api_key
            echo "export GOOGLE_API_KEY='${api_key}'" > ${CONFIG_FILE}
            echo "export LLM_PROVIDER='google'" >> ${CONFIG_FILE}
            echo "export LLM_MODEL='gemini-pro'" >> ${CONFIG_FILE}
            ;;
    esac
    
    chmod 600 ${CONFIG_FILE}
    print_info "Configuration saved to ${CONFIG_FILE}"
    print_info "Run 'source ${CONFIG_FILE}' or restart terminal"
}

# Create the speech output node
create_speech_node() {
    print_info "Creating speech output node..."
    
    mkdir -p ~/rosmaster_a1_scripts
    
    cat > ~/rosmaster_a1_scripts/speech_output.py << 'PYTHON_EOF'
#!/usr/bin/env python3
"""
Speech output node - converts text to speech
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
import threading
import queue


class SpeechOutput(Node):
    def __init__(self):
        super().__init__('speech_output')
        
        self.speech_queue = queue.Queue()
        
        # TTS engine
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)
        
        # Subscriber
        self.sub = self.create_subscription(
            String, '/speech_output', self.speech_callback, 10
        )
        
        # Speech thread
        self.speech_thread = threading.Thread(target=self._speech_loop, daemon=True)
        self.speech_thread.start()
        
        self.get_logger().info("Speech output ready!")
    
    def speech_callback(self, msg):
        self.speech_queue.put(msg.data)
    
    def _speech_loop(self):
        while True:
            text = self.speech_queue.get()
            try:
                self.engine.say(text)
                self.engine.runAndWait()
            except Exception as e:
                self.get_logger().error(f"Speech error: {e}")


def main():
    rclpy.init()
    node = SpeechOutput()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
PYTHON_EOF

    chmod +x ~/rosmaster_a1_scripts/speech_output.py
}

# Create the action executor node
create_action_executor() {
    print_info "Creating action executor node..."
    
    mkdir -p ~/rosmaster_a1_scripts
    
    cat > ~/rosmaster_a1_scripts/action_executor.py << 'PYTHON_EOF'
#!/usr/bin/env python3
"""
Action Executor - Executes robot actions from LLM commands
Bridges LLM decisions to actual robot hardware
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
import re
import math
import yaml
import os


class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_pub = self.create_publisher(String, '/robot_feedback', 10)
        
        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribers
        self.action_sub = self.create_subscription(
            String, '/robot_action', self.action_callback, 10
        )
        
        # Load navigation points
        self.nav_points = self.load_nav_points()
        
        # Current pose storage
        self.saved_pose = None
        
        self.get_logger().info("Action Executor ready!")
    
    def load_nav_points(self):
        config_path = os.path.expanduser('~/rosmaster_a1_config/nav_points.yaml')
        try:
            with open(config_path) as f:
                config = yaml.safe_load(f)
                return config.get('navigation_points', {})
        except:
            return {
                'A': [1.0, 0.0, 0.0],
                'B': [0.0, 1.0, 1.57],
                'C': [-1.0, 0.0, 3.14],
                'zero': [0.0, 0.0, 0.0]
            }
    
    def action_callback(self, msg):
        action = msg.data
        self.get_logger().info(f"Executing action: {action}")
        
        try:
            result = self.execute_action(action)
            self.send_feedback(f"Action {action} {'completed' if result else 'failed'}")
        except Exception as e:
            self.get_logger().error(f"Action error: {e}")
            self.send_feedback(f"Action {action} failed: {e}")
    
    def send_feedback(self, message):
        msg = String()
        msg.data = message
        self.feedback_pub.publish(msg)
    
    def execute_action(self, action: str) -> bool:
        """Parse and execute an action string"""
        
        # Navigation
        if action.startswith("navigation("):
            point = re.search(r'navigation\((\w+)\)', action).group(1)
            return self.navigate_to(point)
        
        # Movement
        elif action.startswith("set_cmdvel("):
            params = re.search(r'set_cmdvel\(([\d\.\-,\s]+)\)', action).group(1)
            values = [float(x.strip()) for x in params.split(',')]
            return self.set_velocity(*values)
        
        elif action.startswith("move_left("):
            params = re.search(r'move_left\(([\d\.\-,\s]+)\)', action).group(1)
            values = [float(x.strip()) for x in params.split(',')]
            return self.turn(values[0], values[1] if len(values) > 1 else 3.0, left=True)
        
        elif action.startswith("move_right("):
            params = re.search(r'move_right\(([\d\.\-,\s]+)\)', action).group(1)
            values = [float(x.strip()) for x in params.split(',')]
            return self.turn(values[0], values[1] if len(values) > 1 else 3.0, left=False)
        
        # Position
        elif action == "get_current_pose()":
            return self.save_current_pose()
        
        # Following modes - publish to enable topics
        elif action == "face_follow()":
            return self.enable_mode('/face_follow/enable')
        elif action.startswith("color_follow("):
            color = re.search(r'color_follow\((\w+)\)', action).group(1)
            return self.enable_color_follow(color)
        elif action == "poseFollow()":
            return self.enable_mode('/pose_follow/enable')
        elif action == "stop_follow()":
            return self.disable_all_following()
        
        # Line following
        elif action.startswith("follow_line("):
            color = re.search(r'follow_line\((\w+)\)', action).group(1)
            return self.enable_line_follow(color)
        
        # Vision
        elif action == "seewhat()":
            return self.capture_image()
        
        # Servo
        elif action == "servo_nod()":
            return self.servo_gesture('nod')
        elif action == "servo_shake()":
            return self.servo_gesture('shake')
        elif action == "servo_init()":
            return self.servo_gesture('reset')
        
        # Task control
        elif action == "finishtask()":
            self.send_feedback("All tasks completed")
            return True
        elif action == "finish_dialogue()":
            self.send_feedback("Dialogue ended")
            return True
        elif action.startswith("wait("):
            seconds = float(re.search(r'wait\(([\d\.]+)\)', action).group(1))
            import time
            time.sleep(seconds)
            return True
        
        else:
            self.get_logger().warning(f"Unknown action: {action}")
            return False
    
    def navigate_to(self, point: str) -> bool:
        """Navigate to a named point"""
        if point == 'zero' and self.saved_pose:
            coords = self.saved_pose
        elif point in self.nav_points:
            coords = self.nav_points[point]
        else:
            self.get_logger().error(f"Unknown point: {point}")
            return False
        
        x, y, theta = coords[0], coords[1], coords[2]
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 not available")
            return False
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(theta / 2)
        goal.pose.pose.orientation.w = math.cos(theta / 2)
        
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        
        return True
    
    def set_velocity(self, lx, ly, az, duration) -> bool:
        """Set robot velocity for duration"""
        twist = Twist()
        twist.linear.x = lx
        twist.linear.y = ly
        twist.angular.z = az
        
        self.cmd_vel_pub.publish(twist)
        
        import time
        time.sleep(duration)
        
        # Stop
        self.cmd_vel_pub.publish(Twist())
        return True
    
    def turn(self, degrees, speed, left=True) -> bool:
        """Turn by degrees"""
        radians = math.radians(degrees)
        duration = radians / speed
        
        twist = Twist()
        twist.angular.z = speed if left else -speed
        
        self.cmd_vel_pub.publish(twist)
        
        import time
        time.sleep(duration)
        
        self.cmd_vel_pub.publish(Twist())
        return True
    
    def save_current_pose(self) -> bool:
        # In practice, subscribe to /amcl_pose or /odom
        self.saved_pose = [0.0, 0.0, 0.0]
        return True
    
    def enable_mode(self, topic: str) -> bool:
        pub = self.create_publisher(Bool, topic, 10)
        msg = Bool()
        msg.data = True
        pub.publish(msg)
        return True
    
    def disable_all_following(self) -> bool:
        for topic in ['/face_follow/enable', '/color_follow/enable', 
                      '/pose_follow/enable', '/line_follower/enable']:
            try:
                pub = self.create_publisher(Bool, topic, 10)
                msg = Bool()
                msg.data = False
                pub.publish(msg)
            except:
                pass
        return True
    
    def enable_color_follow(self, color: str) -> bool:
        # Set color and enable
        color_pub = self.create_publisher(String, '/color_follow/color', 10)
        color_msg = String()
        color_msg.data = color
        color_pub.publish(color_msg)
        return self.enable_mode('/color_follow/enable')
    
    def enable_line_follow(self, color: str) -> bool:
        color_pub = self.create_publisher(String, '/line_follower/color', 10)
        color_msg = String()
        color_msg.data = color
        color_pub.publish(color_msg)
        return self.enable_mode('/line_follower/enable')
    
    def capture_image(self) -> bool:
        # Trigger image capture for LLM analysis
        self.send_feedback("Image captured for analysis")
        return True
    
    def servo_gesture(self, gesture: str) -> bool:
        # Publish servo command
        return True


def main():
    rclpy.init()
    node = ActionExecutor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
PYTHON_EOF

    chmod +x ~/rosmaster_a1_scripts/action_executor.py
}

# Launch LLM brain system
launch_llm_brain() {
    print_info "Launching LLM Robot Brain..."
    
    # Source config
    if [ -f ~/.rosmaster_llm_config ]; then
        source ~/.rosmaster_llm_config
    fi
    
    source_ros2
    
    # Default values
    LLM_PROVIDER="${LLM_PROVIDER:-ollama}"
    LLM_MODEL="${LLM_MODEL:-llama3.2}"
    
    print_info "Using LLM: ${LLM_PROVIDER}/${LLM_MODEL}"
    
    # Start speech output
    print_info "Starting speech output..."
    python3 ~/rosmaster_a1_scripts/speech_output.py &
    sleep 2
    
    # Start action executor
    print_info "Starting action executor..."
    python3 ~/rosmaster_a1_scripts/action_executor.py &
    sleep 2
    
    # Start mapping assistant
    print_info "Starting mapping assistant..."
    python3 ${SCRIPT_DIR}/llm_mapping_assistant.py &
    sleep 2
    
    # Start voice listener (from voice commands script)
    if [ -f ~/rosmaster_a1_scripts/voice_listener.py ]; then
        print_info "Starting voice listener..."
        python3 ~/rosmaster_a1_scripts/voice_listener.py &
        sleep 2
    fi
    
    # Start LLM brain
    print_info "Starting LLM brain..."
    python3 ${SCRIPT_DIR}/llm_robot_brain.py \
        --ros-args \
        -p llm_provider:=${LLM_PROVIDER} \
        -p llm_model:=${LLM_MODEL}
}

# Launch mapping mode with voice
launch_mapping_mode() {
    print_info "Launching voice-driven mapping mode..."
    
    source_ros2
    
    # Source config
    if [ -f ~/.rosmaster_llm_config ]; then
        source ~/.rosmaster_llm_config
    fi
    
    # Start speech output
    print_info "Starting speech output..."
    python3 ~/rosmaster_a1_scripts/speech_output.py &
    sleep 2
    
    # Start voice listener
    if [ -f ~/rosmaster_a1_scripts/voice_listener.py ]; then
        print_info "Starting voice listener..."
        python3 ~/rosmaster_a1_scripts/voice_listener.py &
        sleep 2
    fi
    
    # Start mapping assistant
    print_info "Starting mapping assistant..."
    print_info "Say 'start mapping' to begin learning your home!"
    python3 ${SCRIPT_DIR}/llm_mapping_assistant.py
}

# Interactive chat mode
interactive_chat() {
    print_info "Starting interactive chat mode..."
    
    source_ros2
    
    print_info "Type messages to send to the robot. Press Ctrl+C to exit."
    echo ""
    
    while true; do
        read -p "You: " message
        if [ -n "$message" ]; then
            ros2 topic pub --once /text_input std_msgs/String "data: '${message}'"
        fi
    done
}

# Show usage
show_usage() {
    print_header
    echo ""
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Setup:"
    echo "  install        Install LLM Python packages"
    echo "  install-ollama Install Ollama for local LLM"
    echo "  configure      Configure API keys"
    echo "  setup          Create all required nodes"
    echo ""
    echo "Run:"
    echo "  launch         Launch LLM robot brain"
    echo "  mapping        Voice-driven mapping mode"
    echo "  chat           Interactive text chat mode"
    echo ""
    echo "Supported Providers:"
    echo "  - OpenAI (GPT-4, GPT-4o) - requires API key"
    echo "  - Anthropic (Claude) - requires API key"
    echo "  - Ollama (Local) - no API key, runs on device"
    echo "  - Google (Gemini) - requires API key"
}

# Main
print_header

case "${1:-help}" in
    install)
        install_llm_deps
        ;;
    install-ollama)
        install_ollama
        ;;
    configure)
        configure_api_keys
        ;;
    setup)
        create_speech_node
        create_action_executor
        # Also create voice listener if not exists
        if [ ! -f ~/rosmaster_a1_scripts/voice_listener.py ]; then
            bash ${SCRIPT_DIR}/03_setup_voice_commands.sh setup
        fi
        print_info "All LLM nodes created!"
        ;;
    launch)
        launch_llm_brain
        ;;
    mapping)
        launch_mapping_mode
        ;;
    chat)
        interactive_chat
        ;;
    *)
        show_usage
        ;;
esac
