#!/bin/bash
#===============================================================================
# ROSMASTER A1 - Voice Command Setup Script
# Sets up AI voice recognition and voice control for the robot
# Uses the AI large model voice module integrated with the robot
#===============================================================================

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
LANGUAGE="${LANGUAGE:-en}"  # Language: en, zh
VOICE_MODEL="${VOICE_MODEL:-default}"

print_header() {
    echo -e "${BLUE}"
    echo "========================================"
    echo "  ROSMASTER A1 - Voice Command Setup"
    echo "  Language: ${LANGUAGE}"
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

# Check audio devices
check_audio_devices() {
    print_info "Checking audio devices..."
    
    echo ""
    echo "=== Recording Devices (Microphones) ==="
    arecord -l 2>/dev/null || print_warning "arecord not available"
    
    echo ""
    echo "=== Playback Devices (Speakers) ==="
    aplay -l 2>/dev/null || print_warning "aplay not available"
    
    # Check for the AI voice module
    if lsusb | grep -qi "audio\|microphone\|speech"; then
        print_info "USB audio device detected"
    fi
    
    # Check ALSA devices
    echo ""
    echo "=== ALSA Cards ==="
    cat /proc/asound/cards 2>/dev/null || print_warning "ALSA not available"
}

# Install voice recognition dependencies
install_voice_deps() {
    print_info "Installing voice recognition dependencies..."
    
    sudo apt-get update
    sudo apt-get install -y \
        pulseaudio \
        pulseaudio-utils \
        alsa-utils \
        portaudio19-dev \
        python3-pyaudio \
        libportaudio2 \
        libasound2-dev \
        ffmpeg \
        libespeak1 \
        espeak \
        espeak-ng \
        sox \
        libsox-fmt-all
    
    # Install Python dependencies for voice
    pip3 install --user \
        pyaudio \
        SpeechRecognition \
        pyttsx3 \
        webrtcvad \
        sounddevice \
        soundfile \
        numpy
    
    # Install ROS2 audio packages if available
    sudo apt-get install -y \
        ros-humble-audio-common 2>/dev/null || true
    
    print_info "Voice dependencies installed!"
}

# Configure PulseAudio
configure_audio() {
    print_info "Configuring audio system..."
    
    # Start PulseAudio if not running
    pulseaudio --check 2>/dev/null || pulseaudio --start
    
    # Set default input device
    print_info "Available audio sources:"
    pactl list sources short 2>/dev/null || true
    
    # Set default output device
    print_info "Available audio sinks:"
    pactl list sinks short 2>/dev/null || true
    
    print_info "Audio configured!"
}

# Test microphone
test_microphone() {
    print_info "Testing microphone (speak for 5 seconds)..."
    
    local TEST_FILE="/tmp/mic_test.wav"
    
    # Record for 5 seconds
    arecord -d 5 -f cd -t wav ${TEST_FILE} 2>/dev/null && {
        print_info "Recording complete. Playing back..."
        aplay ${TEST_FILE} 2>/dev/null
        rm -f ${TEST_FILE}
        print_info "Microphone test complete!"
    } || {
        print_error "Failed to record. Check microphone connection."
    }
}

# Test text-to-speech
test_speaker() {
    print_info "Testing text-to-speech..."
    
    if command -v espeak &> /dev/null; then
        espeak "Hello, I am ROSMASTER A1 robot. Voice system is working correctly."
        print_info "Speaker test complete!"
    else
        print_warning "espeak not found, trying pyttsx3..."
        python3 -c "import pyttsx3; engine = pyttsx3.init(); engine.say('Hello, I am ROSMASTER A1 robot'); engine.runAndWait()"
    fi
}

# Create voice command listener Python script
create_voice_listener() {
    print_info "Creating voice command listener..."
    
    mkdir -p ~/rosmaster_a1_scripts
    
    cat > ~/rosmaster_a1_scripts/voice_listener.py << 'PYTHON_EOF'
#!/usr/bin/env python3
"""
ROSMASTER A1 Voice Command Listener
Listens for voice commands and publishes them to ROS2 topics
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import threading
import queue
import time

class VoiceCommandListener(Node):
    def __init__(self):
        super().__init__('voice_command_listener')
        
        # Publishers
        self.command_pub = self.create_publisher(String, '/voice_command', 10)
        self.raw_text_pub = self.create_publisher(String, '/voice_text', 10)
        
        # Speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Adjust for ambient noise
        with self.microphone as source:
            self.get_logger().info("Adjusting for ambient noise... Please wait.")
            self.recognizer.adjust_for_ambient_noise(source, duration=2)
            self.get_logger().info("Ready for voice commands!")
        
        # Command queue
        self.command_queue = queue.Queue()
        
        # Voice commands mapping
        self.command_mapping = {
            # Navigation commands
            'go forward': 'move_forward',
            'move forward': 'move_forward',
            'go back': 'move_backward',
            'move back': 'move_backward',
            'turn left': 'turn_left',
            'turn right': 'turn_right',
            'stop': 'stop',
            'halt': 'stop',
            
            # Navigation points
            'go to kitchen': 'navigation_kitchen',
            'go to bedroom': 'navigation_bedroom',
            'go to living room': 'navigation_living_room',
            'go home': 'navigation_home',
            'return home': 'navigation_home',
            
            # Camera/Vision
            'look around': 'seewhat',
            'what do you see': 'seewhat',
            'take picture': 'capture_image',
            
            # Following
            'follow me': 'face_follow',
            'follow face': 'face_follow',
            'follow color': 'color_follow',
            'stop following': 'stop_follow',
            
            # Line following
            'follow line': 'follow_line',
            'patrol': 'follow_line',
            
            # System
            'hello robot': 'greeting',
            'good bye': 'finish_dialogue',
            'shut down': 'shutdown',
        }
        
        # Start listening thread
        self.listening = True
        self.listen_thread = threading.Thread(target=self._listen_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()
        
        # Process commands timer
        self.create_timer(0.1, self._process_commands)
    
    def _listen_loop(self):
        """Background thread for continuous listening"""
        while self.listening:
            try:
                with self.microphone as source:
                    self.get_logger().debug("Listening...")
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=10)
                
                try:
                    # Use Google Speech Recognition
                    text = self.recognizer.recognize_google(audio).lower()
                    self.get_logger().info(f"Recognized: {text}")
                    self.command_queue.put(text)
                except sr.UnknownValueError:
                    pass  # Could not understand audio
                except sr.RequestError as e:
                    self.get_logger().error(f"Speech recognition error: {e}")
                    
            except sr.WaitTimeoutError:
                pass  # No speech detected
            except Exception as e:
                self.get_logger().error(f"Listening error: {e}")
                time.sleep(1)
    
    def _process_commands(self):
        """Process queued voice commands"""
        while not self.command_queue.empty():
            text = self.command_queue.get()
            
            # Publish raw text
            raw_msg = String()
            raw_msg.data = text
            self.raw_text_pub.publish(raw_msg)
            
            # Map to command
            command = self._map_command(text)
            if command:
                cmd_msg = String()
                cmd_msg.data = command
                self.command_pub.publish(cmd_msg)
                self.get_logger().info(f"Command: {command}")
    
    def _map_command(self, text):
        """Map spoken text to robot command"""
        text = text.lower().strip()
        
        # Direct mapping
        if text in self.command_mapping:
            return self.command_mapping[text]
        
        # Partial matching
        for phrase, command in self.command_mapping.items():
            if phrase in text:
                return command
        
        # Return raw text for AI processing
        return f"raw:{text}"
    
    def destroy_node(self):
        self.listening = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    voice_listener = VoiceCommandListener()
    
    try:
        rclpy.spin(voice_listener)
    except KeyboardInterrupt:
        pass
    finally:
        voice_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYTHON_EOF

    chmod +x ~/rosmaster_a1_scripts/voice_listener.py
    print_info "Voice listener created at ~/rosmaster_a1_scripts/voice_listener.py"
}

# Create voice command executor
create_voice_executor() {
    print_info "Creating voice command executor..."
    
    mkdir -p ~/rosmaster_a1_scripts
    
    cat > ~/rosmaster_a1_scripts/voice_executor.py << 'PYTHON_EOF'
#!/usr/bin/env python3
"""
ROSMASTER A1 Voice Command Executor
Receives voice commands and executes robot actions
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import subprocess
import pyttsx3
import threading


class VoiceCommandExecutor(Node):
    def __init__(self):
        super().__init__('voice_command_executor')
        
        # Subscriber for voice commands
        self.command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10
        )
        
        # Publisher for robot velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for navigation goals
        self.nav_goal_pub = self.create_publisher(String, '/navigation_goal', 10)
        
        # Publisher for robot actions
        self.action_pub = self.create_publisher(String, '/robot_action', 10)
        
        # Text-to-speech engine
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)
        
        # Movement parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        
        # Navigation points (customize for your map)
        self.nav_points = {
            'navigation_kitchen': 'A',
            'navigation_bedroom': 'B',
            'navigation_living_room': 'C',
            'navigation_home': 'zero',
        }
        
        self.get_logger().info("Voice Command Executor initialized!")
        self.speak("Voice command system ready")
    
    def speak(self, text):
        """Text-to-speech output"""
        def _speak():
            try:
                self.tts_engine.say(text)
                self.tts_engine.runAndWait()
            except Exception as e:
                self.get_logger().error(f"TTS error: {e}")
        
        # Run TTS in separate thread to not block
        threading.Thread(target=_speak, daemon=True).start()
    
    def command_callback(self, msg):
        """Handle voice commands"""
        command = msg.data
        self.get_logger().info(f"Executing command: {command}")
        
        # Movement commands
        if command == 'move_forward':
            self.move_robot(self.linear_speed, 0.0)
            self.speak("Moving forward")
            
        elif command == 'move_backward':
            self.move_robot(-self.linear_speed, 0.0)
            self.speak("Moving backward")
            
        elif command == 'turn_left':
            self.move_robot(0.0, self.angular_speed)
            self.speak("Turning left")
            
        elif command == 'turn_right':
            self.move_robot(0.0, -self.angular_speed)
            self.speak("Turning right")
            
        elif command == 'stop':
            self.move_robot(0.0, 0.0)
            self.speak("Stopping")
            
        # Navigation commands
        elif command in self.nav_points:
            nav_point = self.nav_points[command]
            self.navigate_to(nav_point)
            location = command.replace('navigation_', '').replace('_', ' ')
            self.speak(f"Navigating to {location}")
            
        # Vision commands
        elif command == 'seewhat':
            self.execute_action('seewhat()')
            self.speak("Looking around")
            
        elif command == 'capture_image':
            self.execute_action('capture_image()')
            self.speak("Taking a picture")
            
        # Following commands
        elif command == 'face_follow':
            self.execute_action('face_follow()')
            self.speak("Starting face following")
            
        elif command == 'color_follow':
            self.execute_action('colorFollow(red)')
            self.speak("Starting color following")
            
        elif command == 'stop_follow':
            self.execute_action('stop_follow()')
            self.speak("Stopping follow mode")
            
        # Line following
        elif command == 'follow_line':
            self.execute_action('follow_line(green)')
            self.speak("Starting line patrol")
            
        # System commands
        elif command == 'greeting':
            self.speak("Hello! I am ROSMASTER A1 robot. How can I help you?")
            
        elif command == 'finish_dialogue':
            self.speak("Goodbye! Call me if you need anything.")
            self.execute_action('finish_dialogue()')
            
        elif command == 'shutdown':
            self.speak("Shutting down voice control")
            
        # Raw command for AI processing
        elif command.startswith('raw:'):
            raw_text = command[4:]
            self.process_natural_language(raw_text)
    
    def move_robot(self, linear, angular):
        """Publish velocity command"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        
        # Auto-stop after 2 seconds for safety
        self.create_timer(2.0, self.stop_robot_once, oneshot=True)
    
    def stop_robot_once(self):
        """Stop robot (one-shot timer callback)"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def navigate_to(self, point):
        """Send navigation goal"""
        msg = String()
        msg.data = f"navigation({point})"
        self.nav_goal_pub.publish(msg)
        self.action_pub.publish(msg)
    
    def execute_action(self, action):
        """Execute robot action"""
        msg = String()
        msg.data = action
        self.action_pub.publish(msg)
    
    def process_natural_language(self, text):
        """Process natural language commands with AI"""
        self.get_logger().info(f"Processing: {text}")
        # This can be extended to use the AI large model
        # For now, just acknowledge
        self.speak(f"I heard: {text}")


def main(args=None):
    rclpy.init(args=args)
    
    executor = VoiceCommandExecutor()
    
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYTHON_EOF

    chmod +x ~/rosmaster_a1_scripts/voice_executor.py
    print_info "Voice executor created at ~/rosmaster_a1_scripts/voice_executor.py"
}

# Launch voice control system
launch_voice_control() {
    print_info "Launching voice control system..."
    
    source_ros2
    
    # Check if custom voice launch exists
    if ros2 pkg list | grep -q "yahboomcar"; then
        # Try to launch built-in voice control
        if ros2 launch yahboomcar_bringup voice_control_launch.py 2>/dev/null; then
            return 0
        fi
    fi
    
    # Launch custom voice control
    print_info "Starting voice listener..."
    python3 ~/rosmaster_a1_scripts/voice_listener.py &
    LISTENER_PID=$!
    
    sleep 2
    
    print_info "Starting voice executor..."
    python3 ~/rosmaster_a1_scripts/voice_executor.py &
    EXECUTOR_PID=$!
    
    print_info "Voice control system running!"
    print_info "Listener PID: $LISTENER_PID"
    print_info "Executor PID: $EXECUTOR_PID"
    
    # Wait for interrupt
    wait
}

# Show usage
show_usage() {
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  install      Install voice recognition dependencies"
    echo "  check        Check audio devices"
    echo "  configure    Configure audio system"
    echo "  test-mic     Test microphone"
    echo "  test-speak   Test text-to-speech"
    echo "  setup        Create voice control scripts"
    echo "  launch       Launch voice control system"
    echo "  help         Show this help message"
    echo ""
    echo "Environment variables:"
    echo "  LANGUAGE     Language for voice recognition (default: en)"
}

# Main execution
print_header

case "${1:-help}" in
    install)
        install_voice_deps
        ;;
    check)
        check_audio_devices
        ;;
    configure)
        configure_audio
        ;;
    test-mic)
        test_microphone
        ;;
    test-speak)
        test_speaker
        ;;
    setup)
        create_voice_listener
        create_voice_executor
        ;;
    launch)
        launch_voice_control
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
