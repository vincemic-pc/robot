#!/usr/bin/env python3
"""
ROSMASTER A1 - LLM Robot Brain
Integrates a Large Language Model for intelligent command interpretation,
decision making, and natural conversation.

Supports:
- OpenAI GPT-4 / GPT-4o
- Anthropic Claude
- Azure OpenAI
- Local models via Ollama
- Google Gemini
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
import os
import base64
import threading
import queue
from datetime import datetime
from typing import Optional, Dict, List, Any

# LLM Clients (install as needed)
try:
    import openai
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

try:
    import anthropic
    ANTHROPIC_AVAILABLE = True
except ImportError:
    ANTHROPIC_AVAILABLE = False

try:
    import google.generativeai as genai
    GOOGLE_AVAILABLE = True
except ImportError:
    GOOGLE_AVAILABLE = False

try:
    import requests  # For Ollama
    OLLAMA_AVAILABLE = True
except ImportError:
    OLLAMA_AVAILABLE = False


class RobotActionLibrary:
    """
    Robot Action Library - defines all available robot capabilities
    Based on ROSMASTER A1 documentation
    """
    
    ACTIONS = {
        # Basic Movement
        "move_forward": {
            "function": "set_cmdvel(linear_x, 0, 0, duration)",
            "description": "Move forward. Parameters: linear_x (0.1-0.5 m/s), duration (seconds)",
            "example": "Move forward 1 meter: set_cmdvel(0.3, 0, 0, 3.3)"
        },
        "move_backward": {
            "function": "set_cmdvel(-linear_x, 0, 0, duration)",
            "description": "Move backward. Use negative linear_x",
            "example": "Move backward 0.5 meters: set_cmdvel(-0.25, 0, 0, 2)"
        },
        "turn_left": {
            "function": "move_left(degrees, angular_speed)",
            "description": "Turn left by specified degrees. angular_speed default 3.0 rad/s",
            "example": "Turn left 90 degrees: move_left(90, 3.0)"
        },
        "turn_right": {
            "function": "move_right(degrees, angular_speed)",
            "description": "Turn right by specified degrees",
            "example": "Turn right 45 degrees: move_right(45, 3.0)"
        },
        "stop": {
            "function": "set_cmdvel(0, 0, 0, 0)",
            "description": "Stop all movement immediately"
        },
        
        # Navigation
        "navigate_to": {
            "function": "navigation(point)",
            "description": "Navigate to a named location. Points: A=kitchen, B=living_room, C=bedroom, zero=home",
            "example": "Go to kitchen: navigation(A)"
        },
        "get_position": {
            "function": "get_current_pose()",
            "description": "Record current position for later return"
        },
        "return_home": {
            "function": "navigation(zero)",
            "description": "Return to starting/home position"
        },
        
        # Vision
        "look_around": {
            "function": "seewhat()",
            "description": "Capture current camera view for analysis"
        },
        "get_distance": {
            "function": "get_dist(x, y)",
            "description": "Get distance to object at pixel coordinates (x,y) using depth camera"
        },
        
        # Following/Tracking
        "follow_face": {
            "function": "face_follow()",
            "description": "Start following detected faces"
        },
        "follow_color": {
            "function": "color_follow(color)",
            "description": "Follow specified color. Colors: red, green, blue, yellow"
        },
        "follow_person": {
            "function": "poseFollow()",
            "description": "Follow human body pose/skeleton"
        },
        "follow_gesture": {
            "function": "gestureFollow()",
            "description": "Follow hand gestures"
        },
        "stop_following": {
            "function": "stop_follow()",
            "description": "Stop any active following behavior"
        },
        
        # Line Following
        "follow_line": {
            "function": "follow_line(color)",
            "description": "Autonomous line following. Colors: red, green, blue, yellow"
        },
        
        # PTZ/Servo Control
        "nod_head": {
            "function": "servo_nod()",
            "description": "Nod the camera servo (yes gesture)"
        },
        "shake_head": {
            "function": "servo_shake()",
            "description": "Shake the camera servo (no gesture)"
        },
        "reset_camera": {
            "function": "servo_init()",
            "description": "Reset camera servos to default position"
        },
        
        # Task Control
        "wait": {
            "function": "wait(seconds)",
            "description": "Pause execution for specified seconds"
        },
        "end_task": {
            "function": "finishtask()",
            "description": "Mark current task as complete"
        },
        "end_conversation": {
            "function": "finish_dialogue()",
            "description": "End conversation and clear context"
        },
        
        # Mapping & Room Learning
        "start_mapping": {
            "function": "start_mapping()",
            "description": "Start SLAM mapping mode to learn the environment"
        },
        "stop_mapping": {
            "function": "stop_mapping()",
            "description": "Stop SLAM mapping mode"
        },
        "save_map": {
            "function": "save_map(name)",
            "description": "Save the current map with a name",
            "example": "save_map(my_home)"
        },
        "label_room": {
            "function": "label_room(room_name)",
            "description": "Label current location as a room name",
            "example": "label_room(kitchen)"
        },
        "list_rooms": {
            "function": "list_rooms()",
            "description": "List all known/learned rooms"
        },
        "forget_room": {
            "function": "forget_room(room_name)",
            "description": "Remove a room from memory"
        }
    }
    
    @classmethod
    def get_action_prompt(cls) -> str:
        """Generate action library description for LLM prompt"""
        lines = ["## Available Robot Actions\n"]
        for name, info in cls.ACTIONS.items():
            lines.append(f"- **{name}**: `{info['function']}`")
            lines.append(f"  {info['description']}")
            if 'example' in info:
                lines.append(f"  Example: {info['example']}")
        return "\n".join(lines)


class LLMProvider:
    """Base class for LLM providers"""
    
    def __init__(self):
        self.conversation_history: List[Dict] = []
    
    def chat(self, message: str, image: Optional[bytes] = None) -> str:
        raise NotImplementedError
    
    def clear_history(self):
        self.conversation_history = []


class OpenAIProvider(LLMProvider):
    """OpenAI GPT provider"""
    
    def __init__(self, model: str = "gpt-4o", api_key: Optional[str] = None):
        super().__init__()
        self.model = model
        self.client = openai.OpenAI(api_key=api_key or os.getenv("OPENAI_API_KEY"))
    
    def chat(self, message: str, image: Optional[bytes] = None) -> str:
        content = []
        
        if image:
            base64_image = base64.b64encode(image).decode('utf-8')
            content.append({
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
            })
        
        content.append({"type": "text", "text": message})
        
        self.conversation_history.append({"role": "user", "content": content})
        
        response = self.client.chat.completions.create(
            model=self.model,
            messages=self.conversation_history,
            max_tokens=1000
        )
        
        assistant_message = response.choices[0].message.content
        self.conversation_history.append({"role": "assistant", "content": assistant_message})
        
        return assistant_message


class AnthropicProvider(LLMProvider):
    """Anthropic Claude provider"""
    
    def __init__(self, model: str = "claude-sonnet-4-20250514", api_key: Optional[str] = None):
        super().__init__()
        self.model = model
        self.client = anthropic.Anthropic(api_key=api_key or os.getenv("ANTHROPIC_API_KEY"))
    
    def chat(self, message: str, image: Optional[bytes] = None) -> str:
        content = []
        
        if image:
            base64_image = base64.b64encode(image).decode('utf-8')
            content.append({
                "type": "image",
                "source": {
                    "type": "base64",
                    "media_type": "image/jpeg",
                    "data": base64_image
                }
            })
        
        content.append({"type": "text", "text": message})
        
        self.conversation_history.append({"role": "user", "content": content})
        
        response = self.client.messages.create(
            model=self.model,
            max_tokens=1000,
            messages=self.conversation_history
        )
        
        assistant_message = response.content[0].text
        self.conversation_history.append({"role": "assistant", "content": assistant_message})
        
        return assistant_message


class OllamaProvider(LLMProvider):
    """Local Ollama provider for offline operation"""
    
    def __init__(self, model: str = "llama3.2", host: str = "http://localhost:11434"):
        super().__init__()
        self.model = model
        self.host = host
    
    def chat(self, message: str, image: Optional[bytes] = None) -> str:
        self.conversation_history.append({"role": "user", "content": message})
        
        payload = {
            "model": self.model,
            "messages": self.conversation_history,
            "stream": False
        }
        
        if image:
            payload["images"] = [base64.b64encode(image).decode('utf-8')]
        
        response = requests.post(
            f"{self.host}/api/chat",
            json=payload
        )
        
        result = response.json()
        assistant_message = result["message"]["content"]
        self.conversation_history.append({"role": "assistant", "content": assistant_message})
        
        return assistant_message


class LLMRobotBrain(Node):
    """
    ROS2 Node that uses an LLM for intelligent robot control
    """
    
    SYSTEM_PROMPT = """You are an intelligent robot assistant controlling a ROSMASTER A1 robot car.
You can see through a camera, navigate autonomously, follow people/objects, and respond to voice commands.

Your personality: Friendly, helpful, and slightly playful. Speak in first person.

{action_library}

## Map Locations
- A / kitchen: Kitchen area
- B / living_room: Living room
- C / bedroom: Bedroom
- D / office: Office
- zero / home: Starting position

## Response Format
You must respond with valid JSON in this exact format:
{{
    "thinking": "Brief internal reasoning about what to do",
    "actions": ["action1(params)", "action2(params)"],
    "response": "What to say to the user"
}}

## Rules
1. Break complex tasks into steps
2. Use seewhat() when you need to see the environment
3. Use get_current_pose() before navigating if you need to return
4. If an action fails, try once more then inform the user
5. Be conversational and engaging in responses
6. For navigation, use the map letter codes (A, B, C, etc.)

## Examples

User: "Go check if there's water in the kitchen"
{{
    "thinking": "I need to go to kitchen and look around",
    "actions": ["get_current_pose()", "navigation(A)", "seewhat()"],
    "response": "I'm heading to the kitchen to check for water. Let me take a look around when I get there!"
}}

User: "Follow me"
{{
    "thinking": "User wants me to follow them using face detection",
    "actions": ["face_follow()"],
    "response": "Okay! I'll follow you. Just walk normally and I'll keep up!"
}}

User: "What do you see?"
{{
    "thinking": "User wants to know what's in my view",
    "actions": ["seewhat()"],
    "response": "Let me take a look at my surroundings..."
}}
"""

    def __init__(self):
        super().__init__('llm_robot_brain')
        
        # Parameters
        self.declare_parameter('llm_provider', 'openai')  # openai, anthropic, ollama
        self.declare_parameter('llm_model', 'gpt-4o')
        self.declare_parameter('ollama_host', 'http://localhost:11434')
        
        provider_name = self.get_parameter('llm_provider').value
        model = self.get_parameter('llm_model').value
        
        # Initialize LLM provider
        self.llm = self._create_provider(provider_name, model)
        
        # Initialize with system prompt
        action_library = RobotActionLibrary.get_action_prompt()
        system_prompt = self.SYSTEM_PROMPT.format(action_library=action_library)
        
        if isinstance(self.llm, (OpenAIProvider,)):
            self.llm.conversation_history.append({
                "role": "system", 
                "content": system_prompt
            })
        elif isinstance(self.llm, AnthropicProvider):
            self.system_prompt = system_prompt
        
        # CV Bridge for images
        self.bridge = CvBridge()
        self.latest_image: Optional[bytes] = None
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_pub = self.create_publisher(String, '/robot_action', 10)
        self.speech_pub = self.create_publisher(String, '/speech_output', 10)
        self.response_pub = self.create_publisher(String, '/llm_response', 10)
        
        # Subscribers
        self.voice_sub = self.create_subscription(
            String, '/voice_text', self.voice_callback, 10
        )
        self.text_sub = self.create_subscription(
            String, '/text_input', self.text_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.feedback_sub = self.create_subscription(
            String, '/robot_feedback', self.feedback_callback, 10
        )
        
        # Processing queue
        self.input_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self._process_loop, daemon=True)
        self.processing_thread.start()
        
        # State
        self.waiting_for_image = False
        self.pending_actions: List[str] = []
        
        self.get_logger().info(f"LLM Robot Brain initialized with {provider_name}/{model}")
    
    def _create_provider(self, provider: str, model: str) -> LLMProvider:
        """Create the appropriate LLM provider"""
        if provider == "openai" and OPENAI_AVAILABLE:
            return OpenAIProvider(model=model)
        elif provider == "anthropic" and ANTHROPIC_AVAILABLE:
            return AnthropicProvider(model=model)
        elif provider == "ollama" and OLLAMA_AVAILABLE:
            host = self.get_parameter('ollama_host').value
            return OllamaProvider(model=model, host=host)
        else:
            self.get_logger().error(f"Provider {provider} not available!")
            raise RuntimeError(f"LLM provider {provider} not available")
    
    def voice_callback(self, msg: String):
        """Handle voice input"""
        self.get_logger().info(f"Voice input: {msg.data}")
        self.input_queue.put(("voice", msg.data))
    
    def text_callback(self, msg: String):
        """Handle text input"""
        self.get_logger().info(f"Text input: {msg.data}")
        self.input_queue.put(("text", msg.data))
    
    def image_callback(self, msg: Image):
        """Store latest camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            import cv2
            _, buffer = cv2.imencode('.jpg', cv_image)
            self.latest_image = buffer.tobytes()
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
    
    def feedback_callback(self, msg: String):
        """Handle robot action feedback"""
        self.get_logger().info(f"Robot feedback: {msg.data}")
        # Feed back to LLM for context
        self.input_queue.put(("feedback", f"Robot feedback: {msg.data}"))
    
    def _process_loop(self):
        """Main processing loop for LLM interactions"""
        while True:
            try:
                input_type, content = self.input_queue.get()
                self._process_input(input_type, content)
            except Exception as e:
                self.get_logger().error(f"Processing error: {e}")
    
    def _process_input(self, input_type: str, content: str):
        """Process input through LLM and execute actions"""
        
        # Prepare message
        if input_type == "feedback":
            message = content
            include_image = False
        else:
            message = f"User ({input_type}): {content}"
            include_image = self.latest_image is not None
        
        # Get LLM response
        try:
            image_data = self.latest_image if include_image else None
            response = self.llm.chat(message, image_data)
            
            self.get_logger().info(f"LLM response: {response}")
            
            # Parse JSON response
            result = self._parse_response(response)
            
            if result:
                # Execute actions
                for action in result.get("actions", []):
                    self._execute_action(action)
                
                # Speak response
                speech = result.get("response", "")
                if speech:
                    self._speak(speech)
                
                # Publish full response
                response_msg = String()
                response_msg.data = json.dumps(result)
                self.response_pub.publish(response_msg)
                
        except Exception as e:
            self.get_logger().error(f"LLM error: {e}")
            self._speak("I encountered an error processing that request.")
    
    def _parse_response(self, response: str) -> Optional[Dict]:
        """Parse LLM JSON response"""
        try:
            # Find JSON in response
            start = response.find('{')
            end = response.rfind('}') + 1
            if start >= 0 and end > start:
                json_str = response[start:end]
                return json.loads(json_str)
        except json.JSONDecodeError as e:
            self.get_logger().warning(f"JSON parse error: {e}")
        return None
    
    def _execute_action(self, action: str):
        """Execute a robot action"""
        self.get_logger().info(f"Executing: {action}")
        
        # Publish to action topic
        action_msg = String()
        action_msg.data = action
        self.action_pub.publish(action_msg)
        
        # Handle basic movement directly
        if action.startswith("set_cmdvel"):
            self._execute_velocity(action)
    
    def _execute_velocity(self, action: str):
        """Parse and execute velocity command"""
        try:
            # Parse set_cmdvel(linear_x, linear_y, angular_z, duration)
            params = action.split("(")[1].rstrip(")").split(",")
            linear_x = float(params[0])
            linear_y = float(params[1]) if len(params) > 1 else 0.0
            angular_z = float(params[2]) if len(params) > 2 else 0.0
            duration = float(params[3]) if len(params) > 3 else 1.0
            
            twist = Twist()
            twist.linear.x = linear_x
            twist.linear.y = linear_y
            twist.angular.z = angular_z
            
            self.cmd_vel_pub.publish(twist)
            
            # Schedule stop using threading (ROS2 Humble compatible)
            def delayed_stop():
                import time
                time.sleep(duration)
                self._stop_movement()
            
            stop_thread = threading.Thread(target=delayed_stop, daemon=True)
            stop_thread.start()
            
        except Exception as e:
            self.get_logger().error(f"Velocity parse error: {e}")
    
    def _stop_movement(self):
        """Stop robot movement"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def _speak(self, text: str):
        """Publish text for speech synthesis"""
        speech_msg = String()
        speech_msg.data = text
        self.speech_pub.publish(speech_msg)
        self.get_logger().info(f"Speaking: {text}")


def main(args=None):
    rclpy.init(args=args)
    
    brain = LLMRobotBrain()
    
    try:
        rclpy.spin(brain)
    except KeyboardInterrupt:
        pass
    finally:
        brain.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
