import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist
from enum import Enum
import threading
import json

class CameraType(Enum):
    OAK_D_PRO = "oakd"
    REALSENSE = "realsense"

class SlamMode(Enum):
    LIDAR = "lidar"
    VSLAM = "vslam"
    HYBRID = "hybrid"

class VoiceMapper(Node):
    def __init__(self):
        super().__init__('voice_mapper')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.rgb_sub = self.create_subscription(Image, '/oak/rgb/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/oak/stereo/image_raw', self.depth_callback, 10)
        self.voice_command_sub = self.create_subscription(String, '/voice_command', self.voice_command_callback, 10)

        self.lock = threading.Lock()
        self.llm_response = None

    def scan_callback(self, msg):
        # Process LiDAR data
        pass

    def imu_callback(self, msg):
        # Process IMU data
        pass

    def rgb_callback(self, msg):
        # Process RGB camera data
        pass

    def depth_callback(self, msg):
        # Process depth camera data
        pass

    def voice_command_callback(self, msg):
        command = msg.data
        self.execute_command(command)

    def execute_command(self, command):
        # Execute the voice command
        action_data = self.parse_command(command)
        if action_data:
            self.perform_action(action_data)

    def parse_command(self, command):
        # Parse the command and return action data
        try:
            return json.loads(command)
        except json.JSONDecodeError:
            return None

    def perform_action(self, action_data):
        # Perform the action based on parsed data
        action = action_data.get("action")
        if action == "move_forward":
            self.move_forward()
        elif action == "turn_left":
            self.turn_left()
        elif action == "turn_right":
            self.turn_right()
        # Add more actions as needed

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.5
        self.cmd_vel_pub.publish(twist)

    def turn_left(self):
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)

    def turn_right(self):
        twist = Twist()
        twist.angular.z = -0.5
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    voice_mapper = VoiceMapper()
    rclpy.spin(voice_mapper)
    voice_mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()