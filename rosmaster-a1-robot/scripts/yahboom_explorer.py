# This script is a deprecated alternative for exploring with the HP60C camera. It serves as a reference for older implementations.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class YahboomExplorer(Node):
    def __init__(self):
        super().__init__('yahboom_explorer')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/oak/rgb/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        # Process the incoming image data
        self.get_logger().info('Received an image')

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.5  # Move forward at 0.5 m/s
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)  # Stop the robot

def main(args=None):
    rclpy.init(args=args)
    explorer = YahboomExplorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()