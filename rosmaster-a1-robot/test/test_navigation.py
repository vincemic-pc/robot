import unittest
from rclpy import init, shutdown
from rclpy.node import Node
from nav2_msgs.srv import ComputePathToPose
from geometry_msgs.msg import PoseStamped

class TestNavigation(Node):

    def __init__(self):
        super().__init__('test_navigation')
        self.client = self.create_client(ComputePathToPose, 'compute_path_to_pose')

    def test_compute_path_to_pose(self):
        # Wait for the service to be available
        self.client.wait_for_service()

        # Create a request
        request = ComputePathToPose.Request()
        request.start = PoseStamped()  # Set start pose
        request.goal = PoseStamped()    # Set goal pose

        # Call the service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Check the response
        self.assertTrue(future.result() is not None)
        self.assertTrue(len(future.result().path) > 0)  # Ensure a path is returned

if __name__ == '__main__':
    init()
    unittest.main()
    shutdown()