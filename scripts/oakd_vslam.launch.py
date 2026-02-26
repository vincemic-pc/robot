# oakd_vslam.launch.py — Isaac VSLAM for OAK-D Pro stereo camera
# Follows the pattern from isaac_ros_visual_slam_realsense.launch.py
# All params/remappings on ComposableNode — no launch arguments needed.
#
# OAK-D Pro OV9282 outputs mono8 1280x720 (720P) only.
# Isaac ROS ImageFormatConverterNode CUDA converter crashes with
# cudaMemcpy2D pitch mismatch on mono8→rgb8 when run as composable nodes.
# Workaround: Feed mono8 directly to VSLAM — cuVSLAM 3.2.x accepts mono8
# images via NitrosImage negotiation (falls back to sensor_msgs/Image).
# Stereo now at 15fps (reduced from 30fps for RGBD pipeline stability).
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Isaac VSLAM node — reads mono8 720P stereo directly (no format conversion)
    # cuVSLAM internally converts to grayscale for feature extraction anyway.
    # Stereo at 15fps — image_jitter_threshold raised for longer intervals.
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'num_cameras': 2,
            'enable_image_denoising': False,
            'rectified_images': True,
            'enable_imu_fusion': True,
            'enable_localization_n_mapping': True,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'base_frame': 'base_link',
            'imu_frame': 'oak_imu_frame',
            'camera_optical_frames': [
                'oak_left_camera_optical_frame',
                'oak_right_camera_optical_frame',
            ],
            # IMU noise params — RealSense reference values as starting point
            # TODO: Characterize OAK-D Pro BMI270 via Allan variance
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': 350.0,  # stereo ~7-10fps with transient drops, generous margin
        }],
        remappings=[
            ('visual_slam/image_0', '/oak/left/image_rect'),
            ('visual_slam/camera_info_0', '/oak/left/camera_info'),
            ('visual_slam/image_1', '/oak/right/image_rect'),
            ('visual_slam/camera_info_1', '/oak/right/camera_info'),
            ('visual_slam/imu', '/oak/imu/data'),
        ],
    )
    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node,
        ],
        output='screen',
    )
    return launch.LaunchDescription([visual_slam_launch_container])
