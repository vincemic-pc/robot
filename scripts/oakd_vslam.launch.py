# oakd_vslam.launch.py — Isaac VSLAM for OAK-D Pro stereo camera
# Follows the pattern from isaac_ros_visual_slam_realsense.launch.py
# All params/remappings on ComposableNode — no launch arguments needed.
#
# OAK-D Pro OV9282 outputs mono8 1280x720 (720P) only.
# Isaac ROS NitrosSubscriber negotiates bgr8 (3-ch) format but the camera
# publishes mono8 (1-ch), causing cudaMemcpy2D pitch mismatch.
# Fix: ImageFormatConverterNode converts mono8 → rgb8 before VSLAM, AND
# ResizeNode scales 1280x720 → 640x480 to stay within USB 2.0 bandwidth.
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Convert left mono8 → rgb8 (match Nitros negotiated format)
    convert_left = ComposableNode(
        name='convert_left',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        parameters=[{
            'encoding_desired': 'rgb8',
            'image_width': 1280,
            'image_height': 720,
        }],
        remappings=[
            ('image_raw', '/oak/left/image_rect'),
            ('image', '/vslam/left/image_rgb'),
        ],
    )
    # Convert right mono8 → rgb8
    convert_right = ComposableNode(
        name='convert_right',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        parameters=[{
            'encoding_desired': 'rgb8',
            'image_width': 1280,
            'image_height': 720,
        }],
        remappings=[
            ('image_raw', '/oak/right/image_rect'),
            ('image', '/vslam/right/image_rgb'),
        ],
    )
    # Resize left rgb8: 1280x720 → 640x480
    resize_left = ComposableNode(
        name='resize_left',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'output_width': 640,
            'output_height': 480,
        }],
        remappings=[
            ('image', '/vslam/left/image_rgb'),
            ('camera_info', '/oak/left/camera_info'),
            ('resize/image', '/vslam/left/image_resized'),
            ('resize/camera_info', '/vslam/left/camera_info_resized'),
        ],
    )
    # Resize right rgb8: 1280x720 → 640x480
    resize_right = ComposableNode(
        name='resize_right',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'output_width': 640,
            'output_height': 480,
        }],
        remappings=[
            ('image', '/vslam/right/image_rgb'),
            ('camera_info', '/oak/right/camera_info'),
            ('resize/image', '/vslam/right/image_resized'),
            ('resize/camera_info', '/vslam/right/camera_info_resized'),
        ],
    )
    # Isaac VSLAM node — reads resized rgb8 images
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
            'image_jitter_threshold_ms': 200.0,
        }],
        remappings=[
            ('visual_slam/image_0', '/vslam/left/image_resized'),
            ('visual_slam/camera_info_0', '/vslam/left/camera_info_resized'),
            ('visual_slam/image_1', '/vslam/right/image_resized'),
            ('visual_slam/camera_info_1', '/vslam/right/camera_info_resized'),
            ('visual_slam/imu', '/oak/imu/data'),
        ],
    )
    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            convert_left,
            convert_right,
            resize_left,
            resize_right,
            visual_slam_node,
        ],
        output='screen',
    )
    return launch.LaunchDescription([visual_slam_launch_container])
