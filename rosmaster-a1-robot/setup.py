from setuptools import setup

package_name = 'rosmaster_a1_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'geometry_msgs',
        'nav_msgs',
        'opencv-python',
        'numpy',
        'pyaudio',
        'depthai',
        'slam_toolbox',
        'isaac_ros_visual_slam',
    ],
    entry_points={
        'console_scripts': [
            'voice_mapper = scripts.voice_mapper:main',
            'yahboom_explorer = scripts.yahboom_explorer:main',
            'rosmaster_control = scripts.rosmaster_control:main',
            'start_robot = scripts.start_robot:main',
        ],
    },
)