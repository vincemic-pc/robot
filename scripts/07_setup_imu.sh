#!/bin/bash
#===============================================================================
# ROSMASTER A1 - IMU Setup and Integration
# Detects, configures, and integrates IMU sensor for improved odometry
# Supports: MPU9250, MPU6050, BNO055, ICM20948
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
    echo "║   ROSMASTER A1 - IMU Setup & Integration   ║"
    echo "║   Inertial Measurement Unit Configuration  ║"
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

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
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

# Check if IMU exists
check_imu() {
    print_info "Checking for IMU sensor..."
    echo ""
    
    source_ros2
    
    # Check ROS2 topics for IMU
    echo "=== Checking ROS2 IMU Topics ==="
    if ros2 topic list 2>/dev/null | grep -qi "imu"; then
        print_success "IMU topics detected!"
        ros2 topic list | grep -i imu
        echo ""
        
        # Try to get IMU data
        echo "=== IMU Data Sample ==="
        timeout 3 ros2 topic echo /imu/data --once 2>/dev/null || \
        timeout 3 ros2 topic echo /imu --once 2>/dev/null || \
        timeout 3 ros2 topic echo /imu/data_raw --once 2>/dev/null || \
        print_warning "Could not read IMU data (topic may not be publishing)"
    else
        print_warning "No IMU topics found in ROS2"
    fi
    
    echo ""
    echo "=== Checking I2C Devices ==="
    # Check I2C bus for IMU (common addresses)
    if command -v i2cdetect &> /dev/null; then
        echo "I2C Bus 0:"
        sudo i2cdetect -y 0 2>/dev/null || echo "  Bus 0 not available"
        echo ""
        echo "I2C Bus 1:"
        sudo i2cdetect -y 1 2>/dev/null || echo "  Bus 1 not available"
        
        echo ""
        echo "Common IMU I2C addresses:"
        echo "  0x68 - MPU6050/MPU9250/ICM20948"
        echo "  0x69 - MPU6050/MPU9250 (alternate)"
        echo "  0x28 - BNO055"
        echo "  0x29 - BNO055 (alternate)"
    else
        print_warning "i2cdetect not found. Install with: sudo apt install i2c-tools"
    fi
    
    echo ""
    echo "=== Checking Serial Devices ==="
    ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  No USB serial devices found"
    
    echo ""
    echo "=== Checking for IMU in dmesg ==="
    dmesg | grep -i "imu\|mpu\|accelerometer\|gyroscope" | tail -5 || echo "  No IMU references in dmesg"
}

# Install IMU dependencies
install_deps() {
    print_info "Installing IMU dependencies..."
    
    # System packages
    sudo apt-get update
    sudo apt-get install -y \
        i2c-tools \
        libi2c-dev \
        python3-smbus
    
    # ROS2 packages
    sudo apt-get install -y \
        ros-humble-imu-tools \
        ros-humble-robot-localization \
        ros-humble-imu-filter-madgwick \
        ros-humble-imu-complementary-filter
    
    # Python packages for custom IMU drivers
    pip3 install --user \
        smbus2 \
        mpu6050-raspberrypi \
        adafruit-circuitpython-bno055 \
        adafruit-circuitpython-mpu6050
    
    print_success "IMU dependencies installed!"
}

# Create IMU driver node
create_imu_driver() {
    print_info "Creating IMU driver node..."
    
    mkdir -p ~/rosmaster_a1_scripts
    
    cat > ~/rosmaster_a1_scripts/imu_driver.py << 'PYTHON_EOF'
#!/usr/bin/env python3
"""
IMU Driver Node for ROSMASTER A1
Supports MPU6050, MPU9250, BNO055, ICM20948
Publishes sensor_msgs/Imu to /imu/data
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion
import math
import time
from typing import Optional, Tuple

# Try importing different IMU libraries
MPU6050_AVAILABLE = False
BNO055_AVAILABLE = False
SMBUS_AVAILABLE = False

try:
    from mpu6050 import mpu6050
    MPU6050_AVAILABLE = True
except ImportError:
    pass

try:
    import board
    import adafruit_bno055
    BNO055_AVAILABLE = True
except ImportError:
    pass

try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    pass


class MPU6050Driver:
    """Driver for MPU6050/MPU9250 IMU"""
    
    def __init__(self, address: int = 0x68, bus: int = 1):
        self.address = address
        self.bus = bus
        
        if MPU6050_AVAILABLE:
            self.sensor = mpu6050(address)
        elif SMBUS_AVAILABLE:
            self.bus_obj = smbus2.SMBus(bus)
            self._init_mpu6050()
        else:
            raise RuntimeError("No MPU6050 library available")
    
    def _init_mpu6050(self):
        """Initialize MPU6050 via raw I2C"""
        # Wake up MPU6050
        self.bus_obj.write_byte_data(self.address, 0x6B, 0x00)
        time.sleep(0.1)
        # Set accelerometer range to ±2g
        self.bus_obj.write_byte_data(self.address, 0x1C, 0x00)
        # Set gyroscope range to ±250°/s
        self.bus_obj.write_byte_data(self.address, 0x1B, 0x00)
    
    def read_raw_data(self, addr: int) -> int:
        """Read raw 16-bit signed value"""
        high = self.bus_obj.read_byte_data(self.address, addr)
        low = self.bus_obj.read_byte_data(self.address, addr + 1)
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value
    
    def get_data(self) -> dict:
        """Get accelerometer and gyroscope data"""
        if MPU6050_AVAILABLE:
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()
            return {
                'accel': {'x': accel['x'], 'y': accel['y'], 'z': accel['z']},
                'gyro': {'x': gyro['x'], 'y': gyro['y'], 'z': gyro['z']}
            }
        else:
            # Raw I2C read
            ax = self.read_raw_data(0x3B) / 16384.0 * 9.81  # Convert to m/s²
            ay = self.read_raw_data(0x3D) / 16384.0 * 9.81
            az = self.read_raw_data(0x3F) / 16384.0 * 9.81
            gx = self.read_raw_data(0x43) / 131.0 * (math.pi / 180)  # Convert to rad/s
            gy = self.read_raw_data(0x45) / 131.0 * (math.pi / 180)
            gz = self.read_raw_data(0x47) / 131.0 * (math.pi / 180)
            
            return {
                'accel': {'x': ax, 'y': ay, 'z': az},
                'gyro': {'x': gx, 'y': gy, 'z': gz}
            }


class BNO055Driver:
    """Driver for BNO055 9-DOF IMU"""
    
    def __init__(self):
        if not BNO055_AVAILABLE:
            raise RuntimeError("BNO055 library not available")
        
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
    
    def get_data(self) -> dict:
        """Get all IMU data including orientation"""
        accel = self.sensor.acceleration or (0, 0, 0)
        gyro = self.sensor.gyro or (0, 0, 0)
        quat = self.sensor.quaternion or (1, 0, 0, 0)
        mag = self.sensor.magnetic or (0, 0, 0)
        
        return {
            'accel': {'x': accel[0], 'y': accel[1], 'z': accel[2]},
            'gyro': {'x': gyro[0], 'y': gyro[1], 'z': gyro[2]},
            'quaternion': {'w': quat[0], 'x': quat[1], 'y': quat[2], 'z': quat[3]},
            'mag': {'x': mag[0], 'y': mag[1], 'z': mag[2]}
        }


class IMUDriverNode(Node):
    """ROS2 Node for IMU data publishing"""
    
    def __init__(self):
        super().__init__('imu_driver')
        
        # Parameters
        self.declare_parameter('imu_type', 'auto')  # auto, mpu6050, bno055
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 100.0)
        
        self.imu_type = self.get_parameter('imu_type').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.imu_raw_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        
        # Initialize IMU driver
        self.driver = None
        self.has_orientation = False
        self._init_driver()
        
        if self.driver is None:
            self.get_logger().error("No IMU detected! Check connections.")
            return
        
        # Timer for publishing
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.publish_imu_data)
        
        self.get_logger().info(f"IMU driver started: {self.imu_type}")
    
    def _init_driver(self):
        """Initialize the appropriate IMU driver"""
        
        if self.imu_type == 'auto':
            # Try BNO055 first (has orientation)
            try:
                self.driver = BNO055Driver()
                self.imu_type = 'bno055'
                self.has_orientation = True
                self.get_logger().info("Detected BNO055 IMU")
                return
            except Exception as e:
                self.get_logger().debug(f"BNO055 not found: {e}")
            
            # Try MPU6050/MPU9250
            try:
                self.driver = MPU6050Driver(self.i2c_address, self.i2c_bus)
                self.imu_type = 'mpu6050'
                self.has_orientation = False
                self.get_logger().info("Detected MPU6050/MPU9250 IMU")
                return
            except Exception as e:
                self.get_logger().debug(f"MPU6050 not found: {e}")
        
        elif self.imu_type == 'bno055':
            self.driver = BNO055Driver()
            self.has_orientation = True
        
        elif self.imu_type == 'mpu6050':
            self.driver = MPU6050Driver(self.i2c_address, self.i2c_bus)
            self.has_orientation = False
    
    def publish_imu_data(self):
        """Read IMU and publish data"""
        if self.driver is None:
            return
        
        try:
            data = self.driver.get_data()
        except Exception as e:
            self.get_logger().warning(f"IMU read error: {e}")
            return
        
        now = self.get_clock().now().to_msg()
        
        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id
        
        # Linear acceleration
        imu_msg.linear_acceleration.x = data['accel']['x']
        imu_msg.linear_acceleration.y = data['accel']['y']
        imu_msg.linear_acceleration.z = data['accel']['z']
        
        # Angular velocity
        imu_msg.angular_velocity.x = data['gyro']['x']
        imu_msg.angular_velocity.y = data['gyro']['y']
        imu_msg.angular_velocity.z = data['gyro']['z']
        
        # Orientation (if available)
        if self.has_orientation and 'quaternion' in data:
            q = data['quaternion']
            imu_msg.orientation.w = q['w']
            imu_msg.orientation.x = q['x']
            imu_msg.orientation.y = q['y']
            imu_msg.orientation.z = q['z']
            # Covariance - known orientation
            imu_msg.orientation_covariance[0] = 0.01
            imu_msg.orientation_covariance[4] = 0.01
            imu_msg.orientation_covariance[8] = 0.01
        else:
            # Unknown orientation
            imu_msg.orientation_covariance[0] = -1
        
        # Covariances for accel and gyro
        imu_msg.linear_acceleration_covariance[0] = 0.1
        imu_msg.linear_acceleration_covariance[4] = 0.1
        imu_msg.linear_acceleration_covariance[8] = 0.1
        
        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[4] = 0.01
        imu_msg.angular_velocity_covariance[8] = 0.01
        
        # Publish
        self.imu_pub.publish(imu_msg)
        self.imu_raw_pub.publish(imu_msg)
        
        # Magnetometer (if available)
        if 'mag' in data:
            mag_msg = MagneticField()
            mag_msg.header.stamp = now
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = data['mag']['x']
            mag_msg.magnetic_field.y = data['mag']['y']
            mag_msg.magnetic_field.z = data['mag']['z']
            self.mag_pub.publish(mag_msg)


def main():
    rclpy.init()
    node = IMUDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYTHON_EOF

    chmod +x ~/rosmaster_a1_scripts/imu_driver.py
    print_success "IMU driver node created!"
}

# Create sensor fusion node using robot_localization
create_sensor_fusion() {
    print_info "Creating sensor fusion configuration..."
    
    mkdir -p ~/rosmaster_a1_config
    
    # EKF configuration for robot_localization
    cat > ~/rosmaster_a1_config/ekf_config.yaml << 'YAML_EOF'
# Extended Kalman Filter configuration for ROSMASTER A1
# Fuses wheel odometry with IMU data for improved localization

ekf_filter_node:
  ros__parameters:
    # Frequency of filter updates
    frequency: 30.0
    
    # Sensor timeout
    sensor_timeout: 0.1
    
    # Frames
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    
    # Publish tf
    publish_tf: true
    
    # Initial state
    two_d_mode: true
    
    # ============ WHEEL ODOMETRY ============
    odom0: /odom
    odom0_config: [true,  true,  false,   # x, y, z position
                   false, false, true,    # roll, pitch, yaw
                   true,  true,  false,   # x, y, z velocity
                   false, false, true,    # roll, pitch, yaw velocity
                   false, false, false]   # x, y, z acceleration
    odom0_queue_size: 10
    odom0_differential: false
    odom0_relative: false
    
    # ============ IMU DATA ============
    imu0: /imu/data
    imu0_config: [false, false, false,   # x, y, z position (IMU doesn't provide)
                  true,  true,  true,    # roll, pitch, yaw orientation
                  false, false, false,   # x, y, z velocity
                  true,  true,  true,    # roll, pitch, yaw velocity
                  true,  true,  true]    # x, y, z acceleration
    imu0_queue_size: 10
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true
    
    # Process noise covariance
    process_noise_covariance: [0.05,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                               0.0,    0.05,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                               0.0,    0.0,    0.06,   0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                               0.0,    0.0,    0.0,    0.03,   0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                               0.0,    0.0,    0.0,    0.0,    0.03,   0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                               0.0,    0.0,    0.0,    0.0,    0.0,    0.06,   0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                               0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.025,   0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                               0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.025,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                               0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.04,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                               0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.01,   0.0,    0.0,    0.0,    0.0,    0.0,
                               0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.01,   0.0,    0.0,    0.0,    0.0,
                               0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.02,   0.0,    0.0,    0.0,
                               0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.01,   0.0,    0.0,
                               0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   0.0,
                               0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.015]

YAML_EOF

    # IMU filter configuration (Madgwick)
    cat > ~/rosmaster_a1_config/imu_filter.yaml << 'YAML_EOF'
# IMU Filter (Madgwick) configuration
# Computes orientation from raw accelerometer and gyroscope data

imu_filter_madgwick_node:
  ros__parameters:
    use_mag: false
    publish_tf: false
    world_frame: "enu"
    fixed_frame: "odom"
    gain: 0.1
    zeta: 0.0
    
    # Orientation variance
    orientation_stddev: 0.02
YAML_EOF

    print_success "Sensor fusion configuration created!"
}

# Create IMU calibration script
create_calibration() {
    print_info "Creating IMU calibration script..."
    
    cat > ~/rosmaster_a1_scripts/imu_calibrate.py << 'PYTHON_EOF'
#!/usr/bin/env python3
"""
IMU Calibration Tool
Calculates bias offsets for accelerometer and gyroscope
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import yaml
import os


class IMUCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')
        
        self.samples = []
        self.target_samples = 500
        
        self.sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10
        )
        
        self.get_logger().info("IMU Calibration Tool")
        self.get_logger().info("Keep robot STATIONARY on flat surface!")
        self.get_logger().info(f"Collecting {self.target_samples} samples...")
    
    def imu_callback(self, msg: Imu):
        if len(self.samples) >= self.target_samples:
            return
        
        self.samples.append({
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
            'gx': msg.angular_velocity.x,
            'gy': msg.angular_velocity.y,
            'gz': msg.angular_velocity.z
        })
        
        if len(self.samples) % 50 == 0:
            self.get_logger().info(f"Collected {len(self.samples)}/{self.target_samples}")
        
        if len(self.samples) >= self.target_samples:
            self.compute_calibration()
    
    def compute_calibration(self):
        self.get_logger().info("Computing calibration...")
        
        ax = np.mean([s['ax'] for s in self.samples])
        ay = np.mean([s['ay'] for s in self.samples])
        az = np.mean([s['az'] for s in self.samples])
        gx = np.mean([s['gx'] for s in self.samples])
        gy = np.mean([s['gy'] for s in self.samples])
        gz = np.mean([s['gz'] for s in self.samples])
        
        # Gyro should be ~0 when stationary
        gyro_bias = {'x': gx, 'y': gy, 'z': gz}
        
        # Accel should be [0, 0, 9.81] when stationary and level
        accel_bias = {
            'x': ax - 0.0,
            'y': ay - 0.0,
            'z': az - 9.81
        }
        
        calibration = {
            'imu_calibration': {
                'gyro_bias': gyro_bias,
                'accel_bias': accel_bias,
                'samples': self.target_samples
            }
        }
        
        # Save calibration
        config_dir = os.path.expanduser('~/rosmaster_a1_config')
        os.makedirs(config_dir, exist_ok=True)
        
        cal_file = os.path.join(config_dir, 'imu_calibration.yaml')
        with open(cal_file, 'w') as f:
            yaml.dump(calibration, f)
        
        self.get_logger().info("=== Calibration Results ===")
        self.get_logger().info(f"Gyro bias: x={gx:.6f}, y={gy:.6f}, z={gz:.6f} rad/s")
        self.get_logger().info(f"Accel bias: x={accel_bias['x']:.4f}, y={accel_bias['y']:.4f}, z={accel_bias['z']:.4f} m/s²")
        self.get_logger().info(f"Saved to: {cal_file}")
        
        rclpy.shutdown()


def main():
    rclpy.init()
    node = IMUCalibrator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
PYTHON_EOF

    chmod +x ~/rosmaster_a1_scripts/imu_calibrate.py
    print_success "IMU calibration script created!"
}

# Launch IMU driver
launch_imu() {
    print_info "Launching IMU driver..."
    
    source_ros2
    
    python3 ~/rosmaster_a1_scripts/imu_driver.py \
        --ros-args \
        -p imu_type:=auto \
        -p publish_rate:=100.0
}

# Launch sensor fusion
launch_fusion() {
    print_info "Launching sensor fusion (EKF)..."
    
    source_ros2
    
    ros2 launch robot_localization ekf.launch.py \
        params_file:=${HOME}/rosmaster_a1_config/ekf_config.yaml
}

# Run calibration
run_calibration() {
    print_info "Starting IMU calibration..."
    print_warning "KEEP THE ROBOT STATIONARY ON A FLAT SURFACE!"
    sleep 3
    
    source_ros2
    
    python3 ~/rosmaster_a1_scripts/imu_calibrate.py
}

# Monitor IMU data
monitor_imu() {
    print_info "Monitoring IMU data..."
    
    source_ros2
    
    echo "=== IMU Data Stream ==="
    echo "Press Ctrl+C to stop"
    echo ""
    
    ros2 topic echo /imu/data
}

# Show usage
show_usage() {
    print_header
    echo ""
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Detection:"
    echo "  check          Check if IMU is present"
    echo "  monitor        View live IMU data"
    echo ""
    echo "Setup:"
    echo "  install        Install IMU dependencies"
    echo "  setup          Create all IMU nodes and configs"
    echo "  calibrate      Run IMU calibration"
    echo ""
    echo "Run:"
    echo "  launch         Launch IMU driver"
    echo "  fusion         Launch sensor fusion (EKF)"
    echo ""
    echo "Supported IMUs:"
    echo "  - MPU6050 (6-axis)"
    echo "  - MPU9250 (9-axis)"
    echo "  - BNO055 (9-axis with orientation)"
    echo "  - ICM20948 (9-axis)"
}

# Main
print_header

case "${1:-help}" in
    check)
        check_imu
        ;;
    install)
        install_deps
        ;;
    setup)
        create_imu_driver
        create_sensor_fusion
        create_calibration
        print_success "IMU setup complete!"
        ;;
    calibrate)
        run_calibration
        ;;
    launch)
        launch_imu
        ;;
    fusion)
        launch_fusion
        ;;
    monitor)
        monitor_imu
        ;;
    *)
        show_usage
        ;;
esac
