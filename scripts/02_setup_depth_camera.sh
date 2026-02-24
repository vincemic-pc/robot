#!/bin/bash
#===============================================================================
# ROSMASTER A1 - 3D Depth Camera Setup Script
# Sets up depth cameras for the robot
# Supports: OAK-D Pro, Nuwa depth camera, Astra Pro, Realsense cameras
#===============================================================================

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Camera type (oakd, nuwa, astra, realsense)
CAMERA_TYPE="${CAMERA_TYPE:-oakd}"

print_header() {
    echo -e "${BLUE}"
    echo "========================================"
    echo "  ROSMASTER A1 - 3D Camera Setup"
    echo "  Camera Type: ${CAMERA_TYPE}"
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
    
    # Source ROS2 Humble
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        print_info "ROS2 Humble sourced"
    else
        print_error "ROS2 Humble not found!"
        exit 1
    fi
    
    # Source workspace
    if [ -f ~/rosmaster_a1_ws/install/setup.bash ]; then
        source ~/rosmaster_a1_ws/install/setup.bash
        print_info "ROSMASTER A1 workspace sourced"
    elif [ -f ~/yahboomcar_ws/install/setup.bash ]; then
        source ~/yahboomcar_ws/install/setup.bash
        print_info "Yahboom workspace sourced"
    fi
}

# Check if camera is connected
check_camera_connection() {
    print_info "Checking camera connection..."
    
    case ${CAMERA_TYPE} in
        nuwa|astra)
            # Check for USB depth camera
            if lsusb | grep -qi "orbbec\|astra\|nuwa\|2bc5"; then
                print_info "Depth camera detected!"
                lsusb | grep -i "orbbec\|astra\|nuwa\|2bc5" || true
                return 0
            else
                print_warning "No Orbbec/Nuwa/Astra camera detected via USB"
                print_info "Available USB devices:"
                lsusb
                return 1
            fi
            ;;
        realsense)
            if lsusb | grep -qi "intel.*realsense\|8086:0b"; then
                print_info "Intel RealSense camera detected!"
                return 0
            else
                print_warning "No Intel RealSense camera detected"
                return 1
            fi
            ;;
        oakd)
            # Check for Luxonis OAK-D (Movidius VPU, USB vendor ID 03e7)
            if lsusb | grep -qi "03e7"; then
                print_info "OAK-D camera detected (Movidius VPU)!"
                lsusb | grep -i "03e7" || true
                return 0
            else
                print_warning "No OAK-D camera detected (Movidius USB ID 03e7)"
                print_info "Available USB devices:"
                lsusb
                return 1
            fi
            ;;
    esac
}

# Install camera dependencies
install_camera_deps() {
    print_info "Installing camera dependencies..."
    
    sudo apt-get update
    sudo apt-get install -y \
        ros-humble-camera-info-manager \
        ros-humble-image-transport \
        ros-humble-image-transport-plugins \
        ros-humble-compressed-image-transport \
        ros-humble-depth-image-proc \
        ros-humble-image-pipeline \
        ros-humble-cv-bridge \
        ros-humble-vision-opencv \
        libusb-1.0-0-dev \
        libudev-dev
    
    case ${CAMERA_TYPE} in
        nuwa|astra)
            print_info "Installing Orbbec/Astra camera packages..."
            sudo apt-get install -y \
                ros-humble-astra-camera \
                ros-humble-libuvc-camera 2>/dev/null || true
            
            # Install udev rules for Orbbec cameras
            setup_orbbec_udev
            ;;
        realsense)
            print_info "Installing Intel RealSense packages..."
            sudo apt-get install -y \
                ros-humble-realsense2-camera \
                ros-humble-realsense2-description
            ;;
        oakd)
            print_info "Installing OAK-D (DepthAI) camera packages..."
            sudo apt-get install -y \
                ros-humble-depthai-ros-driver \
                ros-humble-depthai-bridge \
                ros-humble-depthai-descriptions 2>/dev/null || true
            # Setup udev rules for Luxonis cameras
            setup_oakd_udev
            ;;
    esac
    
    print_info "Camera dependencies installed!"
}

# Setup udev rules for Orbbec cameras
setup_orbbec_udev() {
    print_info "Setting up udev rules for Orbbec cameras..."
    
    # Create udev rules
    sudo tee /etc/udev/rules.d/56-orbbec-usb.rules > /dev/null << 'EOF'
# Orbbec Astra/Nuwa depth cameras
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE:="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="1d27", MODE:="0666", GROUP="plugdev"

# Additional Orbbec devices
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="*", MODE="0666", GROUP="plugdev"
EOF
    
    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    # Add user to plugdev group
    sudo usermod -aG plugdev $USER
    
    print_info "Udev rules configured!"
}

# Setup udev rules for Luxonis OAK-D cameras
setup_oakd_udev() {
    print_info "Setting up udev rules for Luxonis OAK-D cameras..."
    
    # Create udev rules for Luxonis/Movidius VPU
    sudo tee /etc/udev/rules.d/80-movidius.rules > /dev/null << 'EOF'
# Luxonis OAK-D cameras (Movidius VPU)
SUBSYSTEM=="usb", ATTR{idVendor}=="03e7", MODE="0666", GROUP="plugdev"
EOF
    
    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    # Add user to plugdev group
    sudo usermod -aG plugdev $USER
    
    print_info "OAK-D udev rules configured!"
}

# Launch depth camera node
launch_depth_camera() {
    print_info "Launching depth camera..."
    
    case ${CAMERA_TYPE} in
        nuwa|astra)
            # Launch Astra/Nuwa camera with ROS2
            ros2 launch astra_camera astra.launch.py \
                enable_point_cloud:=true \
                depth_registration:=true \
                color_width:=640 \
                color_height:=480 \
                color_fps:=30 \
                depth_width:=640 \
                depth_height:=480 \
                depth_fps:=30 &
            ;;
        realsense)
            # Launch Intel RealSense camera
            ros2 launch realsense2_camera rs_launch.py \
                enable_pointcloud:=true \
                enable_rgbd:=true \
                depth_module.profile:=640x480x30 \
                rgb_camera.profile:=640x480x30 \
                align_depth.enable:=true &
            ;;
        oakd)
            # Launch OAK-D camera via depthai-ros
            ros2 launch depthai_ros_driver camera.launch.py \
                camera_model:=OAK-D-PRO \
                enableRviz:=false &
            ;;
    esac
    
    print_info "Camera node launched!"
}

# Launch depth camera with ROSMASTER A1 specific configuration
launch_rosmaster_camera() {
    print_info "Launching ROSMASTER A1 camera configuration..."
    
    source_ros2
    
    # Try to launch using the ROSMASTER A1 specific launch file
    if ros2 pkg list | grep -q "yahboomcar_bringup\|rosmaster"; then
        print_info "Using ROSMASTER A1 camera launch..."
        
        # Check for depth camera launch file
        if [ -f ~/rosmaster_a1_ws/src/yahboomcar_bringup/launch/depth_camera_launch.py ]; then
            ros2 launch yahboomcar_bringup depth_camera_launch.py &
        elif [ -f ~/yahboomcar_ws/src/yahboomcar_bringup/launch/depth_camera_launch.py ]; then
            ros2 launch yahboomcar_bringup depth_camera_launch.py &
        else
            print_warning "ROSMASTER A1 depth camera launch not found, using generic launch"
            launch_depth_camera
        fi
    else
        launch_depth_camera
    fi
    
    # Wait for camera to initialize
    sleep 5
    
    # Verify camera topics
    verify_camera_topics
}

# Verify camera topics are publishing
verify_camera_topics() {
    print_info "Verifying camera topics..."
    
    echo "Available camera topics:"
    ros2 topic list | grep -E "camera|depth|rgb|color|image|point" || true
    
    # Check for depth image
    if ros2 topic list | grep -q "depth"; then
        print_info "Depth camera topics detected!"
    else
        print_warning "No depth topics found"
    fi
    
    # Check for color image
    if ros2 topic list | grep -q "color\|rgb"; then
        print_info "Color camera topics detected!"
    else
        print_warning "No color topics found"
    fi
    
    # Check for point cloud
    if ros2 topic list | grep -q "point"; then
        print_info "Point cloud topics detected!"
    else
        print_warning "No point cloud topics found"
    fi
}

# View camera output using RViz2
view_camera_rviz() {
    print_info "Launching RViz2 for camera visualization..."
    
    source_ros2
    
    # Set topics based on camera type
    case ${CAMERA_TYPE} in
        oakd)
            COLOR_TOPIC="/oak/rgb/image_raw"
            DEPTH_TOPIC="/oak/stereo/image_raw"
            POINTS_TOPIC="/oak/points"
            ;;
        realsense)
            COLOR_TOPIC="/camera/color/image_raw"
            DEPTH_TOPIC="/camera/depth/image_rect_raw"
            POINTS_TOPIC="/camera/depth/color/points"
            ;;
        *)
            COLOR_TOPIC="/camera/color/image_raw"
            DEPTH_TOPIC="/camera/depth/image_raw"
            POINTS_TOPIC="/camera/depth/points"
            ;;
    esac
    
    # Create RViz config for camera
    RVIZ_CONFIG="/tmp/camera_view.rviz"
    
    cat > ${RVIZ_CONFIG} << EOF
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Image
      Name: Color Image
      Topic:
        Value: ${COLOR_TOPIC}
      Transport Hint: raw
    - Class: rviz_default_plugins/Image
      Name: Depth Image
      Topic:
        Value: ${DEPTH_TOPIC}
      Transport Hint: raw
    - Class: rviz_default_plugins/PointCloud2
      Name: PointCloud
      Topic:
        Value: ${POINTS_TOPIC}
      Size (m): 0.01
  Global Options:
    Fixed Frame: camera_link
EOF
    
    ros2 run rviz2 rviz2 -d ${RVIZ_CONFIG} &
}

# Show camera info
show_camera_info() {
    print_info "Camera Information:"
    
    source_ros2
    
    # Get camera info
    echo ""
    echo "Camera Topics:"
    ros2 topic list | grep -E "camera|depth|rgb|color|image|point" | while read topic; do
        echo "  $topic"
        ros2 topic info "$topic" 2>/dev/null | head -5
        echo ""
    done
}

# Show usage
show_usage() {
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  install     Install camera dependencies"
    echo "  launch      Launch the depth camera node"
    echo "  check       Check camera connection"
    echo "  view        Launch RViz2 to view camera output"
    echo "  info        Show camera information and topics"
    echo "  help        Show this help message"
    echo ""
    echo "Environment variables:"
    echo "  CAMERA_TYPE  Camera type: oakd, nuwa, astra, realsense (default: oakd)"
}

# Main execution
print_header

case "${1:-help}" in
    install)
        install_camera_deps
        ;;
    launch)
        launch_rosmaster_camera
        ;;
    check)
        check_camera_connection
        ;;
    view)
        view_camera_rviz
        ;;
    info)
        source_ros2
        show_camera_info
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
