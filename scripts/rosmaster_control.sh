#!/bin/bash
#===============================================================================
# ROSMASTER A1 - Master Control Script
# All-in-one script to launch and manage the robot
# Brings together all components: camera, voice, navigation, autonomous driving
#===============================================================================

set -e

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Configuration
ROBOT_IP="${ROBOT_IP:-192.168.2.1}"
ROBOT_USER="${ROBOT_USER:-yahboom}"
LOG_DIR="${HOME}/rosmaster_logs"
PID_FILE="/tmp/rosmaster_pids"

print_banner() {
    echo -e "${CYAN}"
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║                                                               ║"
    echo "║     ██████╗  ██████╗ ███████╗███╗   ███╗ █████╗ ███████╗     ║"
    echo "║     ██╔══██╗██╔═══██╗██╔════╝████╗ ████║██╔══██╗██╔════╝     ║"
    echo "║     ██████╔╝██║   ██║███████╗██╔████╔██║███████║███████╗     ║"
    echo "║     ██╔══██╗██║   ██║╚════██║██║╚██╔╝██║██╔══██║╚════██║     ║"
    echo "║     ██║  ██║╚██████╔╝███████║██║ ╚═╝ ██║██║  ██║███████║     ║"
    echo "║     ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚═╝     ╚═╝╚═╝  ╚═╝╚══════╝     ║"
    echo "║                                                               ║"
    echo "║     ████████╗███████╗██████╗        █████╗  ██╗              ║"
    echo "║     ╚══██╔══╝██╔════╝██╔══██╗      ██╔══██╗███║              ║"
    echo "║        ██║   █████╗  ██████╔╝█████╗███████║╚██║              ║"
    echo "║        ██║   ██╔══╝  ██╔══██╗╚════╝██╔══██║ ██║              ║"
    echo "║        ██║   ███████╗██║  ██║      ██║  ██║ ██║              ║"
    echo "║        ╚═╝   ╚══════╝╚═╝  ╚═╝      ╚═╝  ╚═╝ ╚═╝              ║"
    echo "║                                                               ║"
    echo "║          AI Large Model Robot - Master Control                ║"
    echo "║                                                               ║"
    echo "╚═══════════════════════════════════════════════════════════════╝"
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
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    # Source Yahboom workspaces (library_ws must come before yahboomcar_ws)
    if [ -f ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash ]; then
        source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
    fi
    
    if [ -f ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash ]; then
        source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
    elif [ -f ~/yahboomcar_ws/install/setup.bash ]; then
        source ~/yahboomcar_ws/install/setup.bash
    fi
}

# Save PID to file
save_pid() {
    local name="$1"
    local pid="$2"
    echo "${name}:${pid}" >> ${PID_FILE}
}

# Kill all saved processes
kill_all_processes() {
    print_info "Stopping all ROSMASTER A1 processes..."
    
    if [ -f ${PID_FILE} ]; then
        while IFS=: read -r name pid; do
            if kill -0 $pid 2>/dev/null; then
                print_info "Stopping ${name} (PID: ${pid})"
                kill $pid 2>/dev/null || true
            fi
        done < ${PID_FILE}
        rm -f ${PID_FILE}
    fi
    
    # Also kill any ROS2 nodes
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "rosmaster" 2>/dev/null || true
    
    print_info "All processes stopped"
}

# Check system status
check_status() {
    print_info "System Status Check"
    echo ""
    
    source_ros2
    
    echo "=== ROS2 Nodes ==="
    ros2 node list 2>/dev/null || echo "  No ROS2 nodes running"
    echo ""
    
    echo "=== ROS2 Topics ==="
    ros2 topic list 2>/dev/null | head -20 || echo "  No topics"
    echo ""
    
    echo "=== Camera Status ==="
    if ros2 topic list 2>/dev/null | grep -q "camera"; then
        echo "  Camera topics detected"
    else
        echo "  Camera not running"
    fi
    echo ""
    
    echo "=== LiDAR Status ==="
    if ros2 topic list 2>/dev/null | grep -q "/scan"; then
        echo "  LiDAR topic detected"
    else
        echo "  LiDAR not running"
    fi
    echo ""
    
    echo "=== IMU Status ==="
    if ros2 topic list 2>/dev/null | grep -qi "imu"; then
        echo "  IMU topic detected"
        ros2 topic list 2>/dev/null | grep -i imu
    else
        echo "  IMU not detected (optional sensor)"
    fi
    echo ""
    
    echo "=== Navigation Status ==="
    if ros2 topic list 2>/dev/null | grep -q "navigate_to_pose"; then
        echo "  Navigation stack running"
    else
        echo "  Navigation not running"
    fi
}

# Full system startup
full_startup() {
    print_info "Starting full ROSMASTER A1 system..."
    mkdir -p ${LOG_DIR}
    
    source_ros2
    
    # Start robot base (A1 bringup with odometry/EKF)
    print_info "Starting robot base..."
    if ros2 pkg list | grep -q "yahboomcar_bringup"; then
        ros2 launch yahboomcar_bringup yahboomcar_bringup_A1_launch.py \
            > ${LOG_DIR}/bringup.log 2>&1 &
        save_pid "bringup" $!
        sleep 5
    fi
    
    # Start depth camera (OAK-D Pro via depthai-ros)
    print_info "Starting depth camera (OAK-D Pro)..."
    ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-PRO \
        > ${LOG_DIR}/camera.log 2>&1 &
    save_pid "camera" $!
    sleep 3
    
    # Start LiDAR (SLLidar C1)
    print_info "Starting LiDAR..."
    if ros2 pkg list | grep -q "sllidar_ros2"; then
        ros2 launch sllidar_ros2 sllidar_c1_launch.py \
            > ${LOG_DIR}/lidar.log 2>&1 &
        save_pid "lidar" $!
        sleep 3
    fi
    
    print_info "Base system started!"
    print_info "Logs available in: ${LOG_DIR}"
}

# Start with navigation
start_navigation_mode() {
    local map_file="${1:-${HOME}/maps/rosmaster_map.yaml}"
    
    print_info "Starting navigation mode..."
    
    full_startup
    
    # Start navigation stack
    print_info "Starting navigation stack..."
    if [ -f ${SCRIPT_DIR}/04_setup_navigation.sh ]; then
        bash ${SCRIPT_DIR}/04_setup_navigation.sh nav "${map_file}" \
            > ${LOG_DIR}/navigation.log 2>&1 &
        save_pid "navigation" $!
    fi
    
    print_info "Navigation mode started!"
}

# Start mapping mode
start_mapping_mode() {
    print_info "Starting SLAM mapping mode..."
    
    full_startup
    
    # Start SLAM
    if [ -f ${SCRIPT_DIR}/04_setup_navigation.sh ]; then
        bash ${SCRIPT_DIR}/04_setup_navigation.sh slam \
            > ${LOG_DIR}/slam.log 2>&1 &
        save_pid "slam" $!
    fi
    
    print_info "Mapping mode started!"
    print_info "Use teleop to drive around and create map"
    print_info "Or use: ./rosmaster_control.sh run  (voice mapper has built-in SLAM)"
}

# SSH into robot
ssh_to_robot() {
    bash ${SCRIPT_DIR}/01_ssh_connect.sh "$@"
}

# Install all dependencies
install_all() {
    print_info "Installing all dependencies..."
    
    # Core Python dependencies for voice_mapper.py
    pip3 install --user openai pyaudio numpy scipy pyyaml
    
    # System packages
    sudo apt-get update
    sudo apt-get install -y ffmpeg portaudio19-dev python3-pyaudio mpv
    
    # Optional: Setup scripts for hardware
    if [ -f ${SCRIPT_DIR}/02_setup_depth_camera.sh ]; then
        bash ${SCRIPT_DIR}/02_setup_depth_camera.sh install 2>/dev/null || true
    fi
    if [ -f ${SCRIPT_DIR}/03_setup_voice_commands.sh ]; then
        bash ${SCRIPT_DIR}/03_setup_voice_commands.sh install 2>/dev/null || true
    fi
    if [ -f ${SCRIPT_DIR}/04_setup_navigation.sh ]; then
        bash ${SCRIPT_DIR}/04_setup_navigation.sh install 2>/dev/null || true
    fi
    
    # Create required directories
    mkdir -p ~/exploration_logs ~/maps
    
    print_info "All dependencies installed!"
}

# Run voice_mapper.py directly (recommended)
run_voice_mapper() {
    print_info "Starting Voice Mapper (main script)..."
    
    source_ros2
    
    # Source LLM config
    if [ -f ~/.rosmaster_llm_config ]; then
        source ~/.rosmaster_llm_config
    else
        print_error "OpenAI config not found: ~/.rosmaster_llm_config"
        print_info "Create it with: echo 'export OPENAI_API_KEY=sk-...' > ~/.rosmaster_llm_config"
        exit 1
    fi
    
    export ROS_DOMAIN_ID=62
    cd ${SCRIPT_DIR}
    python3 voice_mapper.py
}

# Run yahboom_explorer.py (alternative for USB 2.0 cameras)
run_yahboom_explorer() {
    print_warning "=== DEPRECATION WARNING ==="
    print_warning "yahboom_explorer.py is deprecated. Use voice_mapper.py instead."
    print_warning "Run: ./rosmaster_control.sh run"
    print_warning "==========================="
    echo ""
    
    print_info "Starting Yahboom Explorer (alternative script)..."
    
    source_ros2
    
    if [ -f ~/.rosmaster_llm_config ]; then
        source ~/.rosmaster_llm_config
    else
        print_error "OpenAI config not found"
        exit 1
    fi
    
    export ROS_DOMAIN_ID=62
    cd ${SCRIPT_DIR}
    python3 yahboom_explorer.py
}

# === Service Management (merged from install_service.sh) ===

SERVICE_NAME="voice_mapper"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

service_install() {
    print_info "Installing ${SERVICE_NAME} systemd service..."
    
    if [ "$EUID" -ne 0 ]; then
        print_error "Please run with sudo: sudo $0 service install"
        exit 1
    fi
    
    # Check dependencies
    if [ ! -f "/home/jetson/robot_scripts/voice_mapper.py" ]; then
        print_error "voice_mapper.py not found in /home/jetson/robot_scripts/"
        exit 1
    fi
    
    if [ ! -f "/home/jetson/.rosmaster_llm_config" ]; then
        print_error "OpenAI config not found: /home/jetson/.rosmaster_llm_config"
        exit 1
    fi
    
    # Create systemd-compatible env file
    sed 's/export //' /home/jetson/.rosmaster_llm_config > /home/jetson/.rosmaster_llm_env
    chown jetson:jetson /home/jetson/.rosmaster_llm_env
    
    # Create service file
    cat > "$SERVICE_FILE" << 'SERVICEEOF'
[Unit]
Description=Voice-Controlled Mapping Explorer Robot
After=network.target sound.target
Wants=network.target

[Service]
Type=simple
User=jetson
Group=jetson
Environment="HOME=/home/jetson"
Environment="ROS_DOMAIN_ID=62"
Environment="DISPLAY=:0"
Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
EnvironmentFile=/home/jetson/.rosmaster_llm_env
ExecStart=/bin/bash /home/jetson/robot_scripts/start_robot.sh
Restart=on-failure
RestartSec=10
TimeoutStopSec=10
KillMode=mixed
StandardOutput=journal
StandardError=journal
SyslogIdentifier=voice_mapper

[Install]
WantedBy=multi-user.target
SERVICEEOF
    
    systemctl daemon-reload
    systemctl enable "$SERVICE_NAME"
    systemctl start "$SERVICE_NAME"
    
    print_info "Service installed and started!"
    print_info "Commands: sudo systemctl [status|stop|start|restart] $SERVICE_NAME"
    print_info "Logs: journalctl -u $SERVICE_NAME -f"
}

service_uninstall() {
    if [ "$EUID" -ne 0 ]; then
        print_error "Please run with sudo"
        exit 1
    fi
    
    systemctl stop "$SERVICE_NAME" 2>/dev/null || true
    systemctl disable "$SERVICE_NAME" 2>/dev/null || true
    rm -f "$SERVICE_FILE"
    systemctl daemon-reload
    print_info "Service uninstalled"
}

service_status() {
    systemctl status "$SERVICE_NAME" --no-pager
}

service_logs() {
    journalctl -u "$SERVICE_NAME" -f
}

service_cmd() {
    case "${1:-status}" in
        install)   service_install ;;
        uninstall) service_uninstall ;;
        start)     sudo systemctl start "$SERVICE_NAME" && service_status ;;
        stop)      sudo systemctl stop "$SERVICE_NAME" ;;
        restart)   sudo systemctl restart "$SERVICE_NAME" && service_status ;;
        status)    service_status ;;
        logs)      service_logs ;;
        *)
            echo "Service commands: install, uninstall, start, stop, restart, status, logs"
            ;;
    esac
}

# Interactive menu
interactive_menu() {
    while true; do
        print_banner
        echo ""
        echo "  ${CYAN}=== Quick Start ===${NC}"
        echo "  ${GREEN}1${NC}) Run Voice Mapper (recommended)"
        echo "  ${GREEN}2${NC}) Run Yahboom Explorer (USB 2.0 camera)"
        echo ""
        echo "  ${CYAN}=== Hardware ===${NC}"
        echo "  ${GREEN}3${NC}) Start Hardware Only (base, LiDAR, camera)"
        echo "  ${GREEN}4${NC}) Check System Status"
        echo ""
        echo "  ${CYAN}=== Navigation ===${NC}"
        echo "  ${GREEN}5${NC}) SLAM Mapping Mode"
        echo "  ${GREEN}6${NC}) Navigation Mode (with map)"
        echo "  ${GREEN}7${NC}) Teleop (Manual Control)"
        echo ""
        echo "  ${CYAN}=== Setup ===${NC}"
        echo "  ${YELLOW}i${NC}) Install Dependencies"
        echo "  ${YELLOW}s${NC}) Service Management (auto-start)"
        echo "  ${YELLOW}c${NC}) SSH Connect to Robot"
        echo ""
        echo "  ${RED}x${NC}) Stop All & Exit"
        echo "  ${RED}q${NC}) Quit (keep running)"
        echo ""
        read -p "  Enter choice: " choice
        
        case ${choice} in
            1) run_voice_mapper ;;
            2) run_yahboom_explorer ;;
            3) full_startup ;;
            4) check_status ;;
            5) start_mapping_mode ;;
            6) 
                read -p "  Map file (or press Enter for default): " map
                start_navigation_mode "${map}"
                ;;
            7) bash ${SCRIPT_DIR}/04_setup_navigation.sh teleop ;;
            i) install_all ;;
            s)
                echo ""
                echo "  Service commands:"
                echo "    install   - Install auto-start service"
                echo "    uninstall - Remove service"
                echo "    status    - Check service status"
                echo "    logs      - View live logs"
                echo ""
                read -p "  Service command: " svc_cmd
                service_cmd "$svc_cmd"
                ;;
            c) ssh_to_robot ;;
            x) 
                kill_all_processes
                exit 0
                ;;
            q) exit 0 ;;
            *) print_error "Invalid option" ;;
        esac
        
        echo ""
        read -p "Press Enter to continue..."
    done
}

# Show usage
show_usage() {
    print_banner
    echo ""
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Quick Start:"
    echo "  run             Run voice_mapper.py (recommended)"
    echo "  yahboom         Run yahboom_explorer.py (USB 2.0 camera)"
    echo ""
    echo "Hardware:"
    echo "  start           Start hardware (camera, LiDAR, base)"
    echo "  status          Check system status"
    echo "  stop            Stop all processes"
    echo ""
    echo "Navigation:"
    echo "  mapping         SLAM mapping mode"
    echo "  nav [map]       Navigation mode with map"
    echo "  teleop          Manual keyboard control"
    echo ""
    echo "Setup:"
    echo "  install         Install all dependencies"
    echo "  service [cmd]   Service management (install|uninstall|status|logs)"
    echo "  ssh             SSH connect to robot"
    echo ""
    echo "Other:"
    echo "  menu            Interactive menu"
    echo "  help            Show this help"
}

# Main execution
case "${1:-menu}" in
    run|voice|voice-mapper)
        run_voice_mapper
        ;;
    yahboom|explorer)
        run_yahboom_explorer
        ;;
    start|hardware)
        full_startup
        ;;
    nav|navigation)
        start_navigation_mode "$2"
        ;;
    mapping|slam)
        start_mapping_mode
        ;;
    teleop)
        source_ros2
        bash ${SCRIPT_DIR}/04_setup_navigation.sh teleop
        ;;
    status)
        check_status
        ;;
    stop)
        kill_all_processes
        ;;
    install)
        install_all
        ;;
    service)
        shift
        service_cmd "$@"
        ;;
    ssh)
        shift
        ssh_to_robot "$@"
        ;;
    menu)
        interactive_menu
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
