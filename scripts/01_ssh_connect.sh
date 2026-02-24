#!/bin/bash
#===============================================================================
# ROSMASTER A1 - SSH Connection Script
# Connects to the Jetson Orin Nano on the robot
#===============================================================================

# Default configuration (modify these for your robot)
ROBOT_IP="${ROBOT_IP:-192.168.2.1}"
ROBOT_USER="${ROBOT_USER:-yahboom}"
ROBOT_PASSWORD="${ROBOT_PASSWORD:-yahboom}"
SSH_PORT="${SSH_PORT:-22}"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BLUE}"
    echo "========================================"
    echo "  ROSMASTER A1 - SSH Connection"
    echo "  Jetson Orin Nano"
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

check_network() {
    print_info "Checking network connectivity to robot at ${ROBOT_IP}..."
    
    if ping -c 1 -W 3 ${ROBOT_IP} > /dev/null 2>&1; then
        print_info "Robot is reachable!"
        return 0
    else
        print_error "Cannot reach robot at ${ROBOT_IP}"
        print_warning "Make sure:"
        echo "  1. Robot is powered on"
        echo "  2. You're connected to the robot's WiFi network"
        echo "  3. The IP address is correct"
        return 1
    fi
}

install_sshpass() {
    if ! command -v sshpass &> /dev/null; then
        print_warning "sshpass not found. Installing..."
        if command -v apt-get &> /dev/null; then
            sudo apt-get update && sudo apt-get install -y sshpass
        elif command -v yum &> /dev/null; then
            sudo yum install -y sshpass
        elif command -v brew &> /dev/null; then
            brew install hudochenkov/sshpass/sshpass
        else
            print_error "Cannot install sshpass. Please install manually."
            return 1
        fi
    fi
    return 0
}

connect_ssh() {
    print_info "Connecting to Jetson Orin Nano..."
    
    # Check if SSH key exists, use it; otherwise use password
    if [ -f ~/.ssh/rosmaster_a1_key ]; then
        print_info "Using SSH key authentication"
        ssh -i ~/.ssh/rosmaster_a1_key -p ${SSH_PORT} ${ROBOT_USER}@${ROBOT_IP}
    else
        print_info "Using password authentication"
        if install_sshpass; then
            sshpass -p "${ROBOT_PASSWORD}" ssh -o StrictHostKeyChecking=no -p ${SSH_PORT} ${ROBOT_USER}@${ROBOT_IP}
        else
            # Fallback to manual password entry
            ssh -o StrictHostKeyChecking=no -p ${SSH_PORT} ${ROBOT_USER}@${ROBOT_IP}
        fi
    fi
}

setup_ssh_key() {
    print_info "Setting up SSH key for passwordless login..."
    
    # Generate SSH key if it doesn't exist
    if [ ! -f ~/.ssh/rosmaster_a1_key ]; then
        ssh-keygen -t ed25519 -f ~/.ssh/rosmaster_a1_key -N "" -C "rosmaster_a1"
        print_info "SSH key generated"
    fi
    
    # Copy key to robot
    print_info "Copying SSH key to robot..."
    if install_sshpass; then
        sshpass -p "${ROBOT_PASSWORD}" ssh-copy-id -i ~/.ssh/rosmaster_a1_key.pub -o StrictHostKeyChecking=no -p ${SSH_PORT} ${ROBOT_USER}@${ROBOT_IP}
        print_info "SSH key copied successfully!"
    else
        ssh-copy-id -i ~/.ssh/rosmaster_a1_key.pub -p ${SSH_PORT} ${ROBOT_USER}@${ROBOT_IP}
    fi
}

execute_remote_command() {
    local cmd="$1"
    print_info "Executing remote command: $cmd"
    
    if [ -f ~/.ssh/rosmaster_a1_key ]; then
        ssh -i ~/.ssh/rosmaster_a1_key -p ${SSH_PORT} ${ROBOT_USER}@${ROBOT_IP} "$cmd"
    else
        if install_sshpass; then
            sshpass -p "${ROBOT_PASSWORD}" ssh -o StrictHostKeyChecking=no -p ${SSH_PORT} ${ROBOT_USER}@${ROBOT_IP} "$cmd"
        fi
    fi
}

show_usage() {
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  connect     Connect to the robot via SSH (default)"
    echo "  setup-key   Setup SSH key for passwordless login"
    echo "  test        Test connectivity to the robot"
    echo "  exec CMD    Execute a command on the robot"
    echo "  help        Show this help message"
    echo ""
    echo "Environment variables:"
    echo "  ROBOT_IP       Robot IP address (default: 192.168.2.1)"
    echo "  ROBOT_USER     SSH username (default: yahboom)"
    echo "  ROBOT_PASSWORD SSH password (default: yahboom)"
    echo "  SSH_PORT       SSH port (default: 22)"
}

# Main execution
print_header

case "${1:-connect}" in
    connect)
        if check_network; then
            connect_ssh
        fi
        ;;
    setup-key)
        if check_network; then
            setup_ssh_key
        fi
        ;;
    test)
        check_network
        if [ $? -eq 0 ]; then
            print_info "Testing SSH connection..."
            execute_remote_command "echo 'SSH connection successful!' && uname -a"
        fi
        ;;
    exec)
        shift
        if check_network; then
            execute_remote_command "$*"
        fi
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
