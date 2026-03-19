#!/bin/bash
# ==============================================================================
# deploy.sh — Deploy robot scripts, configs, services, and interfaces to Jetson
#
# Usage:
#   ./deploy.sh                    # Full deploy (stop → deploy → start → verify)
#   ./deploy.sh --scripts-only     # Deploy scripts/configs only (skip interfaces)
#   ./deploy.sh --no-start         # Deploy everything but don't start services
#   ./deploy.sh --dry-run          # Show what would be deployed without doing it
#
# Requirements:
#   - SSH access to jetson@192.168.7.250 (key-based auth recommended)
#   - sudo access on the Jetson (for systemctl, apt, service file install)
# ==============================================================================
set -euo pipefail

# === Configuration ===
ROBOT_HOST="jetson@192.168.7.250"
ROBOT_IP="192.168.7.250"
ROBOT_SCRIPTS_DIR="/home/jetson/robot_scripts"
ROBOT_WS="/home/jetson/robot_ws"
SYSTEMD_DIR="/etc/systemd/system"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# === Defaults ===
SKIP_INTERFACES=false
NO_START=false
DRY_RUN=false

# === Parse arguments ===
for arg in "$@"; do
    case "$arg" in
        --scripts-only) SKIP_INTERFACES=true ;;
        --no-start)     NO_START=true ;;
        --dry-run)      DRY_RUN=true ;;
        -h|--help)
            head -12 "$0" | tail -8
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: ./deploy.sh [--scripts-only] [--no-start] [--dry-run]"
            exit 1
            ;;
    esac
done

# === Color helpers ===
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

info()    { echo -e "${BLUE}[deploy]${NC} $*"; }
success() { echo -e "${GREEN}[deploy]${NC} $*"; }
warn()    { echo -e "${YELLOW}[deploy]${NC} $*"; }
fail()    { echo -e "${RED}[deploy]${NC} $*"; }

# === Files to deploy ===

# Scripts and configs → /home/jetson/robot_scripts/
ROBOT_SCRIPT_FILES=(
    # Environment
    scripts/ros2_env.sh
    # Service run wrappers
    scripts/run_base.sh
    scripts/run_camera.sh
    scripts/run_lidar.sh
    scripts/run_tf.sh
    scripts/run_vslam.sh
    scripts/run_arbiter.sh
    scripts/run_voice_mapper.sh
    # Python application
    scripts/voice_mapper.py
    scripts/velocity_arbiter.py
    scripts/sensor_snapshot.py
    scripts/llm_navigator.py
    scripts/safety_executor.py
    scripts/exploration_memory.py
    # Launch files
    scripts/oakd_vslam.launch.py
    # Config
    scripts/oakd_params.yaml
    scripts/nav2_params.yaml
    # Legacy / test (still useful)
    scripts/start_robot.sh
    scripts/start_cam_test.sh
    scripts/start_vslam_test.sh
)

# Systemd unit files → /etc/systemd/system/
SYSTEMD_FILES=(
    scripts/robot.target
    scripts/robot-base.service
    scripts/robot-lidar.service
    scripts/robot-camera.service
    scripts/robot-tf.service
    scripts/robot-vslam.service
    scripts/robot-arbiter.service
    scripts/robot-voice-mapper.service
)

# Services in startup order (used for health checks)
SERVICES_ORDERED=(
    robot-base
    robot-lidar
    robot-camera
    robot-tf
    robot-vslam
    robot-arbiter
    robot-voice-mapper
)

# Python pip packages required on the robot
PIP_PACKAGES=(
    openai
    pyaudio
    numpy
    scipy
    pyyaml
)

# ==============================================================================
# Helpers
# ==============================================================================

run_on_robot() {
    # Execute a command on the robot via SSH
    if $DRY_RUN; then
        info "[dry-run] ssh $ROBOT_HOST \"$*\""
        return 0
    fi
    ssh -o ConnectTimeout=10 -o BatchMode=yes "$ROBOT_HOST" "$@"
}

copy_to_robot() {
    # Copy file(s) to robot destination directory
    local dest="$1"
    shift
    if $DRY_RUN; then
        for f in "$@"; do
            info "[dry-run] scp $f → $ROBOT_HOST:$dest/"
        done
        return 0
    fi
    scp -o ConnectTimeout=10 -o BatchMode=yes "$@" "$ROBOT_HOST:$dest/"
}

# ==============================================================================
# Step 0: Preflight checks
# ==============================================================================

step_preflight() {
    info "=== Preflight checks ==="

    # Check local files exist
    local missing=0
    for f in "${ROBOT_SCRIPT_FILES[@]}" "${SYSTEMD_FILES[@]}"; do
        if [ ! -f "$SCRIPT_DIR/$f" ]; then
            fail "Missing file: $f"
            missing=1
        fi
    done
    if [ ! -d "$SCRIPT_DIR/robot_interfaces" ]; then
        if ! $SKIP_INTERFACES; then
            fail "Missing directory: robot_interfaces/"
            missing=1
        fi
    fi
    if [ "$missing" -eq 1 ]; then
        fail "Aborting — missing files. Run from the repository root."
        exit 1
    fi

    # Check SSH connectivity
    info "Testing SSH connection to $ROBOT_IP..."
    if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "$ROBOT_HOST" "echo ok" >/dev/null 2>&1; then
        fail "Cannot SSH to $ROBOT_HOST — check network and SSH keys."
        exit 1
    fi
    success "SSH connection OK"

    # Check sudo access
    if ! run_on_robot "sudo -n true" 2>/dev/null; then
        warn "sudo requires password — you may be prompted during deployment."
    fi
}

# ==============================================================================
# Step 1: Stop services
# ==============================================================================

step_stop_services() {
    info "=== Stopping robot services ==="

    if $DRY_RUN; then
        info "[dry-run] Would stop robot.target"
        return 0
    fi

    # Check if robot.target is loaded
    if run_on_robot "systemctl is-active robot.target" >/dev/null 2>&1; then
        run_on_robot "sudo systemctl stop robot.target" || true
        info "Waiting for services to stop..."
        sleep 3

        # Verify all services stopped
        local still_running=false
        for svc in "${SERVICES_ORDERED[@]}"; do
            if run_on_robot "systemctl is-active ${svc}.service" >/dev/null 2>&1; then
                warn "${svc} still running — force stopping"
                run_on_robot "sudo systemctl stop ${svc}.service" || true
                still_running=true
            fi
        done

        if $still_running; then
            sleep 2
        fi

        success "All services stopped"
    else
        info "robot.target not active — skipping stop"
    fi
}

# ==============================================================================
# Step 2: Deploy scripts and configs
# ==============================================================================

step_deploy_scripts() {
    info "=== Deploying scripts and configs ==="

    # Ensure target directory exists
    run_on_robot "mkdir -p $ROBOT_SCRIPTS_DIR"

    # Copy all script files
    local files_to_copy=()
    for f in "${ROBOT_SCRIPT_FILES[@]}"; do
        files_to_copy+=("$SCRIPT_DIR/$f")
    done

    copy_to_robot "$ROBOT_SCRIPTS_DIR" "${files_to_copy[@]}"

    # Make shell scripts and Python files executable
    run_on_robot "chmod +x ${ROBOT_SCRIPTS_DIR}/*.sh ${ROBOT_SCRIPTS_DIR}/*.py"

    success "Scripts and configs deployed to $ROBOT_SCRIPTS_DIR"
}

# ==============================================================================
# Step 3: Deploy systemd service files
# ==============================================================================

step_deploy_services() {
    info "=== Deploying systemd service files ==="

    # Copy service files to a temp location first (scp can't write to /etc directly)
    local tmp_dir="/tmp/robot_services_deploy"
    run_on_robot "mkdir -p $tmp_dir"

    local svc_files=()
    for f in "${SYSTEMD_FILES[@]}"; do
        svc_files+=("$SCRIPT_DIR/$f")
    done

    copy_to_robot "$tmp_dir" "${svc_files[@]}"

    # Install to systemd directory and reload
    run_on_robot "sudo cp ${tmp_dir}/*.service ${tmp_dir}/*.target ${SYSTEMD_DIR}/ && \
                  sudo systemctl daemon-reload && \
                  rm -rf $tmp_dir"

    # Enable robot.target so it starts on boot
    run_on_robot "sudo systemctl enable robot.target" 2>/dev/null || true

    success "Systemd services installed and daemon reloaded"
}

# ==============================================================================
# Step 4: Deploy robot_interfaces package
# ==============================================================================

step_deploy_interfaces() {
    if $SKIP_INTERFACES; then
        info "=== Skipping robot_interfaces (--scripts-only) ==="
        return 0
    fi

    info "=== Deploying robot_interfaces package ==="

    if $DRY_RUN; then
        info "[dry-run] Would copy robot_interfaces/ and build on robot"
        return 0
    fi

    # Create workspace
    run_on_robot "mkdir -p ${ROBOT_WS}/src"

    # Copy package — use scp -r for directory
    ssh -o ConnectTimeout=10 -o BatchMode=yes "$ROBOT_HOST" "rm -rf ${ROBOT_WS}/src/robot_interfaces"
    scp -o ConnectTimeout=10 -o BatchMode=yes -r \
        "$SCRIPT_DIR/robot_interfaces" "$ROBOT_HOST:${ROBOT_WS}/src/robot_interfaces"

    # Build on robot
    info "Building robot_interfaces on Jetson (colcon)..."
    run_on_robot "bash -c '
        source /opt/ros/humble/setup.bash
        cd ${ROBOT_WS}
        colcon build --packages-select robot_interfaces --symlink-install 2>&1
    '"

    # Verify
    local verify_output
    verify_output=$(run_on_robot "bash -c '
        source /opt/ros/humble/setup.bash
        source ${ROBOT_WS}/install/setup.bash
        ros2 interface show robot_interfaces/msg/VelocityRequest 2>&1
    '")

    if echo "$verify_output" | grep -q "twist"; then
        success "robot_interfaces built and verified"
    else
        fail "robot_interfaces verification failed!"
        echo "$verify_output"
        exit 1
    fi
}

# ==============================================================================
# Step 5: Install Python dependencies
# ==============================================================================

step_install_deps() {
    info "=== Checking Python dependencies ==="

    if $DRY_RUN; then
        info "[dry-run] Would check/install: ${PIP_PACKAGES[*]}"
        return 0
    fi

    # Check which packages are missing
    local missing_pkgs=()
    for pkg in "${PIP_PACKAGES[@]}"; do
        if ! run_on_robot "python3 -c 'import ${pkg}' 2>/dev/null"; then
            missing_pkgs+=("$pkg")
        fi
    done

    if [ ${#missing_pkgs[@]} -eq 0 ]; then
        success "All Python dependencies installed"
        return 0
    fi

    info "Installing missing packages: ${missing_pkgs[*]}"
    run_on_robot "pip3 install --user ${missing_pkgs[*]}"
    success "Python dependencies installed"
}

# ==============================================================================
# Step 6: Start services
# ==============================================================================

step_start_services() {
    if $NO_START; then
        info "=== Skipping service start (--no-start) ==="
        return 0
    fi

    info "=== Starting robot services ==="

    if $DRY_RUN; then
        info "[dry-run] Would start robot.target"
        return 0
    fi

    run_on_robot "sudo systemctl start robot.target"

    # Wait for services to start
    info "Waiting for services to initialize..."
    sleep 8

    success "robot.target started"
}

# ==============================================================================
# Step 7: Health checks
# ==============================================================================

step_health_check() {
    if $NO_START; then
        info "=== Skipping health checks (--no-start) ==="
        return 0
    fi

    info "=== Running health checks ==="

    if $DRY_RUN; then
        info "[dry-run] Would check service status"
        return 0
    fi

    local all_ok=true
    local retries=0
    local max_retries=6  # 6 × 5s = 30s max wait

    while [ $retries -lt $max_retries ]; do
        all_ok=true
        local not_ready=()

        for svc in "${SERVICES_ORDERED[@]}"; do
            local status
            status=$(run_on_robot "systemctl is-active ${svc}.service 2>/dev/null" || echo "inactive")

            case "$status" in
                active)
                    ;;
                activating)
                    not_ready+=("$svc")
                    all_ok=false
                    ;;
                *)
                    # VSLAM is optional (ConditionPathExists may skip it)
                    if [ "$svc" = "robot-vslam" ]; then
                        local condition
                        condition=$(run_on_robot "systemctl show -p ConditionResult ${svc}.service 2>/dev/null" || echo "")
                        if echo "$condition" | grep -q "no"; then
                            warn "  ${svc}: skipped (launch file not present)"
                            continue
                        fi
                    fi
                    not_ready+=("$svc")
                    all_ok=false
                    ;;
            esac
        done

        if $all_ok; then
            break
        fi

        retries=$((retries + 1))
        if [ $retries -lt $max_retries ]; then
            info "Waiting for services: ${not_ready[*]} ($((retries * 5))s / $((max_retries * 5))s)"
            sleep 5
        fi
    done

    # Final status report
    echo ""
    info "=== Service Status ==="
    local failed=false
    for svc in "${SERVICES_ORDERED[@]}"; do
        local status
        status=$(run_on_robot "systemctl is-active ${svc}.service 2>/dev/null" || echo "inactive")

        case "$status" in
            active)
                success "  ${svc}: running"
                ;;
            *)
                # Check if VSLAM was skipped by condition
                if [ "$svc" = "robot-vslam" ]; then
                    local condition
                    condition=$(run_on_robot "systemctl show -p ConditionResult ${svc}.service 2>/dev/null" || echo "")
                    if echo "$condition" | grep -q "no"; then
                        warn "  ${svc}: skipped (optional)"
                        continue
                    fi
                fi
                fail "  ${svc}: $status"
                failed=true
                ;;
        esac
    done
    echo ""

    if $failed; then
        fail "Some services failed to start! Check logs with:"
        echo "  ssh $ROBOT_HOST 'journalctl -u <service-name> --no-pager -n 30'"
        return 1
    fi

    # Quick ROS2 topic check
    info "Checking ROS2 topics..."
    local topics
    topics=$(run_on_robot "bash -c 'source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && timeout 5 ros2 topic list 2>/dev/null'" || echo "")

    local topic_ok=true
    for topic in "/scan" "/odom" "/oak/rgb/image_raw" "/cmd_vel"; do
        if echo "$topics" | grep -q "^${topic}$"; then
            success "  Topic $topic: publishing"
        else
            warn "  Topic $topic: not found (may still be initializing)"
            topic_ok=false
        fi
    done
    echo ""

    if $topic_ok; then
        success "=== Deployment complete — all systems operational ==="
    else
        warn "=== Deployment complete — some topics not yet visible (may need a few more seconds) ==="
    fi
}

# ==============================================================================
# Main
# ==============================================================================

main() {
    echo ""
    echo "============================================================"
    echo "  Robot Deployment"
    echo "  Target: $ROBOT_HOST ($ROBOT_IP)"
    if $DRY_RUN; then
        echo "  Mode: DRY RUN (no changes will be made)"
    fi
    echo "============================================================"
    echo ""

    step_preflight
    echo ""
    step_stop_services
    echo ""
    step_deploy_scripts
    echo ""
    step_deploy_services
    echo ""
    step_deploy_interfaces
    echo ""
    step_install_deps
    echo ""
    step_start_services
    echo ""
    step_health_check
    echo ""
}

main "$@"
