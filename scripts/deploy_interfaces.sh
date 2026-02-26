#!/bin/bash
# deploy_interfaces.sh — Build and install robot_interfaces on the Jetson.
#
# Run from the dev machine (where this repo is checked out):
#   ./scripts/deploy_interfaces.sh
#
# What it does:
#   1. Copies robot_interfaces/ package to Jetson ~/robot_ws/src/robot_interfaces/
#   2. Builds with colcon on the Jetson
#   3. Verifies the message is importable
#
# Idempotent — safe to run multiple times.

set -euo pipefail

ROBOT_HOST="jetson@192.168.7.250"
ROBOT_WS="/home/jetson/robot_ws"
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PKG_DIR="${SCRIPT_DIR}/robot_interfaces"

if [ ! -d "$PKG_DIR" ]; then
    echo "ERROR: robot_interfaces/ not found at $PKG_DIR"
    exit 1
fi

echo "=== Deploying robot_interfaces to ${ROBOT_HOST} ==="

# 1. Create workspace structure on Jetson
echo "[1/4] Creating workspace ${ROBOT_WS}/src/ ..."
ssh "$ROBOT_HOST" "mkdir -p ${ROBOT_WS}/src"

# 2. Copy package to Jetson
echo "[2/4] Copying robot_interfaces/ to Jetson ..."
rsync -av --delete "$PKG_DIR/" "${ROBOT_HOST}:${ROBOT_WS}/src/robot_interfaces/"

# 3. Build on Jetson
echo "[3/4] Building with colcon on Jetson ..."
ssh "$ROBOT_HOST" "bash -c '
    source /opt/ros/humble/setup.bash
    cd ${ROBOT_WS}
    colcon build --packages-select robot_interfaces --symlink-install
'"

# 4. Verify
echo "[4/4] Verifying message definition ..."
ssh "$ROBOT_HOST" "bash -c '
    source /opt/ros/humble/setup.bash
    source ${ROBOT_WS}/install/setup.bash
    ros2 interface show robot_interfaces/msg/VelocityRequest
'"

echo ""
echo "=== robot_interfaces deployed successfully ==="
echo "Ensure ros2_env.sh sources ${ROBOT_WS}/install/setup.bash"
