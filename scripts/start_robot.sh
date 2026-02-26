#!/bin/bash
# ==============================================================================
# start_robot.sh — Full robot startup for systemd service
# Launches all hardware nodes, waits for readiness, then starts voice_mapper.py
# ==============================================================================
set -e

echo "[start_robot.sh] WARNING: This script is deprecated. Use: sudo systemctl start robot.target"
echo "[start_robot.sh] Running in legacy mode for backward compatibility."

# ==============================================================================
# 1. Source all ROS2 workspaces
# ==============================================================================
source /opt/ros/humble/setup.bash

if [ -f ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash ]; then
    source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
fi

if [ -f ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash ]; then
    source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
fi

# ==============================================================================
# 2. Source LLM config and set ROS domain
# ==============================================================================
if [ -f ~/.rosmaster_llm_config ]; then
    source ~/.rosmaster_llm_config
fi
export ROS_DOMAIN_ID=62

# ==============================================================================
# 3. Cleanup handler — set BEFORE any background launches so processes are
#    always cleaned up, even if a subsequent step fails under set -e
# ==============================================================================
BRINGUP_PID=""
LIDAR_PID=""
CAMERA_PID=""
TF_PID=""
VSLAM_PID=""

cleanup() {
    echo "[start_robot.sh] Cleaning up background processes..."
    kill $BRINGUP_PID $LIDAR_PID $CAMERA_PID $TF_BASE_CAMERA_PID $TF_PID $VSLAM_PID 2>/dev/null || true
    wait 2>/dev/null || true
    echo "[start_robot.sh] Cleanup complete."
}
trap cleanup EXIT

# ==============================================================================
# 4. Launch hardware nodes (background)
# ==============================================================================

# 4a. Robot base bringup (motors, odometry, EKF)
echo "[start_robot.sh] Starting robot bringup (A1)..."
ros2 launch yahboomcar_bringup yahboomcar_bringup_A1_launch.py &
BRINGUP_PID=$!
sleep 3

# 4b. LiDAR (SLLidar C1)
echo "[start_robot.sh] Starting LiDAR..."
ros2 launch sllidar_ros2 sllidar_c1_launch.py &
LIDAR_PID=$!
sleep 2

# 4c. Depth camera (OAK-D Pro via depthai-ros)
#     Parameters delivered via YAML params_file (CLI key:=value overrides are
#     silently ignored by camera.launch.py — it has no DeclareLaunchArgument).
#     oakd_params.yaml sets: RGBD pipeline, USB 3.0 SUPER speed, stereo
#     720P@15fps (raw), RGB 1080P@15fps MJPEG, explicit L/R rect topics, IR off.
#     CRITICAL: synced_rect_pair=false — true causes double-free crash in RGBD mode.
#     USB 3.0 target: ~32 MB/s total (8% of VL822 5Gbps hub capacity).
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "[start_robot.sh] Starting OAK-D Pro camera..."
ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-PRO \
    params_file:=${SCRIPT_DIR}/oakd_params.yaml &
CAMERA_PID=$!
sleep 3

# 4d. Static TF: base_link → camera_link
#     Bridges robot TF tree to camera TF tree.
#     Camera driver publishes camera_link → oak-d-base-frame → oak → camera frames.
#     Without this, VSLAM cannot find the base_link → camera_optical_frame chain.
#     Positional args: x y z yaw pitch roll frame_id child_frame_id
echo "[start_robot.sh] Publishing static TF (base_link -> camera_link)..."
ros2 run tf2_ros static_transform_publisher \
    0.05 0 0.1 0 0 0 base_link camera_link &
TF_BASE_CAMERA_PID=$!

# 4e. Static TF: camera_link → oak-d-base-frame
#     Camera is mounted right-side-up (confirmed via Phase 3 discovery).
echo "[start_robot.sh] Publishing static TF (camera_link -> oak-d-base-frame)..."
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 camera_link oak-d-base-frame &
TF_PID=$!

# ==============================================================================
# 5. Readiness checks — wait up to 30s for key topics
# ==============================================================================
echo "[start_robot.sh] Waiting for key topics..."

TIMEOUT=30
ELAPSED=0
READY=false

while [ $ELAPSED -lt $TIMEOUT ]; do
    TOPICS=$(ros2 topic list 2>/dev/null || true)

    HAS_SCAN=$(echo "$TOPICS" | grep -c "^/scan$" || true)
    HAS_ODOM=$(echo "$TOPICS" | grep -c "^/odom$" || true)
    HAS_IMAGE=$(echo "$TOPICS" | grep -c "/oak/rgb/image_raw" || true)

    if [ "$HAS_SCAN" -ge 1 ] && [ "$HAS_ODOM" -ge 1 ] && [ "$HAS_IMAGE" -ge 1 ]; then
        READY=true
        break
    fi

    sleep 2
    ELAPSED=$((ELAPSED + 2))
    echo "[start_robot.sh] Waiting... (${ELAPSED}/${TIMEOUT}s) scan=${HAS_SCAN} odom=${HAS_ODOM} image=${HAS_IMAGE}"
done

if [ "$READY" = true ]; then
    echo "[start_robot.sh] All key topics detected! System ready."
else
    echo "[start_robot.sh] WARNING: Not all topics detected after ${TIMEOUT}s. Proceeding anyway."
    echo "[start_robot.sh]   /scan: ${HAS_SCAN}  /odom: ${HAS_ODOM}  /oak/rgb/image_raw: ${HAS_IMAGE}"
fi

# ==============================================================================
# 6. Launch Isaac VSLAM (background)
#    Requires stereo camera to be publishing. Uses oakd_vslam.launch.py which
#    feeds mono8 rectified stereo directly to cuVSLAM (no CUDA converter).
#    voice_mapper.py auto-detects VSLAM odometry and sets vslam_available=True.
# ==============================================================================
if [ -f ${SCRIPT_DIR}/oakd_vslam.launch.py ]; then
    echo "[start_robot.sh] Starting Isaac VSLAM..."
    ros2 launch ${SCRIPT_DIR}/oakd_vslam.launch.py &
    VSLAM_PID=$!
    sleep 5

    # Quick check: is the VSLAM process still alive?
    if kill -0 $VSLAM_PID 2>/dev/null; then
        echo "[start_robot.sh] Isaac VSLAM launched (PID: $VSLAM_PID)"
    else
        echo "[start_robot.sh] WARNING: Isaac VSLAM process died. Continuing without VSLAM."
        VSLAM_PID=""
    fi
else
    echo "[start_robot.sh] oakd_vslam.launch.py not found — skipping VSLAM."
fi

# ==============================================================================
# 7. Launch voice_mapper.py
#    Run in foreground (not exec) so the EXIT trap fires on exit/signal,
#    cleaning up background hardware processes. This works both under
#    systemd (KillMode=mixed) and standalone invocation.
# ==============================================================================
echo "[start_robot.sh] Starting voice_mapper.py..."
cd ~/robot_scripts
python3 voice_mapper.py
EXIT_CODE=$?
echo "[start_robot.sh] voice_mapper.py exited with code ${EXIT_CODE}"
exit ${EXIT_CODE}
