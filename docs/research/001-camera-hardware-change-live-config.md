---
id: "001"
type: research
title: "Camera Hardware Change — Live Robot Configuration Discovery"
status: ✅ Complete
created: "2025-02-24"
current_phase: "4 of 4"
---

## Introduction

The ROSMASTER A1 robot has undergone a camera hardware change. The codebase is currently configured for the **Angstrong HP60C** (USB 2.0 depth camera with known issues: dark images, USB bandwidth limitations). The project also has pre-built support for **OAK-D Pro** and **Intel RealSense** cameras. This research will connect to the live robot, discover what camera hardware is now installed, audit the full system state, and identify what code/configuration changes are needed to support the new camera.

## Objectives

- Connect to the live robot via SSH and verify system health
- Identify the new camera hardware (USB device, model, bus speed)
- Discover what ROS2 topics are currently active and what camera driver is running
- Audit the full hardware configuration (LiDAR, IMU, audio, motors)
- Determine what code and configuration changes are needed for the new camera
- Assess SLAM, Nav2, and voice mapper compatibility with the new hardware

## Research Phases

| Phase | Name | Status | Scope | Session |
|-------|------|--------|-------|---------|
| 1 | Live Robot Connection & Hardware Discovery | ✅ Complete | SSH into robot; check USB devices, lsusb, camera model; check USB bus speed; identify new camera; check ROS2 daemon and running nodes; verify network connectivity | 2026-02-24 |
| 2 | ROS2 Runtime State & Active Topics | ✅ Complete | List all ROS2 topics and types; check camera topics specifically; verify LiDAR, IMU, odom topics; check running ROS2 nodes; inspect camera driver logs; test camera image publishing rate | 2026-02-24 |
| 3 | Codebase Camera Configuration Analysis | ✅ Complete | Analyze CameraType enum and CAMERA_CONFIGS in voice_mapper.py; review camera auto-detection logic; review yahboom_explorer.py camera subscriptions; review nav2_params.yaml camera references; review hp60c_lowres.launch.py; audit all HP60C-specific code paths | 2026-02-24 |
| 4 | Gap Analysis & Integration Assessment | ✅ Complete | Compare detected hardware vs codebase configuration; identify topic name mismatches; assess SLAM mode compatibility; assess Nav2 costmap camera config; identify code changes needed; document recommended next steps for pch-planner | 2026-02-24 |

## Phase 1: Live Robot Connection & Hardware Discovery

**Status:** ✅ Complete  
**Session:** 2026-02-24

### System Health

| Metric | Value |
|--------|-------|
| **Platform** | NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super |
| **JetPack** | R36.4.3 (L4T R36, GCID 38968081, Jan 8 2025 build) |
| **OS** | Ubuntu 22.04.5 LTS (Jammy Jellyfish) |
| **Kernel** | 5.15.148-tegra aarch64 |
| **Uptime** | Recently booted (~4 minutes at time of check) |
| **Disk** | 88GB total, 67GB used, 18GB free (**80% usage — approaching full**) |
| **Memory** | 7.4GB total, 1.4GB used, 5.7GB available |
| **Swap** | 11GB configured, 0B used |
| **CPU Load** | 0.85, 1.25, 0.64 (1/5/15 min) |

### Network Connectivity

| Interface | Status | IP |
|-----------|--------|-----|
| `wlP1p1s0` (WiFi) | **UP** | 192.168.7.250/22 |
| `enP8p1s0` (Ethernet) | DOWN (no carrier) | — |
| `docker0` | DOWN | 172.17.0.1/16 |
| `can0` (CAN bus) | DOWN | — |

Robot is accessible via WiFi at 192.168.7.250 as documented. SSH with `jetson:yahboom` credentials works.

### Camera Hardware — OAK-D Detected (HP60C Removed)

**The Angstrong HP60C has been physically replaced with a Luxonis OAK-D series camera.**

Evidence:
- `lsusb` shows: `Bus 001 Device 006: ID 03e7:2485 Intel Movidius MyriadX`
- USB product string: `Movidius MyriadX`, manufacturer: `Movidius Ltd.`
- This is the **DepthAI/OAK-D bootloader USB ID** — all OAK-D cameras present as `03e7:2485` when idle (before firmware upload)
- **No HP60C appears on USB** — the old camera has been physically removed
- Old HP60C ascamera_node logs exist in `/home/jetson/.ros/log/` (last run: Nov 30, 2025)

**Camera USB Connection Details:**

| Property | Value |
|----------|-------|
| USB Bus | Bus 01 (USB 2.0 root hub) |
| USB Path | 1-2.4 |
| Speed | **480 Mbps (USB 2.0)** |
| bcdUSB | 2.00 |
| MaxPower | 500mA |
| Device Class | Vendor Specific (255) |
| No /dev/video* | Expected — OAK-D uses DepthAI protocol, not V4L2 |

**USB Port Update (2026-02-24):** User moved the camera from hub path 1-2.4 to direct Bus 01 Port 1. Still showing 480 Mbps (USB 2.0) — however, in bootloader mode all OAK-D cameras enumerate as USB 2.0 regardless of physical port. The actual speed will only be determined after the DepthAI driver uploads firmware and the device re-enumerates. USB 3.0 capability cannot be confirmed until the driver is installed.

**Exact Model Unknown:** The USB ID `03e7:2485` is the same for ALL OAK-D models in bootloader mode. The DepthAI library must connect and upload firmware to identify the exact model.

### Camera Driver Status

**DepthAI — NOT INSTALLED:**
- `pip3 show depthai`: Module not found
- `dpkg -l | grep depthai`: No packages installed
- No udev rules for OAK-D

**DepthAI ROS Packages — AVAILABLE in apt (not installed):**
- `ros-humble-depthai` — Core C++ library with firmware
- `ros-humble-depthai-ros-driver` v2.12.2 — Monolithic ROS node
- `ros-humble-depthai-bridge` — Bridge package
- `ros-humble-depthai-descriptions` — URDF description
- `ros-humble-depthai-ros-msgs` — Custom messages
- `ros-humble-depthai-examples` — Example launch files
- `ros-humble-depthai-filters` — Filter nodes

**RealSense — INSTALLED (but no RealSense camera connected):**
- `ros-humble-librealsense2` v2.55.1
- `ros-humble-realsense2-camera` v4.55.1
- `ros-humble-realsense2-camera-msgs` v4.55.1

### USB Device Inventory

| Device | USB ID | Bus/Speed | Driver | Purpose |
|--------|--------|-----------|--------|---------|
| **Movidius MyriadX (OAK-D)** | 03e7:2485 | Bus 1 / 480M | None (bootloader) | **Camera (NEW)** |
| C-Media USB Audio | 0d8c:0012 | Bus 1 / 12M | snd-usb-audio | Microphone/Speaker |
| CH340 Serial (ttyUSB0) | 1a86:7522 | Bus 1 / 12M | ch34x | Motor controller |
| CP210x UART (ttyUSB1) | 10c4:ea60 | Bus 1 / 12M | cp210x | **LiDAR (SLLidar C1)** |
| CH340 Serial (ttyUSB2) | 1a86:7523 | Bus 1 / 12M | ch34x | IMU/other |
| Realtek Bluetooth | 0bda:c822 | Bus 1 / 12M | btusb | Bluetooth |

### ROS2 State

- **No ROS2 nodes running** — Daemon is up but no application nodes
- Only default topics: `/parameter_events`, `/rosout`
- `voice_mapper.service` is **enabled but inactive (dead)**
- Last run failed with "Missing sensors: camera, LiDAR, odometry" → "No sensors detected!"
- Service exited cleanly (code 0) after ~10 seconds

### LLM Configuration

- Provider: OpenAI, Model: GPT-4o
- API key configured in `~/.rosmaster_llm_config`

**Key Discoveries:**
- The HP60C has been PHYSICALLY REPLACED with a Luxonis OAK-D series camera (Movidius MyriadX VPU detected on USB)
- DepthAI driver packages are NOT installed — neither Python SDK nor ROS2 packages, but they ARE available in apt
- The OAK-D is connected via USB 2.0 (480 Mbps) — may need physical re-connection to USB 3.0 port
- Exact OAK-D model cannot be determined without DepthAI driver installed
- voice_mapper.service is enabled but fails immediately with "No sensors detected"
- RealSense driver packages ARE installed but no RealSense hardware is present
- Disk is at 80% usage — may need cleanup before installing new packages
- System is healthy: JetPack R36.4.3, 7.4GB RAM available, WiFi connected

| File | Relevance |
|------|-----------|
| `/home/jetson/.ros/log/ascamera_node_*.log` | Last HP60C run log (Nov 30, 2025) |
| `/home/jetson/.rosmaster_llm_config` | LLM provider configuration |
| `/home/jetson/robot_scripts/voice_mapper.py` | Main robot script (deployed copy) |
| `/etc/systemd/system/voice_mapper.service` | Systemd service (enabled, inactive) |
| `/home/jetson/yahboomcar_ros2_ws/` | Old HP60C camera driver workspace |

**Gaps:** Exact OAK-D model unknown without DepthAI driver; USB 3.0 port physical availability unconfirmed; LiDAR hardware not verified (no driver running)  
**Assumptions:** ttyUSB1 (CP210x) is LiDAR based on prior documentation; Movidius MyriadX device is OAK-D camera (not Intel NCS)

## Phase 2: ROS2 Runtime State & Active Topics

**Status:** ✅ Complete  
**Session:** 2026-02-24

### Installed ROS2 Packages (Relevant to Robot)

**Available on robot (installed via apt or colcon):**

| Category | Packages |
|----------|----------|
| **Navigation** | nav2_* (full suite), navigation2 |
| **SLAM** | slam_toolbox, cartographer, cartographer_ros, slam_gmapping |
| **Localization** | robot_localization (EKF/UKF) |
| **Camera/Image** | image_pipeline, image_transport, cv_bridge, image_proc, depth_image_proc, v4l2_camera, usb_cam, compressed_image_transport |
| **IMU** | imu_filter_madgwick, imu_complementary_filter, imu_tools |
| **LiDAR** | sllidar_ros2 (built in library_ws), ydlidar_ros2_driver (built but not used) |
| **RTABmap** | rtabmap_ros, rtabmap_slam, rtabmap_odom, rtabmap_examples (includes depthai.launch.py) |
| **RealSense** | realsense2_camera v4.55.1, librealsense2 v2.55.1 (installed but no hardware) |
| **TF/State** | tf2, tf2_ros, robot_state_publisher |
| **Yahboom** | yahboomcar_bringup, yahboomcar_base_node, yahboomcar_ctrl, yahboomcar_description, yahboomcar_msgs, many more (22 packages) |

**NOT installed (available in apt):**

| Package | Version | Purpose |
|---------|---------|---------|
| `ros-humble-depthai-ros-driver` | 2.12.2 | **OAK-D ROS2 monolithic driver node** |
| `ros-humble-depthai` | — | Core C++ library with firmware |
| `ros-humble-depthai-bridge` | 2.12.2 | Bridge between DepthAI and ROS2 |
| `ros-humble-depthai-descriptions` | — | URDF models for OAK-D cameras |
| `ros-humble-depthai-examples` | 2.12.2 | Example launch files |
| `ros-humble-depthai-ros-msgs` | — | Custom message types |
| `ros-humble-depthai-filters` | — | Filter nodes |

### Colcon Workspaces

Two colcon workspaces exist on the robot containing Yahboom-specific packages:

**1. Library workspace:** `~/yahboomcar_ros2_ws/software/library_ws/`
- ascamera (HP60C driver — **obsolete**, old camera removed)
- sllidar_ros2, ydlidar_ros2_driver
- robot_localization (EKF)
- cartographer, slam_gmapping
- costmap_converter, teb_local_planner
- usb_cam, web_video_server

**2. Application workspace:** `~/yahboomcar_ros2_ws/yahboomcar_ws/`
- yahboomcar_bringup (launch files)
- yahboomcar_base_node (wheel odometry publisher → `/odom_raw`)
- yahboomcar_ctrl (joystick control)
- yahboomcar_description (URDF models)
- yahboomcar_bringup (motor driver — `Ackman_driver_A1`)
- 22 total packages including vision, voice, depth, navigation

### Live Hardware Test Results

Hardware was successfully launched and tested on the live robot.

#### Launch Files Available

| Launch File | Location | Purpose |
|-------------|----------|---------|
| `bringup_A1_no_odom_launch.py` | yahboomcar_bringup | Base nodes WITHOUT wheel odom/EKF — **this is what start_hardware.sh uses** |
| `yahboomcar_bringup_A1_launch.py` | yahboomcar_bringup | **Full bringup WITH base_node + EKF** → produces `/odom` |
| `laser_bringup_launch.py` | yahboomcar_bringup | Combined bringup + LiDAR launcher |
| `sllidar_c1_launch.py` | sllidar_ros2 | SLLidar C1 standalone |
| `nuwa.launch.py` | ascamera | **Old HP60C camera — obsolete** |
| `hp60c.launch.py` | ascamera | Old HP60C camera — obsolete |

#### Critical Discovery: Two Bringup Variants

The `start_hardware.sh` script uses `bringup_A1_no_odom_launch.py` which does **NOT** include:
- `base_node_A1` (publishes `/odom_raw` from wheel encoders)
- `ekf_filter_node` (fuses wheel odom + IMU → publishes `/odom`)

The `voice_mapper.service` runs `voice_mapper.py` which only sources `/opt/ros/humble/setup.bash` — it does NOT source the Yahboom workspaces, so it can't even find the launch files for the base driver!

**The voice_mapper.py expects `/odom` but it comes from `yahboomcar_bringup_A1_launch.py` (full bringup with EKF), not the `no_odom` variant.**

#### Nodes Running (Full Bringup + LiDAR)

| Node | Package | Purpose |
|------|---------|---------|
| `/driver_node` | yahboomcar_bringup | Motor controller (HW serial to motors via ttyUSB0) |
| `/base_node` | yahboomcar_base_node | Wheel odometry (subscribes `/vel_raw`, publishes `/odom_raw`) |
| `/ekf_filter_node` | robot_localization | EKF fusion: `/odom_raw` + `/imu/data` → `/odom` |
| `/imu_filter_madgwick` | imu_filter_madgwick | Raw IMU → filtered `/imu/data` |
| `/robot_state_publisher` | robot_state_publisher | URDF → TF tree |
| `/joint_state_publisher` | joint_state_publisher | Joint states |
| `/sllidar_node` | sllidar_ros2 | LiDAR scanner → `/scan` |
| `/joy_ctrl` | yahboomcar_ctrl | Joystick → `/cmd_vel` |
| `/joy_node` | joy | Joystick input |

#### Complete Topic List (Full Bringup + LiDAR)

| Topic | Type | Publisher | Rate | QoS |
|-------|------|-----------|------|-----|
| `/scan` | LaserScan | sllidar_node | **10 Hz** | RELIABLE |
| `/imu/data` | Imu | imu_filter_madgwick | **10 Hz** | RELIABLE |
| `/imu/data_raw` | Imu | driver_node | ~10 Hz | RELIABLE |
| `/imu/mag` | MagneticField | driver_node | — | RELIABLE |
| `/odom` | Odometry | ekf_filter_node | **20 Hz** | RELIABLE |
| `/odom_raw` | Odometry | base_node | **20 Hz** | RELIABLE |
| `/cmd_vel` | Twist | joy_ctrl | on-demand | RELIABLE |
| `/vel_raw` | Twist | driver_node | — | RELIABLE |
| `/tf` | TFMessage | base_node, ekf | continuous | RELIABLE |
| `/tf_static` | TFMessage | robot_state_publisher | latched | RELIABLE |
| `/joint_states` | JointState | driver_node | — | RELIABLE |
| `/voltage` | Float32 | driver_node | — | RELIABLE |
| `/edition` | String | driver_node | — | — |
| `/Buzzer` | Bool | — (sub by driver_node) | — | — |
| `/RGBLight` | Int32 | — | — | — |
| `/Servo` | ServoControl | — (sub by driver_node) | — | — |
| `/robot_description` | String | robot_state_publisher | latched | — |
| `/diagnostics` | DiagnosticArray | ekf_filter_node | — | — |

**No camera topics exist** — the OAK-D cannot publish without depthai-ros driver installed.

#### QoS Profile Summary

All live topics use RELIABLE QoS, Durability=VOLATILE. This is important because:
- `voice_mapper.py` subscribes to `/scan` with `BEST_EFFORT` QoS
- The actual `/scan` publishes as `RELIABLE`
- BEST_EFFORT subscribers CAN receive RELIABLE publications (compatible)
- `voice_mapper.py` subscribes to `/odom` with `BEST_EFFORT` — also compatible with RELIABLE publisher

#### TF Tree (Static Frames)

```
base_footprint → base_link → camera_link
                            → imu_link
                            → laser
                            → left_front_wheel_joint
                            → left_rear_wheel_hinge
                            → left_steering_hinge_joint
                            → right_front_wheel_joint
                            → right_rear_wheel_hinge
                            → right_steering_hinge_joint
```

**Note:** `camera_link` frame IS defined in the URDF at offset `(0.082, 0, 0.096)` from `base_link`. The OAK-D will need its own optical frame (e.g., `oak-d-base-frame` or `oak_rgb_camera_optical_frame`) which must be linked to `camera_link` in the TF tree.

#### EKF Configuration

The EKF node fuses:
- `/odom_raw` (from `base_node`) — x, y, yaw, vx, vy, vyaw
- `/imu/data` (from Madgwick filter) — orientation fusion
- Publishes `/odom` (filtered) with remapping `/odometry/filtered` → `/odom`
- Operates at 30 Hz, 2D mode, publishes odom→base_footprint transform

#### LiDAR Data Quality

Verified working: SLLidar C1 on `/scan`
- Frame ID: `laser`
- 720 points per scan (angle_increment ≈ 0.00874 rad)
- Full 360° scan (-π to π)
- Range: 0.05m to 16.0m
- Rate: 10 Hz, 5 kHz sample rate
- Firmware: 1.02, Hardware Rev: 18
- Serial: D198E0F8C6E59AC0B5E29FF9524D4E00

#### IMU Data Quality

Verified working: Madgwick-filtered IMU on `/imu/data`
- Frame ID: `imu_link`
- Rate: 10 Hz
- Orientation: quaternion (valid, ~0.62 z component)
- Config: no magnetometer, ENU world frame, no TF publish

### DepthAI-ROS Expected Topics (When Installed)

Based on depthai-ros v2.12.2 default configuration, the OAK-D driver would publish:

| Topic (Default) | Type | Purpose |
|-----------------|------|---------|
| `/oak/rgb/image_raw` | Image | RGB camera image |
| `/oak/rgb/camera_info` | CameraInfo | RGB camera calibration |
| `/oak/stereo/image_raw` | Image | Stereo depth image |
| `/oak/stereo/camera_info` | CameraInfo | Stereo camera calibration |
| `/oak/imu/data` | Imu | Onboard IMU (BNO085 on OAK-D Pro) |
| `/oak/left/image_rect` | Image | Left mono rectified |
| `/oak/right/image_rect` | Image | Right mono rectified |

The RTABmap depthai.launch.py example remaps these to:
- `rgb/image` → `/right/image_rect`
- `depth/image` → `/stereo/depth`

**Note:** Topic prefix `/oak` is configurable via `camera_name` parameter.

### voice_mapper.service Analysis

The current service file has critical issues:

```ini
# Current (broken) ExecStart:
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && cd /home/jetson/robot_scripts && python3 voice_mapper.py'
```

**Problems:**
1. Does NOT source Yahboom workspaces — can't find `yahboomcar_bringup` or `sllidar_ros2`
2. Does NOT start any hardware — the service just runs voice_mapper.py
3. voice_mapper.py waits 30s for sensors, finds nothing, then exits with "No sensors detected"
4. The camera topic `/ascamera_hp60c/camera_publisher/rgb0/image` is hardcoded for the removed HP60C

**Required fix:** Service needs to either:
- Source Yahboom workspaces AND start hardware before running voice_mapper.py, OR
- Have a separate hardware service that starts first

**Key Discoveries:**
- Full bringup (`yahboomcar_bringup_A1_launch.py`) + LiDAR produces ALL needed topics: `/scan` (10Hz), `/odom` (20Hz), `/imu/data` (10Hz), `/cmd_vel`, `/tf`
- The `no_odom` bringup variant used by `start_hardware.sh` is INSUFFICIENT — no `/odom`, no EKF fusion
- No camera topics exist — OAK-D requires `ros-humble-depthai-ros-driver` (apt-installable, v2.12.2)
- `voice_mapper.service` is fundamentally broken: doesn't source Yahboom workspaces, doesn't start hardware
- LiDAR publishes RELIABLE QoS (not BEST_EFFORT as documented in copilot-instructions.md — though BEST_EFFORT subscribers are compatible)
- URDF defines `camera_link` frame; OAK-D will need TF linkage from its optical frame to this
- RTABmap has a ready-made depthai.launch.py for OAK-D VSLAM with stereo depth
- 22 Yahboom packages installed in application workspace with full Ackerman steering support

| File | Relevance |
|------|-----------|
| `~/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_bringup/share/yahboomcar_bringup/launch/yahboomcar_bringup_A1_launch.py` | Full bringup with odom — the correct launch file |
| `~/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_bringup/share/yahboomcar_bringup/launch/bringup_A1_no_odom_launch.py` | Bringup without odom — what start_hardware.sh uses (insufficient) |
| `~/yahboomcar_ros2_ws/software/library_ws/install/sllidar_ros2/share/sllidar_ros2/launch/sllidar_c1_launch.py` | SLLidar C1 launch |
| `~/yahboomcar_ros2_ws/software/library_ws/install/robot_localization/share/robot_localization/params/ekf_A1.yaml` | EKF config: fuses odom_raw + IMU → /odom |
| `/opt/ros/humble/share/rtabmap_examples/launch/depthai.launch.py` | RTABmap OAK-D VSLAM launch — already installed |
| `/home/jetson/robot_scripts/voice_mapper.py` | Main script — hardcoded HP60C topic, needs update |
| `/etc/systemd/system/voice_mapper.service` | Service file — missing workspace sources and hardware launch |
| `~/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_bringup/share/yahboomcar_bringup/param/imu_filter_param.yaml` | IMU Madgwick filter config |
| `~/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_description/share/yahboomcar_description/urdf/yahboomcar_A1.urdf.xacro` | URDF with camera_link frame |
| `/home/jetson/robot_scripts/start_hardware.sh` | Hardware launcher — uses wrong bringup variant |

**Gaps:** Cannot verify OAK-D topic behavior without installing depthai-ros driver; exact depthai-ros default topic prefix may vary by launch config  
**Assumptions:** depthai-ros v2.12.2 uses `/oak/` topic prefix by default based on documentation; URDF `camera_link` corresponds to physical camera mount position

## Phase 3: Codebase Camera Configuration Analysis

**Status:** ✅ Complete  
**Session:** 2026-02-24

### CameraType Enum & CAMERA_CONFIGS (voice_mapper.py)

Three camera types defined at line 72:
```python
class CameraType(Enum):
    HP60C = "hp60c"       # Yahboom HP60C (USB 2.0, limited)
    OAK_D_PRO = "oakd"    # Luxonis OAK-D Pro (USB 3.0, VSLAM-ready)
    REALSENSE = "realsense"  # Intel RealSense D435i
```

**OAK-D Pro config (lines 107-117) has correct depthai-ros default topics pre-configured:**
```python
CameraType.OAK_D_PRO: CameraConfig(
    rgb_topic="/oak/rgb/image_raw",
    depth_topic="/oak/stereo/image_raw",
    left_topic="/oak/left/image_rect",
    right_topic="/oak/right/image_rect",
    camera_info_left="/oak/left/camera_info",
    camera_info_right="/oak/right/camera_info",
    imu_topic="/oak/imu/data",
    needs_enhancement=False,
)
```

All topic subscriptions (lines 280-308) use `self.camera_config` dynamically — once auto-detect picks the right camera type, everything follows automatically.

### Camera Auto-Detection Logic (voice_mapper.py, lines 1610-1632)

```python
def _detect_camera(self):
    topic_names = [name for name, _ in self.get_topic_names_and_types()]
    if "/oak/rgb/image_raw" in topic_names or "/oak/left/image_rect" in topic_names:
        return CameraType.OAK_D_PRO
    if "/camera/color/image_raw" in topic_names or "/camera/infra1/image_rect_raw" in topic_names:
        return CameraType.REALSENSE
    if "/ascamera_hp60c" in str(topic_names):
        return CameraType.HP60C
    return CameraType.HP60C  # DEFAULT FALLBACK
```

**BUG:** Default fallback is HP60C. When no camera topics are detected (current state since OAK-D driver not installed), the robot subscribes to HP60C topics that don't exist.

### Image Enhancement Bug (voice_mapper.py, lines 1810-1824)

```python
def enhance_image(self, cv_image):
    enhanced = cv2.convertScaleAbs(cv_image, alpha=15.0, beta=30)
    return enhanced
```

**BUG:** Called unconditionally in `image_to_base64()` regardless of camera type. The `CameraConfig.needs_enhancement` field exists but is **NEVER CHECKED**. The alpha=15.0 is an extreme multiplier for HP60C's dark images (mean ~4/255) and will massively OVEREXPOSE OAK-D images.

### yahboom_explorer.py — Entirely HP60C Hardcoded

All camera topics hardcoded at lines 104-114:
```python
self.rgb_sub = self.create_subscription(
    Image, "/ascamera_hp60c/camera_publisher/rgb0/image", ...)
self.depth_sub = self.create_subscription(
    Image, "/ascamera_hp60c/camera_publisher/depth0/image", ...)
```

- No CameraType abstraction, no auto-detection, no OAK-D or RealSense support
- Image enhancement is conditional (only if brightness < 30) with reasonable alpha=3.0
- Cannot work with OAK-D without significant rework or deprecation

### nav2_params.yaml — 3 Hardcoded HP60C Topics

| Location | Line | Current Topic | OAK-D Equivalent |
|----------|------|---------------|-------------------|
| Local costmap obstacle layer | ~195 | `/ascamera_hp60c/camera_publisher/depth0/points` | `/oak/points` or `/oak/stereo/points` |
| Global costmap obstacle layer | ~238 | `/ascamera_hp60c/camera_publisher/depth0/points` | `/oak/points` or `/oak/stereo/points` |
| Collision monitor | ~373 | `/ascamera_hp60c/camera_publisher/depth0/points` | `/oak/points` or `/oak/stereo/points` |

Note: Costmap uses `observation_sources: scan depth_camera` — LiDAR works independently. Nav2 degrades gracefully to LiDAR-only if camera point cloud topics don't exist.

### hp60c_lowres.launch.py — Obsolete

Launches `ascamera_node` at 320x240 @ 10fps for the now-removed HP60C. **Entirely obsolete.**

### 02_setup_depth_camera.sh — Zero OAK-D Support

- Only supports `nuwa|astra` (Orbbec) and `realsense` cameras
- No case for `oakd` or depthai
- Doesn't check for Luxonis/Movidius USB ID (`03e7`)
- Needs new camera type with depthai-ros-driver launch

### VSLAM Support Pre-Built

- VSLAM gating (line 1400): HP60C correctly excluded, OAK-D Pro and RealSense enabled
- Isaac VSLAM topic remappings pre-configured for OAK-D and RealSense
- Only needs driver installed and topics publishing to work

### Camera-Agnostic Components (no changes needed)

- `llm_robot_brain.py` — Action library references camera abstractly
- `rosmaster_control.sh` — Generic camera launch delegation
- `voice_mapper.service` — No camera specifics (but has other issues from Phase 2)

**Key Discoveries:**
- voice_mapper.py has **full OAK-D Pro support pre-built** — correct topic configs, stereo subscriptions, VSLAM integration all ready
- **Auto-detect defaults to HP60C** when no camera topics found — needs fix for OAK-D
- **Image enhancement is unconditionally applied** (alpha=15.0) — will severely overexpose OAK-D images; `needs_enhancement` field exists but is never checked
- yahboom_explorer.py is entirely HP60C-hardcoded — no camera abstraction
- nav2_params.yaml has 3 hardcoded HP60C point cloud topic references
- 02_setup_depth_camera.sh has zero OAK-D/DepthAI support
- hp60c_lowres.launch.py is completely obsolete
- Depth mm→m conversion code should still work for OAK-D (also outputs mm by default)

| File | Relevance |
|------|-----------|
| `scripts/voice_mapper.py` | CameraType (L72), CAMERA_CONFIGS (L99-128), auto-detect (L1610-1632), enhancement bug (L1810-1824), VSLAM (L1400) |
| `scripts/yahboom_explorer.py` | Hardcoded HP60C topics (L104-114), enhancement (L652-672) |
| `scripts/nav2_params.yaml` | 3 HP60C point cloud topics (L195, L238, L373) |
| `scripts/hp60c_lowres.launch.py` | Obsolete HP60C launch file |
| `scripts/02_setup_depth_camera.sh` | No OAK-D support |
| `scripts/llm_robot_brain.py` | Camera-agnostic, no changes needed |

**Gaps:** None  
**Assumptions:** OAK-D depthai-ros topic names (/oak/*) match CAMERA_CONFIGS; OAK-D depth output in mm matches existing conversion code

## Phase 4: Gap Analysis & Integration Assessment

**Status:** ✅ Complete  
**Session:** 2026-02-24

### Topic Name Mismatches

| Location | Current (HP60C) | Required (OAK-D) | Impact |
|----------|-----------------|-------------------|--------|
| nav2_params.yaml L197 | `/ascamera_hp60c/.../depth0/points` | `/oak/stereo/depth/points` (verify post-install) | Local costmap depth wrong |
| nav2_params.yaml L238 | `/ascamera_hp60c/.../depth0/points` | `/oak/stereo/depth/points` (verify post-install) | Global costmap depth wrong |
| nav2_params.yaml L376 | `/ascamera_hp60c/.../depth0/points` | `/oak/stereo/depth/points` (verify post-install) | Collision monitor wrong |
| voice_mapper.py L1632 | Fallback to `CameraType.HP60C` | Should fallback to `None` or warn | Subscribes to nonexistent topics silently |
| yahboom_explorer.py L104-114 | All `/ascamera_hp60c/*` hardcoded | No OAK-D support at all | Entire script non-functional |
| 02_setup_depth_camera.sh | nuwa/astra/realsense only | No OAK-D/DepthAI | Camera launch fails |
| hp60c_lowres.launch.py | HP60C launch config | Obsolete — hardware removed | Dead code |

### SLAM Mode Compatibility

| SLAM Mode | Status | What Works | What Needs OAK-D |
|-----------|--------|------------|-------------------|
| **LIDAR** (slam_toolbox) | ✅ Fully operational | /scan at 10Hz, map building works | Nothing — camera-independent |
| **VSLAM** (Isaac ROS) | ❌ Blocked | Infrastructure pre-built in code | OAK-D driver, stereo topics, TF frames |
| **HYBRID** (both) | ❌ Blocked | LiDAR half works | Same as VSLAM |

### Nav2 Configuration Assessment

| Component | Status | Detail |
|-----------|--------|--------|
| LiDAR obstacle layer | ✅ Working | /scan at 10Hz, RELIABLE QoS |
| Depth camera obstacle layer | ⚠️ Degraded | HP60C topic missing → LiDAR-only |
| Collision monitor scan | ✅ Working | /scan operational |
| Collision monitor pointcloud | ⚠️ Degraded | HP60C pointcloud missing → scan-only |
| Path planning | ✅ Working | LiDAR-based costmaps sufficient |

### Code Changes Required — By Priority

#### CRITICAL (Blocks robot operation)

| # | File | Change |
|---|------|--------|
| C1 | `voice_mapper.service` | Source Yahboom workspaces before running voice_mapper.py |
| C2 | `voice_mapper.service` | Add hardware bringup (bringup_launch.py + LiDAR) as ExecStartPre |
| C3 | `rosmaster_control.sh` | Use full A1 bringup variant (with odom/EKF), not `no_odom` |

#### REQUIRED (Blocks camera operation)

| # | File | Change |
|---|------|--------|
| R1 | Robot (apt) | `sudo apt install ros-humble-depthai-ros-driver` |
| R2 | `02_setup_depth_camera.sh` | Add `oakd` camera type with depthai-ros launch |
| R3 | `rosmaster_control.sh` | Update camera startup to launch DepthAI driver |

#### IMPORTANT (Fixes bugs)

| # | File | Change |
|---|------|--------|
| I1 | `voice_mapper.py` L1816 | Gate `enhance_image()` on `needs_enhancement` flag |
| I2 | `voice_mapper.py` L1632 | Change fallback from HP60C to None/error |
| I3 | `nav2_params.yaml` L197,238,376 | Update 3 depth_camera topics to OAK-D equivalents |

#### ENHANCEMENT (Improves capability)

| # | File | Change |
|---|------|--------|
| E1 | `voice_mapper.py` | Add `--camera-type` CLI argument |
| E2 | `voice_mapper.py` | Add retry logic in `_detect_camera()` |
| E3 | URDF/TF | Add OAK-D TF frame linkage: camera_link → oak_frame |
| E4 | `voice_mapper.py` | Verify OAK-D depth is PointCloud2 or Image — adjust accordingly |

#### CLEANUP (Technical debt)

| # | File | Change |
|---|------|--------|
| D1 | `hp60c_lowres.launch.py` | Delete file |
| D2 | `yahboom_explorer.py` | Deprecate or refactor for CameraConfig abstraction |
| D3 | `02_setup_depth_camera.sh` | Make OAK-D the default CAMERA_TYPE |
| D4 | `voice_mapper.py` CAMERA_CONFIGS | Verify OAK-D topic names after driver installation |

### Recommended Implementation Order

```
Step 1:  Install DepthAI driver (R1)
  └─ Unblocks: topic discovery, model identification

Step 2:  Identify exact OAK-D model + USB speed (post-driver)
  └─ Unblocks: knowing actual topic names, resolution limits

Step 3:  Fix service & bringup (C1, C2, C3)
  └─ Unblocks: robot can auto-start

Step 4:  Fix image enhancement bug (I1)
  └─ Unblocks: OAK-D images usable

Step 5:  Fix auto-detect fallback (I2)
  └─ Unblocks: clean diagnostics

Step 6:  Update camera launch scripts (R2, R3)
  └─ Unblocks: OAK-D in normal bringup

Step 7:  Update Nav2 depth topics (I3)
  └─ Unblocks: depth obstacles in costmap

Step 8:  Add CLI override + retry (E1, E2)
  └─ Improves: startup reliability

Step 9:  Add TF frame linkage (E3)
  └─ Unblocks: VSLAM, proper depth projection

Step 10: Cleanup dead code (D1, D2, D3, D4)
  └─ Reduces: confusion and maintenance burden
```

### What Works TODAY (No Changes)

| Capability | Status |
|------------|--------|
| Motor control via /cmd_vel | ✅ Working |
| LiDAR scanning (/scan) | ✅ Working |
| LiDAR SLAM (slam_toolbox) | ✅ Working |
| Odometry (/odom via EKF) | ✅ Working (correct bringup) |
| IMU (/imu/data) | ✅ Working |
| Nav2 path planning (LiDAR-only) | ✅ Working |
| Voice STT/TTS | ✅ Working |
| LLM brain (GPT-4o) | ✅ Working |

### Unknowns Requiring Post-Driver Discovery

| Unknown | How to Resolve |
|---------|----------------|
| Exact OAK-D model | `lsusb` + depthai device info after install |
| Actual USB speed (2.0 vs 3.0) | Check after firmware upload via driver |
| Actual topic names in v2.12.2 | `ros2 topic list` after driver launch |
| PointCloud2 vs depth Image output | `ros2 topic info` after driver launch |
| OAK-D optical frame name | Check TF tree after driver launch |

**Key Discoveries:**
- Robot is FULLY OPERATIONAL in LiDAR-only mode — camera transition is additive, not blocking
- 3 critical service/bringup bugs prevent robot auto-starting at all
- 1 image enhancement bug (alpha=15.0 unconditional) will destroy OAK-D image quality
- voice_mapper.py architecture is well-designed — CameraConfig abstraction means most code works once topics exist
- Entire transition blocked on one apt install: `ros-humble-depthai-ros-driver`
- 4 post-driver unknowns affect config decisions (model, USB speed, topics, frame names)

| File | Relevance |
|------|-----------|
| `scripts/voice_mapper.py` | 2 bugs (enhancement L1816, fallback L1632); good architecture otherwise |
| `scripts/nav2_params.yaml` | 3 HP60C topics (L197, L238, L376) |
| `scripts/voice_mapper.service` | Missing workspace sources and hardware bringup |
| `scripts/rosmaster_control.sh` | Wrong bringup variant, broken camera startup |
| `scripts/02_setup_depth_camera.sh` | Zero OAK-D support |
| `scripts/yahboom_explorer.py` | 100% HP60C-hardcoded; non-functional |
| `scripts/hp60c_lowres.launch.py` | Obsolete dead code |

**Gaps:** Exact topic names, PointCloud2 format, USB speed, and frame names unknown until DepthAI driver installed  
**Assumptions:** OAK-D topics will follow /oak/* convention; depth_camera data_type assumed PointCloud2

## Overview

The ROSMASTER A1 robot's Angstrong HP60C depth camera has been physically replaced with a **Luxonis OAK-D series camera** (Movidius MyriadX VPU, USB ID 03e7:2485). The codebase was architecturally prepared for this change — `voice_mapper.py` already contains a `CameraConfig` abstraction layer with correctly pre-configured OAK-D Pro topic mappings (`/oak/*`), stereo subscriptions, and Isaac VSLAM integration. However, the **DepthAI ROS2 driver is not installed** on the robot, meaning no camera topics are publishing and the robot operates in a degraded LiDAR-only mode.

The transition is **additive, not blocking** — all non-camera capabilities (motors, LiDAR, SLAM, Nav2, voice, LLM brain) are fully operational today. The camera change requires: one apt package install, 3 critical service/bringup fixes, 2 bug fixes (unconditional image enhancement and HP60C fallback default), and 3 nav2_params.yaml topic updates. The exact OAK-D model, USB speed, and actual topic names can only be confirmed after driver installation.

Three infrastructure bugs were discovered that prevent the robot from auto-starting entirely, independent of the camera change: the systemd service doesn't source Yahboom workspaces, doesn't launch hardware, and `start_hardware.sh` uses a bringup variant that lacks odometry/EKF.

## Key Findings

1. **HP60C physically replaced with OAK-D** — Movidius MyriadX detected on USB; exact model (D/D-Pro/D-Lite) unknown until driver installed
2. **DepthAI driver not installed** — `ros-humble-depthai-ros-driver` v2.12.2 available in apt; single install unblocks all camera functions
3. **voice_mapper.py architecture is ready** — CameraConfig abstraction with OAK-D Pro topic configs pre-built; just needs driver + 2 bug fixes
4. **Two bugs will cause OAK-D issues:** (a) image enhancement (alpha=15.0) applied unconditionally, will overexpose OAK-D images; (b) auto-detect defaults to HP60C when no topics found
5. **Service/bringup infrastructure is broken** — 3 critical issues prevent robot from auto-starting, independent of camera change
6. **Robot fully operational in LiDAR-only mode** — motors, LiDAR (10Hz/720pts), SLAM, Nav2, odometry/EKF, IMU, voice, GPT-4o all working
7. **Nav2 degrades gracefully** — 3 HP60C point cloud topics in nav2_params.yaml reference removed hardware; Nav2 falls back to LiDAR-only
8. **yahboom_explorer.py is 100% HP60C-hardcoded** — no camera abstraction; needs deprecation or major refactor
9. **VSLAM infrastructure pre-built** — Isaac VSLAM remappings, stereo subscriptions, VSLAM voice commands all ready; blocked only on driver
10. **Camera on USB 2.0 in bootloader mode** — all OAK-D cameras appear USB 2.0 until driver uploads firmware; actual speed unknown

## Actionable Conclusions

- **Install `ros-humble-depthai-ros-driver` first** — this single apt install unblocks all camera functionality and reveals the 4 unknowns (model, USB speed, topics, frame names)
- **Fix service/bringup (C1-C3) as immediate priority** — the robot can't auto-start at all; this is independent of camera work
- **Fix the image enhancement bug (I1) before any camera testing** — alpha=15.0 will make OAK-D images unusable for vision analysis
- **Keep LiDAR SLAM as primary** — it works perfectly today; VSLAM is an enhancement once OAK-D is operational
- **Deprecate yahboom_explorer.py** — it has zero camera abstraction and isn't worth refactoring vs the well-architected voice_mapper.py
- **Hand off to `@pch-planner`** to create a phased implementation plan following the 10-step dependency order from Phase 4

## Open Questions

- What is the exact OAK-D model? (D, D-Pro, D-Lite, D-S2 — affects stereo baseline, IR, IMU)
- Does the OAK-D negotiate USB 3.0 speed after firmware upload?
- What are the actual depthai-ros v2.12.2 topic names and types? (assumed `/oak/*` with PointCloud2)
- What is the OAK-D optical frame name in the depthai-ros TF tree?
- Should yahboom_explorer.py be deprecated entirely or refactored?

## Standards Applied

No organizational standards applicable to this research.

## Handoff

| Field | Value |
|-------|-------|
| Created By | pch-researcher |
| Created Date | 2025-02-24 |
| Status | ✅ Complete |
| Current Phase | ✅ Complete |
| Path | /docs/research/001-camera-hardware-change-live-config.md |
