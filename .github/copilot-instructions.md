# ROSMASTER A1 Robot Control - Copilot Instructions

## Project Overview

This is a ROS2 Humble robotics project for the **Yahboom ROSMASTER A1** robot with Jetson Orin Nano. The main script is `voice_mapper.py` - a comprehensive voice-controlled explorer with SLAM, Nav2, and Isaac VSLAM support.

## Robot Connection

| Setting | Value |
|---------|-------|
| **Robot IP** | `192.168.7.250` (DHCP - may change) |
| **SSH User** | `jetson` |
| **SSH Password** | `yahboom` |
| **ROS Domain ID** | `62` |

```bash
# Connect to robot
ssh jetson@192.168.7.250

# Required environment on robot
source /opt/ros/humble/setup.bash
source ~/.rosmaster_llm_config
export ROS_DOMAIN_ID=62
```

## Quick Start

```bash
# Run voice mapper (recommended)
./rosmaster_control.sh run

# Or run alternative explorer (USB 2.0 camera)
./rosmaster_control.sh yahboom

# Install as auto-start service
sudo ./rosmaster_control.sh service install

# Interactive menu
./rosmaster_control.sh menu
```

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  voice_mapper.py (main script)                          │
│  ├─ Whisper STT (48kHz→16kHz, PulseAudio index 32)     │
│  ├─ GPT-4o Brain + Vision                               │
│  ├─ OpenAI TTS → ffmpeg → aplay (3x volume)            │
│  ├─ Multi-camera: HP60C / OAK-D Pro / RealSense        │
│  ├─ Nav2 path planning + frontier exploration           │
│  ├─ SLAM: LiDAR (slam_toolbox) or VSLAM (Isaac)        │
│  └─ LiDAR: navigation & obstacle avoidance             │
├─────────────────────────────────────────────────────────┤
│  ROS2 Topics (verified working)                         │
│  ├─ /cmd_vel → /driver_node → motors                   │
│  ├─ /scan (720 pts @ 10Hz, BEST_EFFORT QoS)            │
│  ├─ /odom (wheel encoders working)                     │
│  ├─ /imu/data (Madgwick filtered)                      │
│  └─ /ascamera_hp60c/.../rgb0/image (RELIABLE QoS)      │
└─────────────────────────────────────────────────────────┘
```

## Scripts Overview

### Main Scripts
| Script | Purpose |
|--------|---------|
| `voice_mapper.py` | **Main** - Voice + Vision + Nav2 + SLAM + Isaac VSLAM |
| `yahboom_explorer.py` | Alternative - Yahboom depth camera pattern |
| `rosmaster_control.sh` | Master control with menu, service management |
| `llm_robot_brain.py` | Core LLM provider integration |

### Setup Scripts (numbered for order)
| Script | Purpose |
|--------|---------|
| `01_ssh_connect.sh` | SSH connection helper |
| `02_setup_depth_camera.sh` | HP60C camera setup |
| `03_setup_voice_commands.sh` | Audio/voice setup |
| `04_setup_navigation.sh` | Nav2 + SLAM setup |
| `05_autonomous_driving.sh` | Autonomous mode |
| `06_setup_llm_brain.sh` | LLM provider setup |
| `07_setup_imu.sh` | IMU configuration |

## rosmaster_control.sh Commands

```bash
./rosmaster_control.sh run          # Run voice_mapper.py (main)
./rosmaster_control.sh yahboom      # Run yahboom_explorer.py (alternative)
./rosmaster_control.sh start        # Start hardware only
./rosmaster_control.sh status       # Check system status
./rosmaster_control.sh stop         # Stop all processes
./rosmaster_control.sh install      # Install dependencies
./rosmaster_control.sh service install   # Install auto-start service
./rosmaster_control.sh service uninstall # Remove auto-start service
./rosmaster_control.sh service status    # Check service status
./rosmaster_control.sh service logs      # View service logs
./rosmaster_control.sh menu         # Interactive menu
```

## Yahboom Depth Camera Pattern

HP60C is USB 2.0 only - cannot stream RGB+Depth simultaneously. Use Yahboom's proven pattern:

```python
# RGB streaming (continuous) - stays within USB 2.0 bandwidth
self.rgb_sub = create_subscription(Image, ".../rgb0/image", ...)

# Depth queries (on-demand) - NOT continuous streaming
def get_dist(self, x: int, y: int) -> float:
    """Query depth at specific pixel - Yahboom pattern"""
    return self.latest_depth[y, x] / 1000.0  # mm to meters

# LiDAR for navigation (continuous) - primary nav sensor
self.scan_sub = create_subscription(LaserScan, "/scan", ...)
```

## Python ROS2 Nodes Pattern

- Inherit from `rclpy.node.Node`
- Use threading for LLM API calls (non-blocking)
- LLM responses must be JSON: `{"thinking": "...", "actions": [...], "response": "..."}`
- Action strings match `RobotActionLibrary.ACTIONS` dict keys in [llm_robot_brain.py](scripts/llm_robot_brain.py)

## Key ROS2 Topics

| Topic | Type | QoS | Purpose |
|-------|------|-----|---------|
| `/cmd_vel` | Twist | - | Motor velocity commands |
| `/scan` | LaserScan | BEST_EFFORT | LiDAR (720 points, ~10Hz) |
| `/odom` | Odometry | BEST_EFFORT | Wheel odometry |
| `/imu/data` | Imu | - | Filtered IMU (Madgwick) |
| `/ascamera_hp60c/camera_publisher/rgb0/image` | Image | RELIABLE | RGB camera (640x480@20fps) |

## Hardware Configuration

| Component | Model | Notes |
|-----------|-------|-------|
| **LiDAR** | SLLidar C1 | NOT YDLidar - use `sllidar_ros2` package |
| **Camera (current)** | Angstrong HP60C | USB 2.0 only - use on-demand depth queries |
| **Camera (upgrade)** | OAK-D Pro | USB 3.0, VSLAM-capable, IR illumination |
| **Audio** | C-Media USB | Use PulseAudio (index 32), 48kHz only |
| **IMU** | Built-in MPU | `/imu/data`, `/imu/data_raw`, `/imu/mag` |
| **Motors** | Built-in | `/driver_node` subscribes to `/cmd_vel` |

## Isaac VSLAM Support (OAK-D Pro)

The `voice_mapper.py` supports Isaac ROS Visual SLAM with stereo cameras:

### Camera Types (CameraType enum)
```python
class CameraType(Enum):
    HP60C = "hp60c"      # Current - LiDAR SLAM only
    OAK_D_PRO = "oakd"   # VSLAM-capable, recommended upgrade
    REALSENSE = "realsense"  # Alternative VSLAM camera
```

### SLAM Modes (SlamMode enum)
```python
class SlamMode(Enum):
    LIDAR = "lidar"      # slam_toolbox (default)
    VSLAM = "vslam"      # Isaac ROS Visual SLAM
    HYBRID = "hybrid"    # Both for maximum accuracy
```

### Isaac VSLAM Voice Commands
- **"Start visual SLAM"** - Launches Isaac cuVSLAM
- **"Stop visual SLAM"** - Stops VSLAM tracking
- **"VSLAM status"** - Reports tracking quality

### Installation
```bash
# On Jetson Orin Nano
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-depthai-ros  # For OAK-D Pro
```

## Known Issues

1. **Camera darkness**: HP60C produces very dark images (mean brightness ~4/255). Apply 10-20x brightness enhancement in software.
2. **Depth streaming**: HP60C is USB 2.0 only - use `get_dist(x,y)` on-demand queries instead of continuous streaming.
3. **Audio channels**: Direct USB mic access fails with "Invalid number of channels". Use PulseAudio device (index 32) instead.
4. **Threading**: Don't call `rclpy.spin_once()` from multiple threads - causes "generator already executing" error.

## LLM Provider Integration

Multi-provider support in [llm_robot_brain.py](scripts/llm_robot_brain.py):
- **OpenAI**: `OpenAIProvider` - supports vision via GPT-4o
- **Anthropic**: `AnthropicProvider` - Claude with image support
- **Ollama**: `OllamaProvider` - local inference, no API key

Config stored in `~/.rosmaster_llm_config`, loaded via environment variables.

## Configuration Files

| Path | Purpose |
|------|---------|
| `~/.rosmaster_llm_config` | LLM API keys (export OPENAI_API_KEY=...) |
| `~/maps/*.yaml` | Saved SLAM maps |
| `~/exploration_logs/*.yaml` | Exploration discoveries |
| `scripts/nav2_params.yaml` | Nav2 config for Ackerman steering |

## Development Workflow

```bash
# On dev machine: SSH to robot
./scripts/01_ssh_connect.sh connect

# Copy updated scripts to robot
scp scripts/voice_mapper.py jetson@192.168.7.250:~/robot_scripts/

# On robot: Run voice mapper
cd ~/robot_scripts
./rosmaster_control.sh run

# Or install as service for auto-start
sudo ./rosmaster_control.sh service install
```

## Adding New Voice Commands

1. Update `_build_system_prompt()` in `voice_mapper.py` to add command description
2. Add action handler in `execute()` method
3. Test with voice: speak the new command naturally

Example action in system prompt:
```python
# In _build_system_prompt():
"""
15. New Action: {"action": "my_action", "param": "value", "speech": "Doing action..."}
"""

# In execute():
elif action == "my_action":
    param = data.get("param", "default")
    # Do something
    self.speak(speech)
```

## Hardware Dependencies

- **Depth Camera**: Angstrong HP60C → `/ascamera_hp60c/camera_publisher/rgb0/image` (RELIABLE QoS)
- **LiDAR**: SLLidar C1 → `/scan` (720 points, BEST_EFFORT QoS)
- **IMU**: Built-in MPU → `/imu/data` (Madgwick filtered)
- **USB Audio**: C-Media → PulseAudio index 32 (48kHz, mono)
