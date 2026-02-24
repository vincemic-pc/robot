# ROSMASTER A1 Robot Control Scripts

This repository contains scripts to setup and control the **Yahboom ROSMASTER A1** robot with Jetson Orin Nano. These scripts enable:

- 🎥 **3D Depth Camera** setup and streaming (Angstrong HP60C)
- 🎤 **Voice Commands** with OpenAI Whisper STT
- 🧠 **LLM Brain** with GPT-4o reasoning + vision
- 🔊 **Text-to-Speech** with OpenAI TTS
- 🔭 **LiDAR Navigation** with obstacle avoidance (SLLidar C1)
- 🗺️ **SLAM Mapping** and autonomous navigation
- 🚗 **Autonomous Exploration** with visual observations

## ✅ Verified Working Configuration

| Component | Status | Notes |
|-----------|--------|-------|
| **SSH Access** | ✅ Working | `ssh jetson@192.168.7.250` (pwd: yahboom) |
| **Motor Control** | ✅ Working | `/cmd_vel` → `/driver_node` → wheels move |
| **Odometry** | ✅ Working | `/odom` tracks position accurately |
| **LiDAR** | ✅ Working | 720 points/scan @ 10Hz, BEST_EFFORT QoS |
| **IMU** | ✅ Working | `/imu/data` filtered by Madgwick |
| **Voice Input** | ✅ Working | PulseAudio index 32, 48kHz→16kHz resampling |
| **Whisper STT** | ✅ Working | English transcription via OpenAI API |
| **GPT-4o Brain** | ✅ Working | Natural language + JSON action parsing |
| **GPT-4o Vision** | ✅ Working | Camera image analysis |
| **TTS Output** | ✅ Working | OpenAI TTS → ffmpeg → aplay (3x volume boost) |
| **Nav2 Navigation** | ✅ Working | Regulated Pure Pursuit controller |
| **SLAM Mapping** | ✅ Working | slam_toolbox online_sync |
| **Error Alerts** | ✅ Working | Beeps + speech for sensor/nav failures |
| **RGB Camera** | ⚠️ Dark Images | HP60C RGB works but gain=4, needs software enhancement |
| **Depth Camera** | ⚠️ On-Demand Only | HP60C USB 2.0 - use Yahboom's `get_dist(x,y)` pattern, NOT streaming |

## 🔧 Hardware Configuration

### Network Configuration

| Setting | Value |
|---------|-------|
| **Robot IP** | `192.168.7.250` (DHCP, may vary) |
| **SSH User** | `jetson` |
| **SSH Password** | `yahboom` |
| **ROS Domain ID** | `62` |

### Components

| Component | Model | Connection | Notes |
|-----------|-------|------------|-------|
| **Robot Platform** | Yahboom ROSMASTER A1 | - | Ackerman steering chassis |
| **Compute** | Jetson Orin Nano | - | Ubuntu 22.04, ROS2 Humble |
| **LiDAR** | **SLLidar C1** | `/dev/ttyUSB0` (cp210x) | 720 points/scan, ~10Hz |
| **Depth Camera** | **Angstrong HP60C** | USB 2.0 | RGB streaming works; depth via on-demand `get_dist(x,y)` |
| **IMU** | Built-in MPU | I2C | `/imu/data`, `/imu/data_raw`, `/imu/mag` |
| **Voice Module** | Yahboom AI Voice | Serial `/dev/myspeech` | Chinese wake word only |
| **USB Audio** | C-Media USB Audio | Card 0 | 48kHz/44.1kHz, PulseAudio index 32 |
| **Motor Controller** | Built-in | `/dev/ttyUSB1` (ch34x) | Responds to `/cmd_vel` |

### USB Device Mapping
```
/dev/ttyUSB0 → LiDAR (cp210x driver)
/dev/ttyUSB1 → Motor controller (ch34x driver)
/dev/ttyUSB2 → AI Voice Module (ch34x) → symlinked to /dev/myspeech
```

### Battery Monitoring

The robot publishes battery voltage on `/voltage` topic:
```bash
ros2 topic echo /voltage --once
```

| Voltage | Status | Notes |
|---------|--------|-------|
| 12.6V | Full | 3S LiPo fully charged |
| 11.1V | Nominal | Normal operating voltage |
| 10.5V | ⚠️ Low | Robot beeps - charge soon! |
| 9.9V | ❌ Critical | Risk of damage - charge immediately |

**Low battery symptoms:**
- Continuous beeping from the robot
- Motors may become sluggish
- Unexpected shutdowns

**Charge the battery when voltage drops below 10.5V!**

### Audio Hardware Details

The USB Audio Device has specific limitations:
- **Supported sample rates**: 48000 Hz, 44100 Hz only (NOT 16kHz)
- **Channels**: 1 (mono) for capture, 2 (stereo) for playback
- **Format**: S16_LE (16-bit signed little-endian)
- **PulseAudio index**: 32 (preferred for PyAudio)

⚠️ **Important**: 
- OpenAI Whisper expects 16kHz audio. Scripts automatically resample from 48kHz to 16kHz using scipy.
- Direct USB device (index 0) may show "Invalid number of channels" error - use PulseAudio instead.

### Camera Known Issues

#### RGB Camera - Dark Images
The **Angstrong HP60C** camera produces very dark images:
- **Symptom**: Images have mean brightness ~4/255, max ~102/255
- **Cause**: Camera firmware sets gain=4 (very low), no SDK exposure controls
- **Workaround**: Apply software brightness enhancement (10-20x multiplier) in image processing
- **SDK**: Angstrong camera sdk v1.2.28.20241021 (proprietary, encrypted config)

RGB topic: `/ascamera_hp60c/camera_publisher/rgb0/image` (RELIABLE QoS)

#### Depth Camera - USB 2.0 Bandwidth Limitation
The HP60C **cannot stream depth continuously** due to USB 2.0 hardware, but **works with Yahboom's on-demand query pattern**:

- **Root Cause**: Camera reports `bcdUSB 2.00` - it's USB 2.0 only, NOT USB 3.0
- **Symptom**: Continuous RGB+Depth streaming fails after ~40-60 seconds
- **Bandwidth Issue**: 640×480 RGB + Depth @ 25fps = ~38 MB/s, exceeds USB 2.0 limit (~35 MB/s)

**✅ Yahboom's Proven Pattern** (discovered from their GitHub):
Yahboom tutorials use depth camera for **on-demand single-point queries**, NOT continuous streaming:
```python
# Yahboom's approach: query individual pixels when needed
def get_dist(x, y):
    """Get distance to pixel (x,y) in meters"""
    depth_value = depth_image[y, x]
    return depth_value / 1000.0  # Convert mm to meters

# Example: Find distance to detected object
distance = get_dist(320, 240)  # Center of frame
```

This works because:
1. RGB-only streaming stays within USB 2.0 bandwidth (~23 MB/s)
2. Depth frames are cached and queried on-demand
3. LiDAR handles all continuous navigation needs

**Implementation**: See `yahboom_explorer.py` for the working pattern.

Depth topics (unreliable):
- `/ascamera_hp60c/camera_publisher/depth0/image_raw`
- `/ascamera_hp60c/camera_publisher/depth0/points`

## 📋 Prerequisites

- **Robot**: Yahboom ROSMASTER A1 with Jetson Orin Nano
- **OS**: Ubuntu 22.04 with ROS2 Humble
- **Hardware**: 
  - Angstrong HP60C Depth Camera (NOT Nuwa/Astra - despite similar names)
  - SLLidar C1 (NOT YDLidar/T-Mini)
  - USB Audio Device for voice control

## 📦 Dependencies

### Python Packages
```bash
pip3 install openai pyaudio numpy scipy
```

### System Packages
```bash
sudo apt-get install mpv ffmpeg portaudio19-dev python3-pyaudio
```

### ROS2 Packages (Pre-installed)
- `sllidar_ros2` - SLLidar driver
- `ascamera` - Nuwa camera driver
- `yahboomcar_bringup` - Robot base driver
- `nav2_bringup` - Navigation stack

### API Keys

**OpenAI** (Required for `voice_mapper.py`):
```bash
echo 'export OPENAI_API_KEY="sk-proj-..."' >> ~/.rosmaster_llm_config
```

**Alternative - Ollama** (Free, Local):
```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3.2
echo 'export LLM_PROVIDER=ollama' >> ~/.rosmaster_llm_config
```

## 📊 ROS2 Topics

### Active Topics (Verified)

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/cmd_vel` | geometry_msgs/Twist | - | Velocity commands to motors |
| `/scan` | sensor_msgs/LaserScan | BEST_EFFORT | LiDAR data (720 points) |
| `/imu/data` | sensor_msgs/Imu | - | Filtered IMU (Madgwick filter) |
| `/imu/data_raw` | sensor_msgs/Imu | - | Raw IMU readings |
| `/imu/mag` | sensor_msgs/MagneticField | - | Magnetometer data |
| `/odom` | nav_msgs/Odometry | BEST_EFFORT | Wheel odometry |
| `/ascamera_hp60c/camera_publisher/rgb0/image` | sensor_msgs/Image | RELIABLE | RGB camera (640x480@20fps) |
| `/ascamera_hp60c/camera_publisher/depth0/image` | sensor_msgs/Image | RELIABLE | Depth image |
| `/joint_states` | sensor_msgs/JointState | - | Wheel positions |
| `/voltage` | std_msgs/Float32 | - | Battery voltage |
| `/Buzzer` | std_msgs/Int32 | - | Buzzer control |
| `/RGBLight` | std_msgs/Int32 | - | LED control |

### Active Nodes (Base System)

| Node | Purpose |
|------|---------|
| `/driver_node` | Motor driver, subscribes to `/cmd_vel` |
| `/sllidar_node` | LiDAR driver |
| `/ascamera_hp60c/camera_publisher` | Camera driver |
| `/imu_filter_madgwick` | IMU sensor fusion |
| `/base_node` | Robot base |
| `/joint_state_publisher` | Joint states |
| `/robot_state_publisher` | Robot TF |

## 🚀 Quick Start

### 1. Connect to the Robot

First, ensure you're connected to the robot's WiFi network, then:

```bash
# SSH into the robot (password: yahboom)
ssh jetson@192.168.7.250

# Or use the helper script
./scripts/01_ssh_connect.sh connect

# Setup passwordless SSH
./scripts/01_ssh_connect.sh setup-key
```

**Network Configuration:**
| Setting | Value |
|---------|-------|
| Robot IP | `192.168.7.250` (DHCP - may change) |
| SSH User | `jetson` |
| SSH Password | `yahboom` |
| ROS Domain ID | `62` |

### 2. Install Dependencies (First Time Only)

```bash
# On the robot, run:
./scripts/rosmaster_control.sh install
```

### 3. Start Hardware

```bash
# Start robot base, LiDAR, and camera
cd ~/robot_scripts
./start_hardware.sh
```

This launches:
- Robot base driver with servo initialization (`INIT_SERVO_S1=90`, `INIT_SERVO_S2=90`)
- SLLidar C1 (`sllidar_ros2 sllidar_c1_launch.py`)
- Nuwa HP60C camera (from correct directory for config files)

### 4. Start Voice-Controlled Explorer (Recommended)

**Option A: Yahboom-Style Explorer** (best for depth camera)
```bash
cd ~/robot_scripts
source /opt/ros/humble/setup.bash
source ~/.rosmaster_llm_config
export ROS_DOMAIN_ID=62
python3 yahboom_explorer.py
```
Uses Yahboom's proven depth camera pattern: RGB streaming + on-demand depth queries + LiDAR navigation.

**Option B: Voice Mapper with Nav2** (for SLAM mapping)

The voice_mapper.py is the most complete solution with Nav2 integration, SLAM mapping, frontier exploration, and comprehensive error alerts.

```bash
# The voice_mapper runs as a systemd service (auto-starts on boot)
sudo systemctl status voice_mapper.service

# View logs
journalctl -u voice_mapper.service -f

# Restart if needed
sudo systemctl restart voice_mapper.service
```

**Or run manually:**
```bash
cd ~/robot_scripts
source /opt/ros/humble/setup.bash
source ~/.rosmaster_llm_config
export ROS_DOMAIN_ID=62
python3 voice_mapper.py
```

This all-in-one script provides:
- 🎤 **Whisper STT** - Always-listening voice recognition (PulseAudio, 48kHz→16kHz resampling)
- 🧠 **GPT-4o** - Natural language understanding + vision
- 🔊 **OpenAI TTS** - Voice responses with 3x volume boost via ffmpeg
- 📷 **Camera Vision** - GPT-4o describes what it sees (with brightness enhancement)
- 🗺️ **SLAM Mapping** - Real-time map building with slam_toolbox
- 🚀 **Nav2 Navigation** - Path planning with Regulated Pure Pursuit controller
- 🚪 **Frontier Exploration** - Automatically discovers rooms and doorways
- ⚠️ **Error Alerts** - Beeps and speech for sensor/navigation failures
- 🔭 **LiDAR Obstacle Avoidance** - Multi-sector safety monitoring
- 🎯 **Isaac VSLAM** - Visual SLAM support for OAK-D Pro camera (when available)

### Alternative: Yahboom Explorer

```bash
python3 yahboom_explorer.py
```

Uses Yahboom's proven depth camera pattern - better for USB 2.0 bandwidth constraints.

### 5. Voice Commands

Speak naturally:
- **Mapping**: "Start mapping", "Save the map", "Map status"
- **Exploration**: "Start exploring", "Stop exploring", "Explore the area"
- **Movement**: "Move forward", "Turn left", "Back up", "Spin around"
- **Vision**: "What do you see?", "Look around", "Describe your surroundings"
- **Status**: "How far have you gone?", "Status report"
- **Conversation**: Ask anything!

## 🗺️ Voice Mapper Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    voice_mapper.py Architecture                  │
│                                                                  │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐         │
│  │   Whisper    │   │   GPT-4o     │   │    TTS       │         │
│  │  (48k→16kHz) │ → │  Brain+Vision│ → │  (nova, 3x)  │         │
│  │  PulseAudio  │   │              │   │  ffmpeg+aplay│         │
│  └──────────────┘   └──────────────┘   └──────────────┘         │
│         ↑                  ↓                   ↑                 │
│    Voice Input       JSON Actions         Error Alerts          │
│                           ↓                    ↑                 │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐         │
│  │   Camera     │   │    Nav2      │   │   Beep()     │         │
│  │   HP60C      │   │  Navigation  │   │  Alerts      │         │
│  │  (enhanced)  │   │  Pure Pursuit│   │  (ffmpeg)    │         │
│  └──────────────┘   └──────────────┘   └──────────────┘         │
│                           ↓                                      │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐         │
│  │   SLAM       │   │  /cmd_vel    │   │   LiDAR      │         │
│  │ slam_toolbox │   │  Publisher   │   │   /scan      │         │
│  │              │   │              │   │  (720 pts)   │         │
│  └──────────────┘   └──────────────┘   └──────────────┘         │
│                                                                  │
│  Exploration Mode: Frontier-based exploration with Nav2 path    │
│  planning, automatic doorway detection, and room discovery      │
└─────────────────────────────────────────────────────────────────┘
```

## 🤖 Yahboom Explorer Architecture (Recommended)

The `yahboom_explorer.py` uses **Yahboom's proven depth camera pattern** discovered from their tutorials:

```
┌─────────────────────────────────────────────────────────────────┐
│                  yahboom_explorer.py Architecture                │
│                                                                  │
│  SENSOR STRATEGY (matches Yahboom tutorials):                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ RGB Stream (continuous) → GPT-4o vision, color following │   │
│  │ Depth Camera (on-demand) → get_dist(x,y) pixel queries   │   │
│  │ LiDAR (continuous) → Navigation & obstacle avoidance     │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐         │
│  │   Whisper    │   │   GPT-4o     │   │    TTS       │         │
│  │  (48k→16kHz) │ → │  Brain+Vision│ → │  (nova)      │         │
│  │  PulseAudio  │   │              │   │  aplay/mpv   │         │
│  └──────────────┘   └──────────────┘   └──────────────┘         │
│         ↑                  ↓                                     │
│    Voice Input       JSON Actions                                │
│                           ↓                                      │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐         │
│  │   RGB        │   │  Depth Query │   │   LiDAR      │         │
│  │   Camera     │   │  get_dist()  │   │   /scan      │         │
│  │   (HP60C)    │   │  on-demand   │   │  720 pts     │         │
│  └──────────────┘   └──────────────┘   └──────────────┘         │
│         │                  │                  │                  │
│         ▼                  ▼                  ▼                  │
│    observe()        find_object_distance()  choose_direction()  │
│    follow_color()                           move()              │
│                                                                  │
│  Key Features:                                                   │
│  • get_dist(x,y) - Single-point depth query at pixel coords     │
│  • find_object_distance(target) - GPT-4o locates, depth queries │
│  • follow_color(color) - RGB-based color tracking               │
│  • LiDAR-only navigation (no depth point cloud)                 │
└─────────────────────────────────────────────────────────────────┘
```

### Why This Pattern Works

Yahboom's tutorials (face following, color tracking, object detection) all use:
1. **RGB for visual features** - continuous streaming within USB 2.0 bandwidth
2. **Depth for distance queries** - `get_dist(x,y)` to get distance to specific pixels
3. **LiDAR for navigation** - gmapping, cartographer, Nav2 all use `/scan`

This avoids the USB 2.0 bandwidth limitation by NOT streaming RGB+Depth simultaneously.

### Yahboom Explorer Commands

| Command | Action |
|---------|--------|
| "What do you see?" | RGB image → GPT-4o vision description |
| "How far is the [object]?" | GPT-4o finds object → `get_dist(x,y)` |
| "Follow the red color" | RGB-based color tracking |
| "Start exploring" | LiDAR-based autonomous navigation |
| "Move forward" | Movement with LiDAR obstacle checking |
| "Status" | Position, distance traveled, discoveries |

### Error Alert System

The voice_mapper includes comprehensive error monitoring:

| Alert Type | Trigger | Response |
|------------|---------|----------|
| **Startup** | Missing sensors | Error beep + spoken warning |
| **Sensor Loss** | LiDAR/odom/camera stops | Warning beep + announcement |
| **Nav Failure** | Path blocked/timeout | Warning beep + explanation |
| **TTS Failure** | OpenAI API error | Falls back to beep patterns |

**Beep Patterns:**
- `success` - 1 pleasant 600Hz beep
- `warning` - 2 short 800Hz beeps  
- `error` - 3 fast 1200Hz beeps
- `attention` - 1 long 400Hz beep

## 📁 Script Overview

### Main Scripts

| Script | Purpose |
|--------|---------|
| [voice_mapper.py](scripts/voice_mapper.py) | **Main** - Voice + Vision + Nav2 + SLAM + Isaac VSLAM with multi-camera support |
| [yahboom_explorer.py](scripts/yahboom_explorer.py) | Alternative - Yahboom-style explorer with on-demand depth queries |
| [rosmaster_control.sh](scripts/rosmaster_control.sh) | **Master control** - Menu, service management, quick start |

### Support Scripts

| Script | Purpose |
|--------|---------|
| [llm_robot_brain.py](scripts/llm_robot_brain.py) | Core LLM integration (OpenAI/Anthropic/Gemini/Ollama) |
| [01_ssh_connect.sh](scripts/01_ssh_connect.sh) | SSH connection to Jetson Orin |
| [02_setup_depth_camera.sh](scripts/02_setup_depth_camera.sh) | Camera setup (install/launch) |
| [03_setup_voice_commands.sh](scripts/03_setup_voice_commands.sh) | Voice dependencies (install) |
| [04_setup_navigation.sh](scripts/04_setup_navigation.sh) | Nav2 SLAM & navigation |
| [05_autonomous_driving.sh](scripts/05_autonomous_driving.sh) | Line following & patrol (legacy) |
| [06_setup_llm_brain.sh](scripts/06_setup_llm_brain.sh) | LLM dependencies (install) |
| [07_setup_imu.sh](scripts/07_setup_imu.sh) | IMU sensor setup |

### Config Files

| File | Purpose |
|------|---------|
| [nav2_params.yaml](scripts/nav2_params.yaml) | Nav2 config for Ackerman steering |
| [voice_mapper.service](scripts/voice_mapper.service) | Systemd unit for auto-start |

### Quick Start Commands

```bash
# Run voice mapper (recommended)
./rosmaster_control.sh run

# Or run alternative explorer
./rosmaster_control.sh yahboom

# Install as auto-start service
sudo ./rosmaster_control.sh service install

# Interactive menu
./rosmaster_control.sh menu
```

### Configuration Files

| File | Location | Purpose |
|------|----------|---------|
| LLM Config | `~/.rosmaster_llm_config` | API keys and provider settings |
| LLM Env | `~/.rosmaster_llm_env` | Environment vars for systemd service |
| Nav Points | `~/rosmaster_a1_config/nav_points.yaml` | Named navigation waypoints |
| Nav2 Params | `~/robot_scripts/nav2_params.yaml` | Nav2 config (Regulated Pure Pursuit) |
| Maps | `~/maps/*.yaml` | Saved SLAM maps |
| Discovery Log | `~/exploration_logs/*.json` | Exploration discoveries |
| Yahboom Config | `~/yahboomcar_ros2_ws/.../largemodel/config/yahboom.yaml` | Built-in LLM settings |

### LLM Config File (~/.rosmaster_llm_config)
```bash
export LLM_PROVIDER='openai'
export LLM_MODEL='gpt-4o'
export OPENAI_API_KEY="sk-proj-..."
```

## 🧠 LLM Robot Brain

### Voice Mapper (Main Script)

The `voice_mapper.py` script provides the complete voice-controlled exploration system with SLAM mapping, Nav2 navigation, and Isaac VSLAM support:

```
┌─────────────────────────────────────────────────────────────────┐
│                    voice_mapper.py Architecture                  │
│                                                                  │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐         │
│  │   Whisper    │   │   GPT-4o     │   │    TTS       │         │
│  │  (48k→16kHz) │ → │  Brain+Vision│ → │  (nova, 3x)  │         │
│  │  PulseAudio  │   │              │   │  ffmpeg+aplay│         │
│  └──────────────┘   └──────────────┘   └──────────────┘         │
│         ↑                  ↓                   ↑                 │
│    Voice Input       JSON Actions         Error Alerts          │
│                           ↓                    ↑                 │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐         │
│  │   Camera     │   │    Nav2      │   │   Beep()     │         │
│  │ HP60C/OAK-D  │   │  Navigation  │   │  Alerts      │         │
│  │ (auto-detect)│   │  Pure Pursuit│   │  (ffmpeg)    │         │
│  └──────────────┘   └──────────────┘   └──────────────┘         │
│                           ↓                                      │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐         │
│  │ LiDAR SLAM   │   │  /cmd_vel    │   │   LiDAR      │         │
│  │ slam_toolbox │   │  Publisher   │   │   /scan      │         │
│  │              │   │              │   │  (720 pts)   │         │
│  └──────────────┘   └──────────────┘   └──────────────┘         │
│                                                                  │
│  ┌──────────────┐   ┌──────────────┐                            │
│  │ Isaac VSLAM  │   │  Multi-Camera│   Camera Types:            │
│  │ (OAK-D Pro)  │   │  Abstraction │   • HP60C (current)        │
│  │              │   │              │   • OAK-D Pro (VSLAM)      │
│  └──────────────┘   └──────────────┘   • RealSense              │
│                                                                  │
│  Features: Nav2 path planning, frontier exploration,            │
│  doorway detection, SLAM mapping, Isaac VSLAM, error alerts     │
└─────────────────────────────────────────────────────────────────┘
```

### LLM Response Format
```json
{
  "action": "move",
  "linear": 0.15,
  "angular": 0.0,
  "duration": 2.0,
  "speech": "Moving forward!"
}
```

### Available Actions

| Action | Parameters | Description |
|--------|------------|-------------|
| `move` | linear (±0.3), angular (±0.5), duration | Robot movement with obstacle checking |
| `look` | speech | Take picture and describe via GPT-4o vision |
| `get_dist` | target (object name) | Find object in RGB, query depth at location (Yahboom pattern) |
| `explore` | speech | Start autonomous exploration mode |
| `stop` | speech | Stop movement and exploration |
| `status` | speech | Report distance traveled and discoveries |
| `follow_color` | color (red/green/blue/yellow) | RGB-based color tracking |
| `speak` | speech | Voice response only |

### Movement Parameters

| Parameter | Range | Description |
|-----------|-------|-------------|
| `linear` | -0.3 to 0.3 m/s | Forward/backward speed |
| `angular` | -0.5 to 0.5 rad/s | Rotation speed (positive = left) |
| `duration` | 0 to 5 seconds | How long to move |

### Obstacle Avoidance

The explorer uses LiDAR for safety:
- **min_obstacle_dist**: 0.5m - Emergency stop distance
- **slow_dist**: 1.0m - Slow down distance
- **emergency_dist**: 0.3m - Absolute minimum clearance
- **Sectors monitored**: front, front_left, front_right, left, right, back
- **Angular range**: ±60° front cone for main detection

### Alternative: ROS2 Node Architecture

The original `llm_robot_brain.py` uses a ROS2 pub/sub architecture:

```
┌─────────────────────────────────────────────────────────────────┐
│                    LLM Robot Brain Architecture                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   Voice Input          Text Input         Camera Feed           │
│       │                    │                   │                 │
│       ▼                    ▼                   ▼                 │
│  ┌─────────┐         ┌─────────┐         ┌─────────┐            │
│  │ Speech  │         │  Text   │         │  Image  │            │
│  │to Text  │         │ Topic   │         │ Base64  │            │
│  └────┬────┘         └────┬────┘         └────┬────┘            │
│       │                   │                   │                  │
│       └───────────────────┼───────────────────┘                  │
│                           ▼                                      │
│                   ┌───────────────┐                              │
│                   │   LLM Brain   │  ← System Prompt with        │
│                   │ (GPT/Claude/  │    Robot Action Library      │
│                   │  Ollama)      │                              │
│                   └───────┬───────┘                              │
│                           │                                      │
│              JSON Response│{"thinking": "...",                   │
│                           │ "actions": [...],                    │
│                           │ "response": "..."}                   │
│                           ▼                                      │
│       ┌───────────────────┼───────────────────┐                  │
│       ▼                   ▼                   ▼                  │
│  ┌─────────┐        ┌─────────┐        ┌─────────┐              │
│  │ Action  │        │ Speech  │        │ Robot   │              │
│  │Executor │        │ Output  │        │ Actions │              │
│  └────┬────┘        └────┬────┘        └────┬────┘              │
│       │                   │                  │                   │
│       ▼                   ▼                  ▼                   │
│   /cmd_vel           TTS Engine        /robot_action            │
│   /navigate_to_pose                                              │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Setup LLM Brain

```bash
# Install LLM dependencies
./scripts/06_setup_llm_brain.sh install

# For local inference (no API key needed)
./scripts/06_setup_llm_brain.sh install-ollama

# Configure your API key
./scripts/06_setup_llm_brain.sh configure

# Create supporting nodes
./scripts/06_setup_llm_brain.sh setup

# Launch the LLM brain
./scripts/06_setup_llm_brain.sh launch
```

### Supported LLM Providers

| Provider | Models | API Key Required | Vision Support |
|----------|--------|------------------|----------------|
| OpenAI | GPT-4o, GPT-4 | Yes | ✅ |
| Anthropic | Claude Sonnet 4, Claude Opus 4 | Yes | ✅ |
| Ollama | Llama 3.2, Mistral | No (local) | ❌ |
| Google | Gemini Pro | Yes | ✅ |

### Example LLM Commands

With the LLM brain, you can use natural language:

| Simple Command | LLM-Enhanced Equivalent |
|----------------|-------------------------|
| "go to kitchen" | "Can you go to the kitchen and look around for me?" |
| "turn left" | "Turn about 90 degrees to your left please" |
| "follow me" | "Hey robot, come here and follow me around the house" |
| "stop" | "Hold on, wait right there!" |

### Interactive Chat Mode

```bash
# Start text-based chat with the robot
./scripts/06_setup_llm_brain.sh chat
```

## 🎥 3D Camera Setup

Setup and launch the depth camera:

```bash
# Install camera dependencies
./scripts/02_setup_depth_camera.sh install

# Check camera connection
./scripts/02_setup_depth_camera.sh check

# Launch camera
./scripts/02_setup_depth_camera.sh launch

# View in RViz2
./scripts/02_setup_depth_camera.sh view
```

**Environment Variables:**

- `CAMERA_TYPE`: `nuwa`, `astra`, or `realsense` (default: `nuwa`)

## 🎯 Isaac VSLAM (Visual SLAM) - OAK-D Pro Support

The `voice_mapper.py` supports **Isaac ROS Visual SLAM** for high-precision localization using stereo cameras. This provides better accuracy than wheel odometry alone, especially in feature-rich indoor environments.

### Supported VSLAM Cameras

| Camera | Status | VSLAM Quality | Notes |
|--------|--------|---------------|-------|
| **OAK-D Pro** | ✅ Recommended | Excellent | USB 3.0, IR illumination, built-in IMU |
| **Intel RealSense D435i** | ✅ Supported | Excellent | USB 3.0, built-in IMU |
| **Angstrong HP60C** | ❌ Not Suitable | - | USB 2.0 only, no stereo |

### Isaac VSLAM Installation (Jetson Orin Nano)

```bash
# Install Isaac ROS Visual SLAM
sudo apt update
sudo apt install ros-humble-isaac-ros-visual-slam

# For OAK-D cameras, install DepthAI ROS2 driver
sudo apt install ros-humble-depthai-ros

# For RealSense cameras
sudo apt install ros-humble-realsense2-camera
```

### Isaac VSLAM Features

- **GPU-Accelerated**: Uses NVIDIA cuVSLAM for 200Hz+ visual odometry on Jetson
- **Loop Closure**: Automatically detects revisited areas
- **Map Persistence**: Save and load visual maps for relocalization
- **IMU Fusion**: Optional IMU integration for VIO (Visual-Inertial Odometry)
- **3D Landmarks**: Builds sparse 3D point cloud of visual features

### Using VSLAM with Voice Commands

Once OAK-D Pro is connected and Isaac VSLAM is installed:

```bash
# Start voice mapper (auto-detects OAK-D Pro)
python3 voice_mapper.py
```

Voice commands for VSLAM:
- **"Start visual SLAM"** - Launches Isaac VSLAM with stereo camera
- **"Stop visual SLAM"** - Stops VSLAM tracking
- **"VSLAM status"** - Reports tracking status and path length
- **"Start mapping"** - Uses LiDAR SLAM (slam_toolbox) for 2D occupancy map

### SLAM Modes

| Mode | Command | Uses | Best For |
|------|---------|------|----------|
| **LiDAR SLAM** | "Start mapping" | slam_toolbox + /scan | 2D occupancy maps, navigation |
| **Visual SLAM** | "Start visual SLAM" | Isaac VSLAM + stereo camera | Precise localization, 3D landmarks |
| **Hybrid** | Both | LiDAR + VSLAM | Maximum accuracy |

### Camera Auto-Detection

The `voice_mapper.py` automatically detects which camera is connected:

```python
# Auto-detection based on ROS2 topics
if "/oak/rgb/image_raw" in topics:
    camera = CameraType.OAK_D_PRO  # VSLAM-capable
elif "/camera/color/image_raw" in topics:
    camera = CameraType.REALSENSE  # VSLAM-capable
else:
    camera = CameraType.HP60C      # LiDAR SLAM only
```

### Isaac VSLAM Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/visual_slam/tracking/odometry` | Odometry | High-frequency visual odometry |
| `/visual_slam/tracking/slam_path` | Path | VSLAM trajectory |
| `/visual_slam/status` | Status | Tracking status |
| `/visual_slam/vis/landmarks_cloud` | PointCloud2 | 3D visual landmarks |

### Isaac VSLAM Services

| Service | Description |
|---------|-------------|
| `/visual_slam/save_map` | Save visual map for later relocalization |
| `/visual_slam/load_map` | Load previously saved map |
| `/visual_slam/reset` | Reset VSLAM tracking |

## 📐 IMU Setup (Optional)

If your ROSMASTER A1 expansion board has an IMU (MPU6050/MPU9250/BNO055):

```bash
# Check if IMU is present
./scripts/07_setup_imu.sh check

# Install IMU dependencies
./scripts/07_setup_imu.sh install

# Setup IMU driver and sensor fusion
./scripts/07_setup_imu.sh setup

# Calibrate IMU (keep robot stationary!)
./scripts/07_setup_imu.sh calibrate

# Launch IMU driver
./scripts/07_setup_imu.sh launch

# Launch sensor fusion (EKF)
./scripts/07_setup_imu.sh fusion
```

### IMU Benefits

| Without IMU | With IMU |
|-------------|----------|
| Wheel odometry only | Fused odometry (more accurate) |
| Drift over time | Gyro corrects drift |
| No tilt detection | Knows if robot is tilted |
| Slip causes errors | IMU detects wheel slip |

### Supported IMUs

- **MPU6050** - 6-axis (accelerometer + gyroscope)
- **MPU9250** - 9-axis (+ magnetometer)
- **BNO055** - 9-axis with built-in sensor fusion
- **ICM20948** - 9-axis

## 🎤 Voice Control

Enable voice commands for the robot:

```bash
# Install voice dependencies
./scripts/03_setup_voice_commands.sh install

# Test microphone
./scripts/03_setup_voice_commands.sh test-mic

# Test speaker
./scripts/03_setup_voice_commands.sh test-speak

# Create voice control scripts
./scripts/03_setup_voice_commands.sh setup

# Launch voice control
./scripts/03_setup_voice_commands.sh launch
```

### Available Voice Commands

| Command | Action |
|---------|--------|
| "go forward" | Move forward |
| "turn left/right" | Turn in direction |
| "stop" | Stop movement |
| "go to kitchen" | Navigate to kitchen |
| "follow me" | Start face following |
| "patrol" | Start line patrol |
| "hello robot" | Greeting response |

## 🗺️ Navigation

### Creating a Map (SLAM)

```bash
# Start SLAM mapping mode
./scripts/04_setup_navigation.sh slam

# In another terminal, drive the robot
./scripts/04_setup_navigation.sh teleop

# When done, save the map
./scripts/04_setup_navigation.sh save-map my_home_map
```

### Using Navigation

```bash
# Start navigation with saved map
./scripts/04_setup_navigation.sh nav ~/maps/my_home_map.yaml
```

### Configure Navigation Points

Edit `~/rosmaster_a1_config/nav_points.yaml`:

```yaml
navigation_points:
  home: [0.0, 0.0, 0.0]
  kitchen: [2.0, 1.0, 1.57]
  bedroom: [1.0, 3.0, 3.14]
  living_room: [3.0, -1.0, 0.0]
```

## 🚗 Autonomous Driving

### Line Following

```bash
# Setup autonomous scripts
./scripts/05_autonomous_driving.sh setup

# Start line following (colors: red, green, blue, yellow)
./scripts/05_autonomous_driving.sh line green
```

### Autonomous Patrol

```bash
# Start patrol between waypoints
./scripts/05_autonomous_driving.sh patrol
```

### Obstacle Avoidance

```bash
# Start obstacle avoidance mode
./scripts/05_autonomous_driving.sh obstacle
```

## 🔧 Configuration

### Robot Network Settings

Default settings (can be overridden with environment variables):

```bash
export ROBOT_IP="192.168.2.1"
export ROBOT_USER="yahboom"
export ROBOT_PASSWORD="yahboom"
```

### ROS2 Domain

```bash
export ROS_DOMAIN_ID=0
```

## 📊 Monitoring

### Check System Status

```bash
./scripts/rosmaster_control.sh status
```

### View ROS2 Topics

```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /scan
```

### View Camera Feed

```bash
ros2 run rqt_image_view rqt_image_view
```

## 🛑 Stopping the Robot

```bash
# Stop all processes
./scripts/rosmaster_control.sh stop

# Emergency stop (publish zero velocity)
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{}"
```

## 📚 Documentation Links

- [Yahboom ROSMASTER A1 Official Page](https://www.yahboom.net/study/ROSMASTER-A1)
- [GitHub Repository](https://github.com/YahboomTechnology/ROSMASTER-A1)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)

## 🎬 Video Tutorials

- [Jetson Orin Installation](https://youtu.be/rp2YeOTvKZk)
- [Jetson Nano Installation](https://youtu.be/eAUPQlqZ3xs)
- [Raspberry Pi Installation](https://youtu.be/6Cb6laEvzPg)

## ⚠️ Troubleshooting

### Audio / Voice Issues

**Microphone not detected or "Invalid number of channels":**
The USB Audio device may show 0 input channels when accessed directly. Use PulseAudio instead:
```python
# In voice_mapper.py, microphone selection prioritizes PulseAudio
# PulseAudio device (index 32) has 32 virtual input channels
# Direct USB device (index 0) may fail with "Invalid number of channels"
```

```bash
# List all audio devices
python3 -c "import pyaudio; p=pyaudio.PyAudio(); [print(f'{i}: {p.get_device_info_by_index(i)}') for i in range(p.get_device_count())]"
```

**Sample rate errors ("Invalid sample rate"):**
The USB mic only supports 48kHz/44.1kHz, NOT 16kHz!
```bash
# Test at native sample rate
arecord -D plughw:0,0 -f S16_LE -r 48000 -c 1 -d 3 /tmp/test.wav
aplay /tmp/test.wav
```

Scripts automatically resample 48kHz→16kHz using scipy for Whisper compatibility.

**TTS not playing:**
```bash
# Test speaker output
speaker-test -D plughw:0,0 -c 1 -t sine -f 440 -l 1

# Install mpv for TTS playback
sudo apt-get install mpv ffmpeg

# TTS uses ffmpeg to convert MP3→WAV, then aplay or mpv
```

### Camera Issues

**Dark/black images from HP60C camera:**

The Angstrong HP60C camera produces very dark images due to firmware gain settings:
- Camera sets gain=4 (very low)
- Images have mean brightness ~4/255
- SDK is proprietary with encrypted config files
- No exposure/gain controls available in SDK

**Workaround - software brightness enhancement:**
```python
import cv2
import numpy as np

# Apply brightness enhancement
enhanced = cv2.convertScaleAbs(image, alpha=15.0, beta=30)

# Or use histogram equalization
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
equalized = cv2.equalizeHist(gray)
```

**Camera config files location:**
```
~/yahboomcar_ros2_ws/software/library_ws/src/ascamera/configurationfiles/
├── hp60c_v2_00_20230704_configEncrypt.json  # Encrypted - cannot modify
└── ... other configs
```

**Camera launch file:**
```bash
# Launch from correct directory (required for config files)
cd ~/yahboomcar_ros2_ws/software/library_ws/src/ascamera
ros2 launch ascamera hp60c.launch.py
```

**Camera topic verification:**
```bash
# Check camera is publishing
ros2 topic hz /ascamera_hp60c/camera_publisher/rgb0/image
# Should show ~20 Hz

# View image info
ros2 topic echo /ascamera_hp60c/camera_publisher/rgb0/image --field header --once
```

### LiDAR Issues

**Wrong LiDAR driver:**
This robot uses **SLLidar C1**, NOT YDLidar! Use:
```bash
# Correct launch
ros2 launch sllidar_ros2 sllidar_c1_launch.py

# NOT: ros2 launch ydlidar_ros2_driver ...
```

**LiDAR data verification:**
```bash
# Check topic
ros2 topic hz /scan  # Should be ~10 Hz
ros2 topic echo /scan --field ranges --once | head -20
# Should show 720 range values
```

**LiDAR timeout errors:**
Device is busy. Stop other processes:
```bash
pkill -f sllidar
```

### Motor/Movement Issues

**Robot not moving despite `/cmd_vel` commands:**
1. Verify motor driver is running:
   ```bash
   ros2 node list | grep driver
   # Should show: /driver_node
   ```

2. Check odometry is changing:
   ```bash
   ros2 topic echo /odom --field pose.pose.position.x
   ```

3. Test direct motor command:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
   # Robot should move forward
   
   # Stop with:
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once
   ```

**Servo initialization errors:**
Set servo environment variables:
```bash
export INIT_SERVO_S1=90
export INIT_SERVO_S2=90
```

### IMU Issues

**Verify IMU is working:**
```bash
# Check IMU topics
ros2 topic list | grep imu
# Should show: /imu/data, /imu/data_raw, /imu/mag

# Check data flow
ros2 topic hz /imu/data  # Should be ~100 Hz
ros2 topic echo /imu/data --field orientation --once
```

### Threading Issues in voice_mapper.py

**"ValueError: generator already executing" error:**

This occurs when `rclpy.spin_once()` is called from multiple threads simultaneously. The voice_mapper.py uses threading for voice input while the main thread handles ROS spinning.

**Fix**: Ensure only the main thread calls `rclpy.spin_once()`. Movement commands should use a separate timer or queue system.

### Process Management

**Multiple instances running:**
```bash
# Kill all LLM/voice processes
pkill -9 -f 'python3.*llm\|python3.*voice\|python3.*openai\|python3.*explorer'

# Check what's running
ps aux | grep python3

# Kill specific script
pkill -f voice_mapper
```

**Clean ROS2 state:**
```bash
# Reset ROS2 daemon
ros2 daemon stop
ros2 daemon start
```

### Network/SSH Issues

**Connection timeout:**
```bash
# Check robot IP (may have changed via DHCP)
# Default is 192.168.7.250 but verify with router

# Ping test
ping 192.168.7.250

# If using WiFi, ensure same network as robot
```

### Yahboom's Built-in Voice System

The robot comes with Yahboom's `largemodel` package which:
- Uses Chinese wake word ("Hello Yahboom")
- Connects to Chinese APIs (iFlytek, Baidu)
- Requires `/dev/myspeech` serial device

Our `voice_mapper.py` **replaces** this with English OpenAI-based system.

**Switching languages in Yahboom's system:**
Edit `~/yahboomcar_ros2_ws/yahboomcar_ws/src/largemodel/config/yahboom.yaml`:
```yaml
asr:
  ros__parameters:
    language: 'en'                      # 'zh' for Chinese
    regional_setting: "international"   # "China" for domestic
```

## 📝 License

This project is for educational purposes with the Yahboom ROSMASTER A1 robot.

## 🤝 Support

- **Technical Email**: support@yahboom.com
- **WhatsApp**: +86 18682378128
