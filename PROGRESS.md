# Autonomous Robot Explorer - Progress Tracker

## Project Status: 🔄 In Progress

**Last Updated:** February 24, 2026
**Latest Milestone:** OAK-D Pro camera fully deployed and verified on robot (Plan 002 complete)

---

## ✅ Completed Features

### 1. Voice Control System
- [x] Whisper STT integration (48kHz → 16kHz resampling)
- [x] PulseAudio device configuration (index 32)
- [x] OpenAI TTS with ffmpeg → aplay pipeline
- [x] Beep fallback when TTS fails

### 2. LiDAR Navigation
- [x] SLLidar C1 integration (`/scan` topic, 720 points)
- [x] BEST_EFFORT QoS configuration
- [x] Percentile-based obstacle detection (10th percentile, not min)
- [x] Sector blockage threshold (>30% blocked, not just 1 point)
- [x] `inf` values treated as open space (10.0m)
- [x] **Doorway/Gap Detection Algorithm:**
  - Detects gaps where distance increases by >1.0m
  - Minimum gap width: 0.5m
  - Maximum gap distance: 8.0m
  - Stores in `self.detected_gaps` list

### 3. Exploration Logic
- [x] Reactive exploration loop with doorway-first priority
- [x] Random exploration walk that uses detected gaps
- [x] Safety thresholds:
  - min_obstacle_dist: 0.5m
  - slow_dist: 1.0m
  - emergency_dist: 0.3m

### 4. Error Alert System
- [x] `beep()` function with ffmpeg audio generation
- [x] `_sensor_monitor_loop()` for health monitoring
- [x] Startup sensor checks with alerts
- [x] `speak()` with beep fallback

### 5. Nav2 Configuration
- [x] Updated `nav2_params.yaml` with:
  - voxel_layer for 3D obstacle detection
  - SmacPlanner2D for better constrained spaces
  - `inf_is_valid: true` for doorway handling
  - Configured for both LiDAR and depth camera

### 6. Camera Transition: HP60C → OAK-D Pro (February 2026)
- [x] Removed all HP60C code paths from `voice_mapper.py`
- [x] Replaced `CameraType.HP60C` enum with `CameraConfig` abstraction (OAK-D Pro + RealSense)
- [x] Updated `nav2_params.yaml` — all `/ascamera_hp60c/` topics → `/oak/` topics
- [x] Updated `start_robot.sh` — launches `depthai-ros` instead of `ascamera` package
- [x] Updated `voice_mapper.service` — fixed 3 service bugs (WorkingDirectory, DISPLAY, sourcing)
- [x] Updated `02_setup_depth_camera.sh` — OAK-D Pro + depthai-ros installation
- [x] Updated `rosmaster_control.sh` — removed HP60C references, added OAK-D Pro status checks
- [x] Deleted `hp60c_lowres.launch.py` (obsolete launch file)
- [x] Added deprecation header to `yahboom_explorer.py`
- [x] Updated `.github/copilot-instructions.md` — full OAK-D Pro documentation
- [x] Isaac VSLAM support enabled in code (pending `ros-humble-isaac-ros-visual-slam` installation on robot)

---

## ✅ Recently Completed (Plan 002 — Robot Deployment & Verification)

### OAK-D Pro Camera — Fully Deployed
- **Status:** ✅ Working — all topics publishing
- **Camera:** OAK-D-PRO (Luxonis) on USB 2.0 (5-10 fps, 1280x720)
- **Driver:** DepthAI ROS2 v2.12.2 installed
- **RGB topic:** `/oak/rgb/image_raw` ✅
- **Depth topic:** `/oak/stereo/image_raw` ✅
- **TF base frame:** `oak-d-base-frame` ✅
- **Camera orientation:** Right-side-up (no flip needed — removed roll=π from start_robot.sh)
- **Point cloud:** `/oak/points` NOT published by default (needs separate composable node — follow-up item)
- **Stereo topics:** NOT published by default (fixed with publish params in launch)

### Isaac VSLAM — Installed & Verified
- **Status:** ✅ Installed and verified
- **Package:** `ros-humble-isaac-ros-visual-slam` v3.2.6 with cuVSLAM v12.6
- **Odometry topic:** `/visual_slam/tracking/odometry` ✅

### Service Auto-Start — Working
- **Status:** ✅ `voice_mapper.service` running, auto-starts on boot
- **Startup message:** "Mapper ready with 3 sensors" confirmed
- **All key topics:** /scan, /odom, /oak/rgb/image_raw, /cmd_vel, /imu/data, /visual_slam/tracking/odometry

### Robot Cleanup
- Removed ascamera build/install artifacts from `~/yahboomcar_ros2_ws/software/library_ws/`
- Active scripts confirmed clean of HP60C/ascamera references
- Disk: 67% used (28 GB free on 88 GB)

---

## 🔄 In Progress / Not Started

### 1. Full Nav2 Integration
- [ ] Test Nav2 with updated params
- [ ] Set up point cloud topic for costmap (needs `depth_image_proc` composable node)
- [ ] Test autonomous navigation through doorways

### 2. SLAM Mapping
- [ ] Integrate slam_toolbox
- [ ] Room discovery and labeling
- [ ] Map persistence

### 3. Vision Intelligence
- [ ] GPT-4o vision for scene understanding
- [ ] Object recognition
- [ ] Landmark detection

### 4. USB 3.0 Port Migration
- [ ] Move OAK-D Pro to USB 3.0 port for better FPS (currently 5-10 fps on USB 2.0)
- [ ] Verify 640x480@30fps after port migration

### 5. Hybrid SLAM Mode
- [ ] Test hybrid SLAM mode (LiDAR + VSLAM)
- [ ] Tune VSLAM/LiDAR fusion parameters

---

## 🔧 Hardware Status

| Component | Status | Notes |
|-----------|--------|-------|
| **LiDAR** | ✅ Working | SLLidar C1, `/scan`, 720 pts @ 10Hz |
| **Camera** | ✅ Working | OAK-D-PRO (Luxonis), DepthAI v2.12.2, USB 2.0 (5-10 fps) |
| **VSLAM** | ✅ Installed | Isaac VSLAM v3.2.6, cuVSLAM v12.6 |
| **IMU** | ✅ Working | `/imu/data` (Madgwick filtered) |
| **Motors** | ✅ Working | `/cmd_vel` → `/driver_node` |
| **Audio Out** | ✅ Working | aplay, speaker-test confirmed |
| **Audio In** | ✅ Working | PulseAudio index 32 |

---

## 📋 Next Steps

### Immediate
1. [ ] Move OAK-D Pro to USB 3.0 port (currently USB 2.0, limiting to 5-10 fps)
2. [ ] Set up `/oak/points` PointCloud2 topic via `depth_image_proc` composable node
3. [ ] Test Nav2 navigation with OAK-D Pro depth in costmap
4. [ ] Test Isaac VSLAM stereo tracking via voice commands

### Short Term
1. [ ] Tune costmap inflation radius with depth data
2. [ ] Add room labeling via voice
3. [ ] Test hybrid SLAM mode (LiDAR + VSLAM)
4. [ ] GPT-4o vision scene understanding

### Medium Term
1. [ ] SLAM map building and saving
2. [ ] Named waypoint navigation
3. [ ] Multi-room exploration strategy
4. [ ] Object recognition and landmark detection

---

## 🐛 Known Issues

1. **Threading:** Don't call `rclpy.spin_once()` from multiple threads
2. **Audio channels:** Direct USB mic fails - must use PulseAudio

---

## 📁 Key Files

| File | Purpose |
|------|---------|
| `scripts/voice_mapper.py` | Main explorer node |
| `scripts/nav2_params.yaml` | Nav2 configuration |
| `scripts/rosmaster_control.sh` | Service management |
| `scripts/start_robot.sh` | Hardware launcher (LiDAR, camera, IMU, driver) |
| `scripts/llm_robot_brain.py` | LLM action library |

---

## 🔌 Connection Info

```bash
# Robot connection
ssh jetson@192.168.7.250
# Password: yahboom
# ROS_DOMAIN_ID=62

# Quick test
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=62
ros2 topic list
```

---

## 📝 Session Notes

**Feb 24, 2026 (Plan 002 — Robot Deployment & Verification):**
- Deployed all updated scripts to robot via SCP
- Installed DepthAI ROS2 v2.12.2 (OAK-D Pro driver)
- Installed Isaac VSLAM v3.2.6 with cuVSLAM v12.6
- Confirmed OAK-D-PRO model on USB 2.0 (5-10 fps, 1280x720)
- Discovery results:
  - RGB: `/oak/rgb/image_raw` ✅ (no deviation)
  - Depth: `/oak/stereo/image_raw` ✅ (no deviation)
  - Point cloud: `/oak/points` NOT published (needs separate composable node)
  - Stereo: NOT published by default (fixed with publish params)
  - TF base frame: `oak-d-base-frame` ✅
  - Camera orientation: Right-side-up (removed roll=π from start_robot.sh)
- Fixed `--remap` syntax bug in VSLAM launch (correct: `--ros-args -r`)
- Service verified: `voice_mapper.service` auto-starts, "Mapper ready with 3 sensors"
- Cleaned ascamera build artifacts from robot
- 28 GB free disk space (67% used)

**Feb 24, 2026 (Plan 001 — Camera Transition Code Changes):**
- Completed camera transition from HP60C to OAK-D Pro (code side)
- Removed all HP60C code paths from voice_mapper.py, replaced with CameraConfig abstraction
- Updated nav2_params.yaml, start_robot.sh, voice_mapper.service, 02_setup_depth_camera.sh, rosmaster_control.sh
- Fixed 3 service bugs: WorkingDirectory, DISPLAY var, source order
- Deleted hp60c_lowres.launch.py, deprecated yahboom_explorer.py
- Isaac VSLAM support ready in code (pending on-robot apt install)

**Nov 29, 2025:**
- Discovered depth camera USB communication failure
- Updated Nav2 params for voxel_layer + depth integration
- Improved LiDAR doorway detection algorithm
- User reconnecting camera to different USB port - awaiting confirmation
