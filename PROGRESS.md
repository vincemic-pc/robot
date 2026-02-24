# Autonomous Robot Explorer - Progress Tracker

## Project Status: 🔄 In Progress

**Last Updated:** February 24, 2026

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

## 🔄 In Progress

### OAK-D Pro Camera + Isaac VSLAM
- **Status:** ⚠️ Pending on-robot installation
- **Camera:** OAK-D Pro (Luxonis) connected via USB 3.0
- **Driver:** `depthai-ros` package (to be installed on robot)
- **Topics:** `/oak/rgb/image_raw`, `/oak/stereo/image_raw`, `/oak/left/image_rect`, `/oak/right/image_rect`
- **VSLAM:** Isaac ROS Visual SLAM support in code, pending `ros-humble-isaac-ros-visual-slam` apt install
- **Action Required:** SSH to robot, run `02_setup_depth_camera.sh` to install OAK-D Pro drivers

---

## ❌ Blocked / Not Started

### 1. Full Nav2 Integration
- [ ] Test Nav2 with updated params
- [ ] Verify OAK-D Pro depth point cloud in costmap
- [ ] Test autonomous navigation through doorways
- **Blocked by:** OAK-D Pro driver installation on robot

### 2. SLAM Mapping
- [ ] Integrate slam_toolbox
- [ ] Room discovery and labeling
- [ ] Map persistence

### 3. Vision Intelligence
- [ ] GPT-4o vision for scene understanding
- [ ] Object recognition
- [ ] Landmark detection
- **Partially blocked by:** OAK-D Pro driver installation

### 4. Isaac VSLAM Activation
- [ ] Install `ros-humble-isaac-ros-visual-slam` on Jetson
- [ ] Install `ros-humble-depthai-ros` on Jetson
- [ ] Test stereo VSLAM tracking
- [ ] Test hybrid SLAM mode (LiDAR + VSLAM)
- **Blocked by:** SSH installation step (Phase 5 of camera transition plan)

---

## 🔧 Hardware Status

| Component | Status | Notes |
|-----------|--------|-------|
| **LiDAR** | ✅ Working | SLLidar C1, `/scan`, 720 pts @ 10Hz |
| **Camera** | ⚠️ Pending | OAK-D Pro (Luxonis), driver install needed on robot |
| **IMU** | ✅ Working | `/imu/data` (Madgwick filtered) |
| **Motors** | ✅ Working | `/cmd_vel` → `/driver_node` |
| **Audio Out** | ✅ Working | aplay, speaker-test confirmed |
| **Audio In** | ✅ Working | PulseAudio index 32 |

---

## 📋 Next Steps

### Immediate (On-Robot SSH)
1. [ ] Run `02_setup_depth_camera.sh` to install OAK-D Pro drivers
2. [ ] Install Isaac VSLAM: `sudo apt install ros-humble-isaac-ros-visual-slam`
3. [ ] Deploy updated scripts: `scp scripts/* jetson@192.168.7.250:~/robot_scripts/`
4. [ ] Verify OAK-D Pro topics: `/oak/rgb/image_raw`, `/oak/stereo/image_raw`

### Short Term
1. [ ] Test Nav2 navigation with OAK-D Pro depth in costmap
2. [ ] Test Isaac VSLAM stereo tracking
3. [ ] Tune costmap inflation radius
4. [ ] Add room labeling via voice

### Medium Term
1. [ ] SLAM map building and saving
2. [ ] Named waypoint navigation
3. [ ] Multi-room exploration strategy
4. [ ] Hybrid SLAM mode (LiDAR + VSLAM) testing

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

**Feb 24, 2026:**
- Completed camera transition from HP60C to OAK-D Pro (code side)
- Removed all HP60C code paths from voice_mapper.py, replaced with CameraConfig abstraction
- Updated nav2_params.yaml, start_robot.sh, voice_mapper.service, 02_setup_depth_camera.sh, rosmaster_control.sh
- Fixed 3 service bugs: WorkingDirectory, DISPLAY var, source order
- Deleted hp60c_lowres.launch.py, deprecated yahboom_explorer.py
- Isaac VSLAM support ready in code (pending on-robot apt install)
- Remaining: SSH to robot to install depthai-ros and Isaac VSLAM packages

**Nov 29, 2025:**
- Discovered depth camera USB communication failure
- Updated Nav2 params for voxel_layer + depth integration
- Improved LiDAR doorway detection algorithm
- User reconnecting camera to different USB port - awaiting confirmation
