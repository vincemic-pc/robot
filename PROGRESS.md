# Autonomous Robot Explorer - Progress Tracker

## Project Status: 🔄 In Progress

**Last Updated:** November 29, 2025

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

---

## 🔄 In Progress

### Depth Camera Integration
- **Status:** ⚠️ USB 3.0 port required
- **Problem:** Camera connected to USB 2.0 port - insufficient bandwidth for depth
- **Error logs:**
  ```
  [ERROR] [CameraHp60c.cpp] [195] [startStreaming] not support media type
  ```
- **Current State:** Topics advertised but depth NOT streaming
- **Root Cause:** HP60C depth camera needs USB 3.0 (SuperSpeed) for depth stream
- **Camera ID:** `3482:6723 NOVATEK ASJ ZNX_NVT` on Bus 001 (USB 2.0)
- **Action Required:** Connect camera to USB 3.0 port (blue port, or USB 3.0 hub)

---

## ❌ Blocked / Not Started

### 1. Full Nav2 Integration
- [ ] Test Nav2 with updated params
- [ ] Verify depth camera point cloud in costmap
- [ ] Test autonomous navigation through doorways
- **Blocked by:** Depth camera not working

### 2. SLAM Mapping
- [ ] Integrate slam_toolbox
- [ ] Room discovery and labeling
- [ ] Map persistence

### 3. Vision Intelligence
- [ ] GPT-4o vision for scene understanding
- [ ] Object recognition
- [ ] Landmark detection
- **Partially blocked by:** Camera issues

---

## 🔧 Hardware Status

| Component | Status | Notes |
|-----------|--------|-------|
| **LiDAR** | ✅ Working | SLLidar C1, `/scan`, 720 pts @ 10Hz |
| **RGB Camera** | ✅ Working | HP60C, `/ascamera_hp60c/.../rgb0/image` |
| **Depth Camera** | ❌ Failing | USB errors, reconnecting to new port |
| **IMU** | ✅ Working | `/imu/data` (Madgwick filtered) |
| **Motors** | ✅ Working | `/cmd_vel` → `/driver_node` |
| **Audio Out** | ✅ Working | aplay, speaker-test confirmed |
| **Audio In** | ✅ Working | PulseAudio index 32 |

---

## 📋 Next Steps

### Immediate (After Camera Reconnection)
1. [ ] Verify depth camera on new USB port
2. [ ] Check for depth topics: `/ascamera_hp60c/.../depth0/points`
3. [ ] Test depth data quality
4. [ ] Restart voice_mapper service

### Short Term
1. [ ] Test Nav2 navigation through doorways
2. [ ] Tune costmap inflation radius
3. [ ] Add room labeling via voice

### Medium Term
1. [ ] SLAM map building and saving
2. [ ] Named waypoint navigation
3. [ ] Multi-room exploration strategy

---

## 🐛 Known Issues

1. **Camera darkness:** HP60C RGB images very dark (mean ~4/255) - apply 10-20x enhancement
2. **Threading:** Don't call `rclpy.spin_once()` from multiple threads
3. **Audio channels:** Direct USB mic fails - must use PulseAudio

---

## 📁 Key Files

| File | Purpose |
|------|---------|
| `scripts/voice_mapper.py` | Main explorer node (~2039 lines) |
| `scripts/nav2_params.yaml` | Nav2 configuration |
| `scripts/rosmaster_control.sh` | Service management |
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

**Nov 29, 2025:**
- Discovered depth camera USB communication failure
- Updated Nav2 params for voxel_layer + depth integration
- Improved LiDAR doorway detection algorithm
- User reconnecting camera to different USB port - awaiting confirmation
